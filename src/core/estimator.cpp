// eKalibr, Copyright 2024, the School of Geodesy and Geomatics (SGG), Wuhan University, China
// https://github.com/Unsigned-Long/eKalibr.git
// Author: Shuolong Chen (shlchen@whu.edu.cn)
// GitHub: https://github.com/Unsigned-Long
//  ORCID: 0000-0002-5283-9057
// Purpose: See .h/.hpp file.
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * The names of its contributors can not be
//   used to endorse or promote products derived from this software without
//   specific prior written permission.
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "core/estimator.h"
#include "core/calib_param_mgr.h"
#include "ctraj/core/trajectory_estimator.h"
#include "spdlog/spdlog.h"
#include "util/utils_tpl.hpp"
#include "factor/imu_gyro_factor.hpp"
#include <sensor/imu_intrinsic.h>
#include "factor/hand_eye_rot_align_factor.hpp"

namespace ns_ekalibr {
std::shared_ptr<ceres::EigenQuaternionManifold> Estimator::QUATER_MANIFOLD(
    new ceres::EigenQuaternionManifold());
std::shared_ptr<ceres::SphereManifold<3>> Estimator::GRAVITY_MANIFOLD(
    new ceres::SphereManifold<3>());

Estimator::Estimator(SplineBundleType::Ptr splines, CalibParamManager::Ptr calibParamManager)
    : ceres::Problem(DefaultProblemOptions()),
      splines(std::move(splines)),
      parMagr(std::move(calibParamManager)) {}

Estimator::Ptr Estimator::Create(const SplineBundleType::Ptr &splines,
                                 const CalibParamManager::Ptr &calibParamManager) {
    return std::make_shared<Estimator>(splines, calibParamManager);
}

ceres::Problem::Options Estimator::DefaultProblemOptions() {
    return ns_ctraj::TrajectoryEstimator<Configor::Prior::SplineOrder>::DefaultProblemOptions();
}

ceres::Solver::Options Estimator::DefaultSolverOptions(int threadNum, bool toStdout, bool useCUDA) {
    auto defaultSolverOptions =
        ns_ctraj::TrajectoryEstimator<Configor::Prior::SplineOrder>::DefaultSolverOptions(
            threadNum, toStdout, useCUDA);
    if (!useCUDA) {
        defaultSolverOptions.linear_solver_type = ceres::DENSE_SCHUR;
    }
    defaultSolverOptions.trust_region_strategy_type = ceres::DOGLEG;
    return defaultSolverOptions;
}

ceres::Solver::Summary Estimator::Solve(const ceres::Solver::Options &options) {
    ceres::Solver::Summary summary;
    ceres::Solve(options, this, &summary);
    return summary;
}

Eigen::MatrixXd Estimator::GetHessianMatrix(const std::vector<double *> &consideredParBlocks,
                                            int numThread) {
    // remove params that are not involved
    ceres::Problem::EvaluateOptions evalOpt;
    evalOpt.parameter_blocks = consideredParBlocks;
    evalOpt.num_threads = numThread;

    // evaluate
    ceres::CRSMatrix jacobianCRSMatrix;
    this->Evaluate(evalOpt, nullptr, nullptr, nullptr, &jacobianCRSMatrix);

    // obtain hessian matrix and residual vector
    Eigen::MatrixXd JMat = CRSMatrix2EigenMatrix(&jacobianCRSMatrix);

    Eigen::MatrixXd HMat = JMat.transpose() * JMat;

    return HMat;
}

void Estimator::PrintParameterInfo() const {
    std::vector<double *> parameterBlocks;
    this->GetParameterBlocks(&parameterBlocks);
    int totalParamBlocks = parameterBlocks.size();

    int numOptimizedParamBlock = 0;
    int numOptimizedParameter = 0;
    for (const auto &paramBlock : parameterBlocks) {
        if (!IsParameterBlockConstant(paramBlock)) {
            ++numOptimizedParamBlock;
            numOptimizedParameter += this->ParameterBlockSize(paramBlock);
        }
    }
    spdlog::info(
        "Total number of parameter blocks: {}, with {} blocks ({} params) that are optimized",
        totalParamBlocks, numOptimizedParamBlock, numOptimizedParameter);
}

void Estimator::SetRefIMUParamsConstant() {
    auto SO3_BiToBr = parMagr->EXTRI.SO3_BiToBr.at(Configor::DataStream::RefIMUTopic).data();
    if (this->HasParameterBlock(SO3_BiToBr)) {
        this->SetParameterBlockConstant(SO3_BiToBr);
    }

    auto POS_BiInBr = parMagr->EXTRI.POS_BiInBr.at(Configor::DataStream::RefIMUTopic).data();
    if (this->HasParameterBlock(POS_BiInBr)) {
        this->SetParameterBlockConstant(POS_BiInBr);
    }

    auto TO_BiToBr = &parMagr->TEMPORAL.TO_BiToBr.at(Configor::DataStream::RefIMUTopic);
    if (this->HasParameterBlock(TO_BiToBr)) {
        this->SetParameterBlockConstant(TO_BiToBr);
    }
}

void Estimator::AddRdKnotsData(std::vector<double *> &paramBlockVec,
                               const Estimator::SplineBundleType::RdSplineType &spline,
                               const Estimator::SplineMetaType &splineMeta,
                               bool setToConst) {
    // for each segment
    for (const auto &seg : splineMeta.segments) {
        // the factor 'seg.dt * 0.5' is the treatment for numerical accuracy
        auto idxMaster = spline.ComputeTIndex(seg.t0 + seg.dt * 0.5).second;

        // from the first control point to the last control point
        for (std::size_t i = idxMaster; i < idxMaster + seg.NumParameters(); ++i) {
            auto *data = const_cast<double *>(spline.GetKnot(static_cast<int>(i)).data());

            this->AddParameterBlock(data, 3);

            paramBlockVec.push_back(data);
            // set this param block to be constant
            if (setToConst) {
                this->SetParameterBlockConstant(data);
            }
        }
    }
}

void Estimator::AddSo3KnotsData(std::vector<double *> &paramBlockVec,
                                const Estimator::SplineBundleType::So3SplineType &spline,
                                const Estimator::SplineMetaType &splineMeta,
                                bool setToConst) {
    // for each segment
    for (const auto &seg : splineMeta.segments) {
        // the factor 'seg.dt * 0.5' is the treatment for numerical accuracy
        auto idxMaster = spline.ComputeTIndex(seg.t0 + seg.dt * 0.5).second;

        // from the first control point to the last control point
        for (std::size_t i = idxMaster; i < idxMaster + seg.NumParameters(); ++i) {
            auto *data = const_cast<double *>(spline.GetKnot(static_cast<int>(i)).data());
            // the local parameterization is very important!!!
            this->AddParameterBlock(data, 4, QUATER_MANIFOLD.get());

            paramBlockVec.push_back(data);
            // set this param block to be constant
            if (setToConst) {
                this->SetParameterBlockConstant(data);
            }
        }
    }
}

Eigen::MatrixXd Estimator::CRSMatrix2EigenMatrix(const ceres::CRSMatrix *jacobian_crs_matrix) {
    Eigen::MatrixXd J(jacobian_crs_matrix->num_rows, jacobian_crs_matrix->num_cols);
    J.setZero();

    std::vector<int> jacobian_crs_matrix_rows, jacobian_crs_matrix_cols;
    std::vector<double> jacobian_crs_matrix_values;
    jacobian_crs_matrix_rows = jacobian_crs_matrix->rows;
    jacobian_crs_matrix_cols = jacobian_crs_matrix->cols;
    jacobian_crs_matrix_values = jacobian_crs_matrix->values;

    int cur_index_in_cols_and_values = 0;
    // rows is a num_rows + 1 sized array
    int row_size = static_cast<int>(jacobian_crs_matrix_rows.size()) - 1;
    // outer loop traverse rows, inner loop traverse cols and values
    for (int row_index = 0; row_index < row_size; ++row_index) {
        while (cur_index_in_cols_and_values < jacobian_crs_matrix_rows[row_index + 1]) {
            J(row_index, jacobian_crs_matrix_cols[cur_index_in_cols_and_values]) =
                jacobian_crs_matrix_values[cur_index_in_cols_and_values];
            cur_index_in_cols_and_values++;
        }
    }
    return J;
}

std::optional<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> Estimator::InertialVelIntegration(
    const std::vector<IMUFrame::Ptr> &data,
    const std::string &imuTopic,
    double sTimeByBi,
    double eTimeByBi) const {
    auto vecMatSeq = InertialIntegrationBase(data, imuTopic, sTimeByBi, eTimeByBi);
    if (vecMatSeq.first.size() < 2 || vecMatSeq.second.size() < 2) {
        // invalid integration data
        return {};
    }
    return std::pair<Eigen::Vector3d, Eigen::Matrix3d>{TrapIntegrationOnce(vecMatSeq.first),
                                                       TrapIntegrationOnce(vecMatSeq.second)};
}

std::optional<std::pair<std::pair<Eigen::Vector3d, Eigen::Matrix3d>,
                        std::pair<Eigen::Vector3d, Eigen::Matrix3d>>>
Estimator::InertialPosIntegration(const std::vector<IMUFrame::Ptr> &data,
                                  const std::string &imuTopic,
                                  double sTimeByBi,
                                  double eTimeByBi) const {
    auto vecMatSeq = InertialIntegrationBase(data, imuTopic, sTimeByBi, eTimeByBi);
    if (vecMatSeq.first.size() < 2 || vecMatSeq.second.size() < 2) {
        // invalid integration data
        return {};
    }
    return std::pair<std::pair<Eigen::Vector3d, Eigen::Matrix3d>,
                     std::pair<Eigen::Vector3d, Eigen::Matrix3d>>{
        {TrapIntegrationOnce(vecMatSeq.first), TrapIntegrationOnce(vecMatSeq.second)},
        {TrapIntegrationTwice(vecMatSeq.first), TrapIntegrationTwice(vecMatSeq.second)}};
}

std::pair<std::vector<std::pair<double, Eigen::Vector3d>>,
          std::vector<std::pair<double, Eigen::Matrix3d>>>
Estimator::InertialIntegrationBase(const std::vector<IMUFrame::Ptr> &data,
                                   const std::string &imuTopic,
                                   double sTimeByBi,
                                   double eTimeByBi) const {
    // vector and matrix sequence for integration
    std::vector<std::pair<double, Eigen::Vector3d>> vecSeq;
    std::vector<std::pair<double, Eigen::Matrix3d>> matSeq;

    double timeOffset = parMagr->TEMPORAL.TO_BiToBr.at(imuTopic);
    auto SO3_BiToBr = parMagr->EXTRI.SO3_BiToBr.at(imuTopic);

    // extract data by considering the initialized time offsets
    auto [sIter, eIter] = ExtractIMUDataPiece(data, sTimeByBi, eTimeByBi);

    const auto &so3Spline = splines->GetSo3Spline(Configor::Preference::SO3_SPLINE);

    for (auto iter = sIter; iter != eIter; ++iter) {
        const auto &frame = *iter;
        double curTimeByBr = frame->GetTimestamp() + timeOffset;

        if (!splines->TimeInRangeForSo3(curTimeByBr, Configor::Preference::SO3_SPLINE)) {
            continue;
        }

        auto SO3_BrToBr0 = so3Spline.Evaluate(curTimeByBr);

        // angular velocity in world
        auto SO3_VEL_BrToBr0InBr0 = SO3_BrToBr0 * so3Spline.VelocityBody(curTimeByBr);
        Eigen::Matrix3d SO3_VEL_MAT = Sophus::SO3d::hat(SO3_VEL_BrToBr0InBr0);

        // angular acceleration in world
        auto SO3_ACCE_BrToBr0InBr0 = SO3_BrToBr0 * so3Spline.AccelerationBody(curTimeByBr);
        Eigen::Matrix3d SO3_ACCE_MAT = Sophus::SO3d::hat(SO3_ACCE_BrToBr0InBr0);

        // store
        vecSeq.emplace_back(curTimeByBr, SO3_BrToBr0 * SO3_BiToBr * frame->GetAcce());
        matSeq.emplace_back(curTimeByBr,
                            (SO3_ACCE_MAT + SO3_VEL_MAT * SO3_VEL_MAT) * SO3_BrToBr0.matrix());
    }

    return {vecSeq, matSeq};
}

/**
 * param blocks:
 * [ SO3 | ... | SO3 | GYRO_BIAS | GYRO_MAP_COEFF | SO3_AtoG | SO3_BiToBr | TO_BiToBr ]
 */
void Estimator::AddIMUGyroMeasurement(const IMUFrame::Ptr &imuFrame,
                                      const std::string &topic,
                                      Opt option,
                                      double gyroWeight) {
    // prepare metas for splines
    SplineMetaType so3Meta;

    // different relative control points finding [single vs. range]
    // for the inertial measurements from the reference IMU, there is no need to consider a time
    // padding, as its time offsets would be fixed as identity
    if (IsOptionWith(Opt::OPT_TO_BiToBr, option) && Configor::DataStream::RefIMUTopic != topic) {
        double minTime = imuFrame->GetTimestamp() - Configor::Prior::TimeOffsetPadding;
        double maxTime = imuFrame->GetTimestamp() + Configor::Prior::TimeOffsetPadding;
        // invalid time stamp
        if (!splines->TimeInRangeForSo3(minTime, Configor::Preference::SO3_SPLINE) ||
            !splines->TimeInRangeForSo3(maxTime, Configor::Preference::SO3_SPLINE)) {
            return;
        }
        splines->CalculateSo3SplineMeta(Configor::Preference::SO3_SPLINE, {{minTime, maxTime}},
                                        so3Meta);
    } else {
        double curTime = imuFrame->GetTimestamp() + parMagr->TEMPORAL.TO_BiToBr.at(topic);

        // check point time stamp
        if (!splines->TimeInRangeForSo3(curTime, Configor::Preference::SO3_SPLINE)) {
            return;
        }
        splines->CalculateSo3SplineMeta(Configor::Preference::SO3_SPLINE, {{curTime, curTime}},
                                        so3Meta);
    }

    // create a cost function
    auto costFunc =
        IMUGyroFactor<Configor::Prior::SplineOrder>::Create(so3Meta, imuFrame, gyroWeight);

    // so3 knots param block [each has four sub params]
    for (int i = 0; i < static_cast<int>(so3Meta.NumParameters()); ++i) {
        costFunc->AddParameterBlock(4);
    }

    // GYRO gyroBias
    costFunc->AddParameterBlock(3);
    // GYRO map coeff
    costFunc->AddParameterBlock(6);
    // SO3_AtoG
    costFunc->AddParameterBlock(4);
    // SO3_BiToBr
    costFunc->AddParameterBlock(4);
    // TIME_OFFSET_BiToBc
    costFunc->AddParameterBlock(1);

    // set Residuals
    costFunc->SetNumResiduals(3);

    // organize the param block vector
    std::vector<double *> paramBlockVec;

    // so3 knots param block
    AddSo3KnotsData(paramBlockVec, splines->GetSo3Spline(Configor::Preference::SO3_SPLINE), so3Meta,
                    !IsOptionWith(Opt::OPT_SO3_SPLINE, option));

    // GYRO gyroBias
    auto gyroBias = parMagr->INTRI.IMU.at(topic)->GYRO.BIAS.data();
    paramBlockVec.push_back(gyroBias);
    // GYRO map coeff
    auto gyroMapCoeff = parMagr->INTRI.IMU.at(topic)->GYRO.MAP_COEFF.data();
    paramBlockVec.push_back(gyroMapCoeff);
    // SO3_AtoG
    auto SO3_AtoG = parMagr->INTRI.IMU.at(topic)->SO3_AtoG.data();
    paramBlockVec.push_back(SO3_AtoG);
    // SO3_BiToBr
    auto SO3_BiToBr = parMagr->EXTRI.SO3_BiToBr.at(topic).data();
    paramBlockVec.push_back(SO3_BiToBr);
    // TIME_OFFSET_BiToBc
    auto TIME_OFFSET_BiToBc = &parMagr->TEMPORAL.TO_BiToBr.at(topic);
    paramBlockVec.push_back(TIME_OFFSET_BiToBc);

    // pass to problem
    this->AddResidualBlock(costFunc, nullptr, paramBlockVec);

    this->SetManifold(SO3_AtoG, QUATER_MANIFOLD.get());
    this->SetManifold(SO3_BiToBr, QUATER_MANIFOLD.get());

    if (!IsOptionWith(Opt::OPT_GYRO_BIAS, option)) {
        this->SetParameterBlockConstant(gyroBias);
    }

    if (!IsOptionWith(Opt::OPT_GYRO_MAP_COEFF, option)) {
        this->SetParameterBlockConstant(gyroMapCoeff);
    }

    if (!IsOptionWith(Opt::OPT_SO3_AtoG, option)) {
        this->SetParameterBlockConstant(SO3_AtoG);
    }

    if (!IsOptionWith(Opt::OPT_SO3_BiToBr, option)) {
        this->SetParameterBlockConstant(SO3_BiToBr);
    }

    if (!IsOptionWith(Opt::OPT_TO_BiToBr, option)) {
        this->SetParameterBlockConstant(TIME_OFFSET_BiToBc);
    } else {
        // set bound
        this->SetParameterLowerBound(TIME_OFFSET_BiToBc, 0, -Configor::Prior::TimeOffsetPadding);
        this->SetParameterUpperBound(TIME_OFFSET_BiToBc, 0, Configor::Prior::TimeOffsetPadding);
    }
}

void Estimator::AddHandEyeRotAlignment(const std::string &camTopic,
                                       double tLastByCj,
                                       double tCurByCj,
                                       const Sophus::SO3d &so3LastCjToW,
                                       const Sophus::SO3d &so3CurCjToW,
                                       Opt option,
                                       double weight) {
    // prepare metas for splines
    SplineMetaType so3Meta;

    // different relative control points finding [single vs. range]
    if (IsOptionWith(Opt::OPT_TO_CjToBr, option)) {
        double lastMinTime = tLastByCj - Configor::Prior::TimeOffsetPadding;
        double lastMaxTime = tLastByCj + Configor::Prior::TimeOffsetPadding;
        // invalid time stamp
        if (!splines->TimeInRangeForSo3(lastMinTime, Configor::Preference::SO3_SPLINE) ||
            !splines->TimeInRangeForSo3(lastMaxTime, Configor::Preference::SO3_SPLINE)) {
            return;
        }

        double curMinTime = tCurByCj - Configor::Prior::TimeOffsetPadding;
        double curMaxTime = tCurByCj + Configor::Prior::TimeOffsetPadding;
        // invalid time stamp
        if (!splines->TimeInRangeForSo3(curMinTime, Configor::Preference::SO3_SPLINE) ||
            !splines->TimeInRangeForSo3(curMaxTime, Configor::Preference::SO3_SPLINE)) {
            return;
        }

        splines->CalculateSo3SplineMeta(Configor::Preference::SO3_SPLINE,
                                        {{lastMinTime, lastMaxTime}, {curMinTime, curMaxTime}},
                                        so3Meta);
    } else {
        double lastTime = tLastByCj + parMagr->TEMPORAL.TO_CjToBr.at(camTopic);
        double curTime = tCurByCj + parMagr->TEMPORAL.TO_CjToBr.at(camTopic);

        // check point time stamp
        if (!splines->TimeInRangeForSo3(lastTime, Configor::Preference::SO3_SPLINE) ||
            !splines->TimeInRangeForSo3(curTime, Configor::Preference::SO3_SPLINE)) {
            return;
        }
        splines->CalculateSo3SplineMeta(Configor::Preference::SO3_SPLINE,
                                        {{lastTime, lastTime}, {curTime, curTime}}, so3Meta);
    }

    // create a cost function
    auto costFunc = HandEyeRotationAlignFactor<Configor::Prior::SplineOrder>::Create(
        so3Meta, tLastByCj, tCurByCj, so3LastCjToW.inverse() * so3CurCjToW, weight);

    // so3 knots param block [each has four sub params]
    for (int i = 0; i < static_cast<int>(so3Meta.NumParameters()); ++i) {
        costFunc->AddParameterBlock(4);
    }

    // SO3_CmToBr
    costFunc->AddParameterBlock(4);
    // TO_CmToBr
    costFunc->AddParameterBlock(1);

    // set Residuals
    costFunc->SetNumResiduals(3);

    // organize the param block vector
    std::vector<double *> paramBlockVec;

    // so3 knots param block
    AddSo3KnotsData(paramBlockVec, splines->GetSo3Spline(Configor::Preference::SO3_SPLINE), so3Meta,
                    !IsOptionWith(Opt::OPT_SO3_SPLINE, option));

    auto SO3_CmToBr = parMagr->EXTRI.SO3_CjToBr.at(camTopic).data();
    paramBlockVec.push_back(SO3_CmToBr);

    auto TO_CmToBr = &parMagr->TEMPORAL.TO_CjToBr.at(camTopic);
    paramBlockVec.push_back(TO_CmToBr);

    // pass to problem
    this->AddResidualBlock(costFunc, nullptr, paramBlockVec);

    this->SetManifold(SO3_CmToBr, QUATER_MANIFOLD.get());

    if (!IsOptionWith(Opt::OPT_SO3_CjToBr, option)) {
        this->SetParameterBlockConstant(SO3_CmToBr);
    }

    if (!IsOptionWith(Opt::OPT_SO3_CjToBr, option)) {
        this->SetParameterBlockConstant(TO_CmToBr);
    } else {
        // set bound
        this->SetParameterLowerBound(TO_CmToBr, 0, -Configor::Prior::TimeOffsetPadding);
        this->SetParameterUpperBound(TO_CmToBr, 0, Configor::Prior::TimeOffsetPadding);
    }
}
}  // namespace ns_ekalibr
