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

#include "calib/estimator.h"
#include "calib/calib_param_mgr.h"
#include "ctraj/core/trajectory_estimator.h"
#include "spdlog/spdlog.h"
#include "util/utils_tpl.hpp"
#include "factor/imu_gyro_factor.hpp"
#include <sensor/imu_intrinsic.h>
#include "factor/hand_eye_rot_align_factor.hpp"
#include "factor/so3_spline_world_align_factor.hpp"
#include "factor/event_inertial_align_factor.hpp"
#include "factor/imu_acce_factor.hpp"
#include "factor/lin_scale_factor.hpp"
#include "factor/so3_factor.hpp"
#include "factor/visual_projection_factor.hpp"
#include <veta/camera/pinhole.h>
#include "factor/inertial_align_factor.hpp"
#include "factor/prior_extri_pos_factor.hpp"
#include "factor/prior_extri_so3_factor.hpp"
#include "factor/prior_time_offset_factor.hpp"
#include "calib/spat_temp_priori.h"
#include <factor/hand_eye_transform_align_factor.hpp>
#include "factor/regularization_l2_factor.hpp"

namespace ns_ekalibr {
std::shared_ptr<ceres::EigenQuaternionManifold> Estimator::QUATER_MANIFOLD(
    new ceres::EigenQuaternionManifold());
std::shared_ptr<ceres::SphereManifold<3>> Estimator::GRAVITY_MANIFOLD(
    new ceres::SphereManifold<3>());

Estimator::Estimator(CalibParamManager::Ptr calibParamManager)
    : ceres::Problem(DefaultProblemOptions()),
      parMagr(std::move(calibParamManager)) {}

Estimator::Ptr Estimator::Create(const CalibParamManager::Ptr &calibParamManager) {
    return std::make_shared<Estimator>(calibParamManager);
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

ceres::Solver::Summary Estimator::Solve(const ceres::Solver::Options &options,
                                        const SpatialTemporalPrioriPtr &priori) {
    if (priori != nullptr) {
        priori->AddSpatTempPrioriConstraint(*this, *parMagr);
    }
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

void Estimator::SetIMUParamsConstant(const std::string &refIMUTopic) {
    auto SO3_BiToBr = parMagr->EXTRI.SO3_BiToBr.at(refIMUTopic).data();
    if (this->HasParameterBlock(SO3_BiToBr)) {
        this->SetParameterBlockConstant(SO3_BiToBr);
    }

    auto POS_BiInBr = parMagr->EXTRI.POS_BiInBr.at(refIMUTopic).data();
    if (this->HasParameterBlock(POS_BiInBr)) {
        this->SetParameterBlockConstant(POS_BiInBr);
    }

    auto TO_BiToBr = &parMagr->TEMPORAL.TO_BiToBr.at(refIMUTopic);
    if (this->HasParameterBlock(TO_BiToBr)) {
        this->SetParameterBlockConstant(TO_BiToBr);
    }
}

void Estimator::SetEvCamParamsConstant(const std::string &refCamTopic) {
    auto SO3_CjToBr = parMagr->EXTRI.SO3_CjToBr.at(refCamTopic).data();
    if (this->HasParameterBlock(SO3_CjToBr)) {
        this->SetParameterBlockConstant(SO3_CjToBr);
    }

    auto POS_CjInBr = parMagr->EXTRI.POS_CjInBr.at(refCamTopic).data();
    if (this->HasParameterBlock(POS_CjInBr)) {
        this->SetParameterBlockConstant(POS_CjInBr);
    }

    auto TO_CjToBr = &parMagr->TEMPORAL.TO_CjToBr.at(refCamTopic);
    if (this->HasParameterBlock(TO_CjToBr)) {
        this->SetParameterBlockConstant(TO_CjToBr);
    }
}

void Estimator::AddRdKnotsData(std::vector<double *> &paramBlockVec,
                               const Estimator::PosSplineType &spline,
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
                                const Estimator::So3SplineType &spline,
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
    const So3SplineType &so3Spline,
    const std::vector<IMUFrame::Ptr> &data,
    const std::string &imuTopic,
    double sTimeByBi,
    double eTimeByBi) const {
    auto vecMatSeq = InertialIntegrationBase(so3Spline, data, imuTopic, sTimeByBi, eTimeByBi);
    if (vecMatSeq.first.size() < 2 || vecMatSeq.second.size() < 2) {
        // invalid integration data
        return {};
    }
    return std::pair<Eigen::Vector3d, Eigen::Matrix3d>{TrapIntegrationOnce(vecMatSeq.first),
                                                       TrapIntegrationOnce(vecMatSeq.second)};
}

std::optional<std::pair<std::pair<Eigen::Vector3d, Eigen::Matrix3d>,
                        std::pair<Eigen::Vector3d, Eigen::Matrix3d>>>
Estimator::InertialPosIntegration(const So3SplineType &so3Spline,
                                  const std::vector<IMUFrame::Ptr> &data,
                                  const std::string &imuTopic,
                                  double sTimeByBi,
                                  double eTimeByBi) const {
    auto vecMatSeq = InertialIntegrationBase(so3Spline, data, imuTopic, sTimeByBi, eTimeByBi);
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
Estimator::InertialIntegrationBase(const So3SplineType &so3Spline,
                                   const std::vector<IMUFrame::Ptr> &data,
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

    for (auto iter = sIter; iter != eIter; ++iter) {
        const auto &frame = *iter;
        double curTimeByBr = frame->GetTimestamp() + timeOffset;

        if (!so3Spline.TimeStampInRange(curTimeByBr)) {
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
void Estimator::AddIMUGyroMeasurement(const So3SplineType &so3Spline,
                                      const IMUFrame::Ptr &imuFrame,
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
        if (!so3Spline.TimeStampInRange(minTime) || !so3Spline.TimeStampInRange(maxTime)) {
            return;
        }
        SplineBundleType::CalculateSplineMeta(so3Spline, {{minTime, maxTime}}, so3Meta);
    } else {
        double curTime = imuFrame->GetTimestamp() + parMagr->TEMPORAL.TO_BiToBr.at(topic);

        // check point time stamp
        if (!so3Spline.TimeStampInRange(curTime)) {
            return;
        }
        SplineBundleType::CalculateSplineMeta(so3Spline, {{curTime, curTime}}, so3Meta);
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
    AddSo3KnotsData(paramBlockVec, so3Spline, so3Meta, !IsOptionWith(Opt::OPT_SO3_SPLINE, option));

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

/**
 * param blocks:
 * [ SO3 | ... | SO3 | SO3_CjToBr | TO_CjToBr ]
 */
void Estimator::AddHandEyeRotAlignment(const So3SplineType &so3Spline,
                                       const std::string &camTopic,
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
        if (!so3Spline.TimeStampInRange(lastMinTime) || !so3Spline.TimeStampInRange(lastMaxTime)) {
            return;
        }

        double curMinTime = tCurByCj - Configor::Prior::TimeOffsetPadding;
        double curMaxTime = tCurByCj + Configor::Prior::TimeOffsetPadding;
        // invalid time stamp
        if (!so3Spline.TimeStampInRange(curMinTime) || !so3Spline.TimeStampInRange(curMaxTime)) {
            return;
        }

        SplineBundleType::CalculateSplineMeta(
            so3Spline, {{lastMinTime, lastMaxTime}, {curMinTime, curMaxTime}}, so3Meta);
    } else {
        double lastTime = tLastByCj + parMagr->TEMPORAL.TO_CjToBr.at(camTopic);
        double curTime = tCurByCj + parMagr->TEMPORAL.TO_CjToBr.at(camTopic);

        // check point time stamp
        if (!so3Spline.TimeStampInRange(lastTime) || !so3Spline.TimeStampInRange(curTime)) {
            return;
        }
        SplineBundleType::CalculateSplineMeta(so3Spline, {{lastTime, lastTime}, {curTime, curTime}},
                                              so3Meta);
    }

    // create a cost function
    auto costFunc = HandEyeRotationAlignFactor<Configor::Prior::SplineOrder>::Create(
        so3Meta, tLastByCj, tCurByCj, so3LastCjToW.inverse() * so3CurCjToW, weight);

    // so3 knots param block [each has four sub params]
    for (int i = 0; i < static_cast<int>(so3Meta.NumParameters()); ++i) {
        costFunc->AddParameterBlock(4);
    }

    // SO3_CjToBr
    costFunc->AddParameterBlock(4);
    // TO_CjToBr
    costFunc->AddParameterBlock(1);

    // set Residuals
    costFunc->SetNumResiduals(3);

    // organize the param block vector
    std::vector<double *> paramBlockVec;

    // so3 knots param block
    AddSo3KnotsData(paramBlockVec, so3Spline, so3Meta, !IsOptionWith(Opt::OPT_SO3_SPLINE, option));

    auto SO3_CjToBr = parMagr->EXTRI.SO3_CjToBr.at(camTopic).data();
    paramBlockVec.push_back(SO3_CjToBr);

    auto TO_CjToBr = &parMagr->TEMPORAL.TO_CjToBr.at(camTopic);
    paramBlockVec.push_back(TO_CjToBr);

    // pass to problem
    this->AddResidualBlock(costFunc, nullptr, paramBlockVec);

    this->SetManifold(SO3_CjToBr, QUATER_MANIFOLD.get());

    if (!IsOptionWith(Opt::OPT_SO3_CjToBr, option)) {
        this->SetParameterBlockConstant(SO3_CjToBr);
    }

    if (!IsOptionWith(Opt::OPT_TO_CjToBr, option)) {
        this->SetParameterBlockConstant(TO_CjToBr);
    } else {
        // set bound
        this->SetParameterLowerBound(TO_CjToBr, 0, -Configor::Prior::TimeOffsetPadding);
        this->SetParameterUpperBound(TO_CjToBr, 0, Configor::Prior::TimeOffsetPadding);
    }
}

void Estimator::AddHandEyeTransformAlignment(const So3SplineType &so3Spline,
                                             const PosSplineType &posSpline,
                                             const std::string &camTopic,
                                             double tLastByCj,
                                             double tCurByCj,
                                             const Sophus::SE3d &se3LastCjToW,
                                             const Sophus::SE3d &se3CurCjToW,
                                             Opt option,
                                             double weight) {
    // prepare metas for splines
    SplineMetaType so3Meta, posMeta;

    // different relative control points finding [single vs. range]
    if (IsOptionWith(Opt::OPT_TO_CjToBr, option)) {
        double lastMinTime = tLastByCj - Configor::Prior::TimeOffsetPadding;
        double lastMaxTime = tLastByCj + Configor::Prior::TimeOffsetPadding;
        // invalid time stamp
        if (!so3Spline.TimeStampInRange(lastMinTime) || !so3Spline.TimeStampInRange(lastMaxTime) ||
            !posSpline.TimeStampInRange(lastMinTime) || !posSpline.TimeStampInRange(lastMaxTime)) {
            return;
        }

        double curMinTime = tCurByCj - Configor::Prior::TimeOffsetPadding;
        double curMaxTime = tCurByCj + Configor::Prior::TimeOffsetPadding;
        // invalid time stamp
        if (!so3Spline.TimeStampInRange(curMinTime) || !so3Spline.TimeStampInRange(curMaxTime) ||
            !posSpline.TimeStampInRange(curMinTime) || !posSpline.TimeStampInRange(curMaxTime)) {
            return;
        }

        SplineBundleType::CalculateSplineMeta(
            so3Spline, {{lastMinTime, lastMaxTime}, {curMinTime, curMaxTime}}, so3Meta);
        SplineBundleType::CalculateSplineMeta(
            posSpline, {{lastMinTime, lastMaxTime}, {curMinTime, curMaxTime}}, posMeta);
    } else {
        double lastTime = tLastByCj + parMagr->TEMPORAL.TO_CjToBr.at(camTopic);
        double curTime = tCurByCj + parMagr->TEMPORAL.TO_CjToBr.at(camTopic);

        // check point time stamp
        if (!so3Spline.TimeStampInRange(lastTime) || !so3Spline.TimeStampInRange(curTime) ||
            !posSpline.TimeStampInRange(lastTime) || !posSpline.TimeStampInRange(curTime)) {
            return;
        }
        SplineBundleType::CalculateSplineMeta(so3Spline, {{lastTime, lastTime}, {curTime, curTime}},
                                              so3Meta);
        SplineBundleType::CalculateSplineMeta(posSpline, {{lastTime, lastTime}, {curTime, curTime}},
                                              posMeta);
    }

    // create a cost function
    auto costFunc = HandEyeTransformAlignFactor<Configor::Prior::SplineOrder>::Create(
        so3Meta, posMeta, tLastByCj, tCurByCj, se3LastCjToW.inverse() * se3CurCjToW, weight);

    // so3 knots param block [each has four sub params]
    for (int i = 0; i < static_cast<int>(so3Meta.NumParameters()); ++i) {
        costFunc->AddParameterBlock(4);
    }
    // pos knots param block [each has three sub params]
    for (int i = 0; i < static_cast<int>(posMeta.NumParameters()); ++i) {
        costFunc->AddParameterBlock(3);
    }

    // SO3_CjToBr
    costFunc->AddParameterBlock(4);
    // POS_CjInBr
    costFunc->AddParameterBlock(3);
    // TO_CjToBr
    costFunc->AddParameterBlock(1);

    // set Residuals
    costFunc->SetNumResiduals(6);

    // organize the param block vector
    std::vector<double *> paramBlockVec;

    // so3 knots param block
    AddSo3KnotsData(paramBlockVec, so3Spline, so3Meta, !IsOptionWith(Opt::OPT_SO3_SPLINE, option));
    AddRdKnotsData(paramBlockVec, posSpline, posMeta, !IsOptionWith(Opt::OPT_SCALE_SPLINE, option));

    auto SO3_CjToBr = parMagr->EXTRI.SO3_CjToBr.at(camTopic).data();
    paramBlockVec.push_back(SO3_CjToBr);

    auto POS_CjInBr = parMagr->EXTRI.POS_CjInBr.at(camTopic).data();
    paramBlockVec.push_back(POS_CjInBr);

    auto TO_CjToBr = &parMagr->TEMPORAL.TO_CjToBr.at(camTopic);
    paramBlockVec.push_back(TO_CjToBr);

    // pass to problem
    this->AddResidualBlock(costFunc, nullptr, paramBlockVec);

    this->SetManifold(SO3_CjToBr, QUATER_MANIFOLD.get());

    if (!IsOptionWith(Opt::OPT_SO3_CjToBr, option)) {
        this->SetParameterBlockConstant(SO3_CjToBr);
    }

    if (!IsOptionWith(Opt::OPT_POS_BiInBr, option)) {
        this->SetParameterBlockConstant(POS_CjInBr);
    }

    if (!IsOptionWith(Opt::OPT_TO_CjToBr, option)) {
        this->SetParameterBlockConstant(TO_CjToBr);
    } else {
        // set bound
        this->SetParameterLowerBound(TO_CjToBr, 0, -Configor::Prior::TimeOffsetPadding);
        this->SetParameterUpperBound(TO_CjToBr, 0, Configor::Prior::TimeOffsetPadding);
    }
}

/**
 * param blocks:
 * [ SO3 | ... | SO3 | SO3_CjToBr | TO_CjToBr | SO3_Br0ToW ]
 */
void Estimator::AddSo3SplineAlignToWorldConstraint(const So3SplineType &so3Spline,
                                                   Sophus::SO3d *SO3_Br0ToW,
                                                   const std::string &camTopic,
                                                   double timeByCj,
                                                   const Sophus::SO3d &SO3_CurCjToW,
                                                   Opt option,
                                                   double weight) {
    // prepare metas for splines
    SplineMetaType so3Meta;

    // different relative control points finding [single vs. range]
    if (IsOptionWith(Opt::OPT_TO_CjToBr, option)) {
        double curMinTime = timeByCj - Configor::Prior::TimeOffsetPadding;
        double curMaxTime = timeByCj + Configor::Prior::TimeOffsetPadding;
        // invalid time stamp
        if (!so3Spline.TimeStampInRange(curMinTime) || !so3Spline.TimeStampInRange(curMaxTime)) {
            return;
        }

        SplineBundleType::CalculateSplineMeta(so3Spline, {{curMinTime, curMaxTime}}, so3Meta);
    } else {
        double curTime = timeByCj + parMagr->TEMPORAL.TO_CjToBr.at(camTopic);

        // check point time stamp
        if (!so3Spline.TimeStampInRange(curTime)) {
            return;
        }
        SplineBundleType::CalculateSplineMeta(so3Spline, {{curTime, curTime}}, so3Meta);
    }

    // create a cost function
    auto costFunc = So3SplineAlignToWorldFactor<Configor::Prior::SplineOrder>::Create(
        so3Meta, timeByCj, SO3_CurCjToW, weight);

    // so3 knots param block [each has four sub params]
    for (int i = 0; i < static_cast<int>(so3Meta.NumParameters()); ++i) {
        costFunc->AddParameterBlock(4);
    }

    // SO3_CjToBr
    costFunc->AddParameterBlock(4);
    // TO_CjToBr
    costFunc->AddParameterBlock(1);
    // SO3_Br0ToW
    costFunc->AddParameterBlock(4);

    // set Residuals
    costFunc->SetNumResiduals(3);

    // organize the param block vector
    std::vector<double *> paramBlockVec;

    // so3 knots param block
    AddSo3KnotsData(paramBlockVec, so3Spline, so3Meta, !IsOptionWith(Opt::OPT_SO3_SPLINE, option));

    auto SO3_CmToBr = parMagr->EXTRI.SO3_CjToBr.at(camTopic).data();
    paramBlockVec.push_back(SO3_CmToBr);

    auto TO_CmToBr = &parMagr->TEMPORAL.TO_CjToBr.at(camTopic);
    paramBlockVec.push_back(TO_CmToBr);

    paramBlockVec.push_back(SO3_Br0ToW->data());

    // pass to problem
    this->AddResidualBlock(costFunc, nullptr, paramBlockVec);

    this->SetManifold(SO3_CmToBr, QUATER_MANIFOLD.get());
    this->SetManifold(SO3_Br0ToW->data(), QUATER_MANIFOLD.get());

    if (!IsOptionWith(Opt::OPT_SO3_CjToBr, option)) {
        this->SetParameterBlockConstant(SO3_CmToBr);
    }

    if (!IsOptionWith(Opt::OPT_TO_CjToBr, option)) {
        this->SetParameterBlockConstant(TO_CmToBr);
    } else {
        // set bound
        this->SetParameterLowerBound(TO_CmToBr, 0, -Configor::Prior::TimeOffsetPadding);
        this->SetParameterUpperBound(TO_CmToBr, 0, Configor::Prior::TimeOffsetPadding);
    }
}

/**
 * param blocks:
 * [ POS_CjInBr | POS_BiInBr | S_VEL | E_VEL | GRAVITY ]
 */
void Estimator::AddEventInertialAlignment(const So3SplineType &so3Spline,
                                          const std::vector<IMUFrame::Ptr> &data,
                                          const std::string &camTopic,
                                          const std::string &imuTopic,
                                          const ns_ctraj::Posed &sPose,
                                          const ns_ctraj::Posed &ePose,
                                          Eigen::Vector3d *sVel,
                                          Eigen::Vector3d *eVel,
                                          Opt option,
                                          double weight) {
    double st = sPose.timeStamp, et = ePose.timeStamp;
    double TO_CjToBr = parMagr->TEMPORAL.TO_CjToBr.at(camTopic);

    if (!so3Spline.TimeStampInRange(st + TO_CjToBr) ||
        !so3Spline.TimeStampInRange(et + TO_CjToBr)) {
        return;
    }

    double TO_CjToBi = TO_CjToBr - parMagr->TEMPORAL.TO_BiToBr.at(imuTopic);
    auto integrationData =
        InertialPosIntegration(so3Spline, data, imuTopic, st + TO_CjToBi, et + TO_CjToBi);
    if (integrationData == std::nullopt) {
        return;
    }
    auto [velVecMat, posVecMat] = *integrationData;

    // create a cost function
    auto helper = EventInertialAlignHelper<Configor::Prior::SplineOrder>(
        so3Spline, sPose, ePose, TO_CjToBr, velVecMat, posVecMat);
    auto costFunc = EventInertialAlignFactor<Configor::Prior::SplineOrder>::Create(helper, weight);

    costFunc->AddParameterBlock(3);
    costFunc->AddParameterBlock(3);

    costFunc->AddParameterBlock(3);
    costFunc->AddParameterBlock(3);

    costFunc->AddParameterBlock(3);

    costFunc->SetNumResiduals(6);

    // organize the param block vector
    std::vector<double *> paramBlockVec;

    auto POS_CjInBr = parMagr->EXTRI.POS_CjInBr.at(camTopic).data();
    paramBlockVec.push_back(POS_CjInBr);

    auto POS_BiInBr = parMagr->EXTRI.POS_BiInBr.at(imuTopic).data();
    paramBlockVec.push_back(POS_BiInBr);

    paramBlockVec.push_back(sVel->data());
    paramBlockVec.push_back(eVel->data());

    auto GRAVITY = parMagr->GRAVITY.data();
    paramBlockVec.push_back(GRAVITY);

    this->AddResidualBlock(costFunc, nullptr, paramBlockVec);
    this->SetManifold(GRAVITY, GRAVITY_MANIFOLD.get());

    if (!IsOptionWith(Opt::OPT_POS_CjInBr, option)) {
        this->SetParameterBlockConstant(POS_CjInBr);
    }
    if (!IsOptionWith(Opt::OPT_POS_BiInBr, option)) {
        this->SetParameterBlockConstant(POS_BiInBr);
    }
    if (!IsOptionWith(Opt::OPT_GRAVITY, option)) {
        this->SetParameterBlockConstant(GRAVITY);
    }
}

/**
 * param blocks:
 * [ POS_BiInBr | START_VEL | END_VEL | GRAVITY ]
 */
void Estimator::AddInertialAlignment(const So3SplineType &so3Spline,
                                     const std::vector<IMUFrame::Ptr> &data,
                                     const std::string &imuTopic,
                                     double sTimeByBr,
                                     double eTimeByBr,
                                     Eigen::Vector3d *sVel,
                                     Eigen::Vector3d *eVel,
                                     Opt option,
                                     double weight) {
    if (!so3Spline.TimeStampInRange(sTimeByBr) || !so3Spline.TimeStampInRange(eTimeByBr)) {
        return;
    }

    double TO_BrToBi = -parMagr->TEMPORAL.TO_BiToBr.at(imuTopic);
    auto velVecMat = InertialVelIntegration(so3Spline, data, imuTopic, sTimeByBr + TO_BrToBi,
                                            eTimeByBr + TO_BrToBi);

    if (velVecMat == std::nullopt) {
        return;
    }

    auto helper = InertialAlignHelper(eTimeByBr - sTimeByBr, *velVecMat);
    auto costFunc = InertialAlignFactor::Create(helper, weight);

    costFunc->AddParameterBlock(3);
    costFunc->AddParameterBlock(3);
    costFunc->AddParameterBlock(3);
    costFunc->AddParameterBlock(3);

    costFunc->SetNumResiduals(3);

    // organize the param block vector
    std::vector<double *> paramBlockVec;

    auto POS_BiInBr = parMagr->EXTRI.POS_BiInBr.at(imuTopic).data();
    paramBlockVec.push_back(POS_BiInBr);

    auto GRAVITY = parMagr->GRAVITY.data();
    paramBlockVec.push_back(GRAVITY);

    paramBlockVec.push_back(sVel->data());
    paramBlockVec.push_back(eVel->data());

    this->AddResidualBlock(costFunc, nullptr, paramBlockVec);
    this->SetManifold(GRAVITY, GRAVITY_MANIFOLD.get());

    if (!IsOptionWith(Opt::OPT_POS_BiInBr, option)) {
        this->SetParameterBlockConstant(POS_BiInBr);
    }
    if (!IsOptionWith(Opt::OPT_GRAVITY, option)) {
        this->SetParameterBlockConstant(GRAVITY);
    }
}

/**
 * param blocks:
 * [ SO3 | ... | SO3 | LIN_SCALE | ... | LIN_SCALE | ACCE_BIAS | ACCE_MAP_COEFF | GRAVITY |
 *   SO3_BiToBr | POS_BiInBr | TO_BiToBr ]
 */
void Estimator::AddIMUAcceMeasurement(const So3SplineType &so3Spline,
                                      const PosSplineType &posSpline,
                                      const IMUFrame::Ptr &imuFrame,
                                      const std::string &topic,
                                      Opt option,
                                      double acceWeight) {
    // prepare metas for splines
    SplineMetaType so3Meta, scaleMeta;

    // different relative control points finding [single vs. range]
    // for the inertial measurements from the reference IMU, there is no need to consider a time
    // padding, as its time offsets would be fixed as identity
    if (IsOptionWith(Opt::OPT_TO_BiToBr, option) && Configor::DataStream::RefIMUTopic != topic) {
        double minTime = imuFrame->GetTimestamp() - Configor::Prior::TimeOffsetPadding;
        double maxTime = imuFrame->GetTimestamp() + Configor::Prior::TimeOffsetPadding;
        // invalid time stamp
        if (!so3Spline.TimeStampInRange(minTime) || !so3Spline.TimeStampInRange(maxTime) ||
            !posSpline.TimeStampInRange(minTime) || !posSpline.TimeStampInRange(maxTime)) {
            return;
        }
        SplineBundleType::CalculateSplineMeta(so3Spline, {{minTime, maxTime}}, so3Meta);
        SplineBundleType::CalculateSplineMeta(posSpline, {{minTime, maxTime}}, scaleMeta);
    } else {
        double curTime = imuFrame->GetTimestamp() + parMagr->TEMPORAL.TO_BiToBr.at(topic);

        // check point time stamp
        if (!so3Spline.TimeStampInRange(curTime) || !posSpline.TimeStampInRange(curTime)) {
            return;
        }
        SplineBundleType::CalculateSplineMeta(so3Spline, {{curTime, curTime}}, so3Meta);
        SplineBundleType::CalculateSplineMeta(posSpline, {{curTime, curTime}}, scaleMeta);
    }
    // create a cost function
    auto costFunc = IMUAcceFactor<Configor::Prior::SplineOrder, 2>::Create(so3Meta, scaleMeta,
                                                                           imuFrame, acceWeight);

    // so3 knots param block [each has four sub params]
    for (int i = 0; i < static_cast<int>(so3Meta.NumParameters()); ++i) {
        costFunc->AddParameterBlock(4);
    }
    // pos knots param block [each has three sub params]
    for (int i = 0; i < static_cast<int>(scaleMeta.NumParameters()); ++i) {
        costFunc->AddParameterBlock(3);
    }

    // ACCE_BIAS
    costFunc->AddParameterBlock(3);
    // ACCE_MAP_COEFF
    costFunc->AddParameterBlock(6);
    // GRAVITY
    costFunc->AddParameterBlock(3);
    // SO3_BiToBr
    costFunc->AddParameterBlock(4);
    // POS_BiInBr
    costFunc->AddParameterBlock(3);
    // TO_BiToBr
    costFunc->AddParameterBlock(1);

    costFunc->SetNumResiduals(3);

    // organize the param block vector
    std::vector<double *> paramBlockVec;

    // so3 knots param block
    AddSo3KnotsData(paramBlockVec, so3Spline, so3Meta, !IsOptionWith(Opt::OPT_SO3_SPLINE, option));

    // lin acce knots
    AddRdKnotsData(paramBlockVec, posSpline, scaleMeta,
                   !IsOptionWith(Opt::OPT_SCALE_SPLINE, option));

    // ACCE_BIAS
    auto acceBias = parMagr->INTRI.IMU.at(topic)->ACCE.BIAS.data();
    paramBlockVec.push_back(acceBias);
    // ACCE_MAP_COEFF
    auto aceMapCoeff = parMagr->INTRI.IMU.at(topic)->ACCE.MAP_COEFF.data();
    paramBlockVec.push_back(aceMapCoeff);
    // GRAVITY
    auto gravity = parMagr->GRAVITY.data();
    paramBlockVec.push_back(gravity);
    // SO3_BiToBc
    auto SO3_BiToBc = parMagr->EXTRI.SO3_BiToBr.at(topic).data();
    paramBlockVec.push_back(SO3_BiToBc);
    // POS_BiInBc
    auto POS_BiInBc = parMagr->EXTRI.POS_BiInBr.at(topic).data();
    paramBlockVec.push_back(POS_BiInBc);
    // TIME_OFFSET_BiToBc
    auto TIME_OFFSET_BiToBc = &parMagr->TEMPORAL.TO_BiToBr.at(topic);
    paramBlockVec.push_back(TIME_OFFSET_BiToBc);

    // pass to problem
    this->AddResidualBlock(costFunc, nullptr, paramBlockVec);
    this->SetManifold(gravity, GRAVITY_MANIFOLD.get());
    this->SetManifold(SO3_BiToBc, QUATER_MANIFOLD.get());

    if (!IsOptionWith(Opt::OPT_ACCE_BIAS, option)) {
        this->SetParameterBlockConstant(acceBias);
    }

    if (!IsOptionWith(Opt::OPT_ACCE_MAP_COEFF, option)) {
        this->SetParameterBlockConstant(aceMapCoeff);
    }

    if (!IsOptionWith(Opt::OPT_GRAVITY, option)) {
        this->SetParameterBlockConstant(gravity);
    }

    if (!IsOptionWith(Opt::OPT_SO3_BiToBr, option)) {
        this->SetParameterBlockConstant(SO3_BiToBc);
    }

    if (!IsOptionWith(Opt::OPT_POS_BiInBr, option)) {
        this->SetParameterBlockConstant(POS_BiInBc);
    }

    if (!IsOptionWith(Opt::OPT_TO_BiToBr, option)) {
        this->SetParameterBlockConstant(TIME_OFFSET_BiToBc);
    } else {
        // set bound
        this->SetParameterLowerBound(TIME_OFFSET_BiToBc, 0, -Configor::Prior::TimeOffsetPadding);
        this->SetParameterUpperBound(TIME_OFFSET_BiToBc, 0, Configor::Prior::TimeOffsetPadding);
    }
}

void Estimator::AddPositionConstraint(const PosSplineType &posSpline,
                                      double timeByBr,
                                      const Eigen::Vector3d &pos,
                                      Opt option,
                                      double weight) {
    // check point time stamp
    if (!posSpline.TimeStampInRange(timeByBr)) {
        return;
    }

    SplineMetaType scaleMeta;
    SplineBundleType::CalculateSplineMeta(posSpline, {{timeByBr, timeByBr}}, scaleMeta);

    // create a cost function
    auto costFunc = LinearScaleDerivFactor<Configor::Prior::SplineOrder, 0>::Create(
        scaleMeta, timeByBr, pos, weight);

    // pos knots param block [each has three sub params]
    for (int i = 0; i < static_cast<int>(scaleMeta.NumParameters()); ++i) {
        costFunc->AddParameterBlock(3);
    }

    // the Residual
    costFunc->SetNumResiduals(3);

    // organize the param block vector
    std::vector<double *> paramBlockVec;

    // lin acce knots
    AddRdKnotsData(paramBlockVec, posSpline, scaleMeta,
                   !IsOptionWith(Opt::OPT_SCALE_SPLINE, option));

    // pass to problem
    this->AddResidualBlock(costFunc, nullptr, paramBlockVec);
}

void Estimator::AddSo3Constraint(const So3SplineType &so3Spline,
                                 double timeByBr,
                                 const Sophus::SO3d &so3,
                                 Opt option,
                                 double weight) {
    // check point time stamp
    if (!so3Spline.TimeStampInRange(timeByBr)) {
        return;
    }

    SplineMetaType so3Meta;
    SplineBundleType::CalculateSplineMeta(so3Spline, {{timeByBr, timeByBr}}, so3Meta);

    // create a cost function
    auto costFunc = So3Factor<Configor::Prior::SplineOrder>::Create(so3Meta, timeByBr, so3, weight);

    // pos knots param block [each has three sub params]
    for (int i = 0; i < static_cast<int>(so3Meta.NumParameters()); ++i) {
        costFunc->AddParameterBlock(4);
    }

    // the Residual
    costFunc->SetNumResiduals(3);

    // organize the param block vector
    std::vector<double *> paramBlockVec;

    // so3 knots param block
    AddSo3KnotsData(paramBlockVec, so3Spline, so3Meta, !IsOptionWith(Opt::OPT_SO3_SPLINE, option));

    // pass to problem
    this->AddResidualBlock(costFunc, nullptr, paramBlockVec);
}

/**
 * param blocks:
 * [ SO3 | ... | SO3 | LIN_SCALE | ... | LIN_SCALE | SO3_CjToBr | POS_CjInBr | TO_CjToBr |
 *   FX | FY | CX | CY | DIST_COEFFS ]
 */
void Estimator::AddVisualProjectionFactor(const So3SplineType &so3Spline,
                                          const PosSplineType &posSpline,
                                          const std::string &camTopic,
                                          const VisualProjectionPairPtr &pair,
                                          Opt option,
                                          double weight) {
    // prepare metas for splines
    SplineMetaType so3Meta, scaleMeta;

    if (IsOptionWith(Opt::OPT_TO_CjToBr, option)) {
        double minTime = pair->timestamp - Configor::Prior::TimeOffsetPadding;
        double maxTime = pair->timestamp + Configor::Prior::TimeOffsetPadding;
        // invalid time stamp
        if (!so3Spline.TimeStampInRange(minTime) || !so3Spline.TimeStampInRange(maxTime) ||
            !posSpline.TimeStampInRange(minTime) || !posSpline.TimeStampInRange(maxTime)) {
            return;
        }
        SplineBundleType::CalculateSplineMeta(so3Spline, {{minTime, maxTime}}, so3Meta);
        SplineBundleType::CalculateSplineMeta(posSpline, {{minTime, maxTime}}, scaleMeta);
    } else {
        double curTime = pair->timestamp + parMagr->TEMPORAL.TO_CjToBr.at(camTopic);

        // check point time stamp
        if (!so3Spline.TimeStampInRange(curTime) || !posSpline.TimeStampInRange(curTime)) {
            return;
        }
        SplineBundleType::CalculateSplineMeta(so3Spline, {{curTime, curTime}}, so3Meta);
        SplineBundleType::CalculateSplineMeta(posSpline, {{curTime, curTime}}, scaleMeta);
    }
    // create a cost function
    auto costFunc = VisualProjectionFactor<Configor::Prior::SplineOrder>::Create(so3Meta, scaleMeta,
                                                                                 pair, weight);

    // so3 knots param block [each has four sub params]
    for (int i = 0; i < static_cast<int>(so3Meta.NumParameters()); ++i) {
        costFunc->AddParameterBlock(4);
    }
    // pos knots param block [each has three sub params]
    for (int i = 0; i < static_cast<int>(scaleMeta.NumParameters()); ++i) {
        costFunc->AddParameterBlock(3);
    }

    // SO3_CjToBr
    costFunc->AddParameterBlock(4);
    // POS_CjInBr
    costFunc->AddParameterBlock(3);
    // TO_CjToBr
    costFunc->AddParameterBlock(1);

    // fx, fy, cx, cy
    costFunc->AddParameterBlock(1);
    costFunc->AddParameterBlock(1);
    costFunc->AddParameterBlock(1);
    costFunc->AddParameterBlock(1);

    // k1, k2, k3, p1, p2
    costFunc->AddParameterBlock(5);

    costFunc->SetNumResiduals(2);

    // organize the param block vector
    std::vector<double *> paramBlockVec;

    // so3 knots param block
    AddSo3KnotsData(paramBlockVec, so3Spline, so3Meta, !IsOptionWith(Opt::OPT_SO3_SPLINE, option));

    // lin acce knots
    AddRdKnotsData(paramBlockVec, posSpline, scaleMeta,
                   !IsOptionWith(Opt::OPT_SCALE_SPLINE, option));

    // SO3_CjToBr
    auto SO3_CjToBr = parMagr->EXTRI.SO3_CjToBr.at(camTopic).data();
    paramBlockVec.push_back(SO3_CjToBr);
    // POS_CjInBr
    auto POS_CjInBr = parMagr->EXTRI.POS_CjInBr.at(camTopic).data();
    paramBlockVec.push_back(POS_CjInBr);
    // TIME_OFFSET_CjToBr
    auto TIME_OFFSET_CjToBr = &parMagr->TEMPORAL.TO_CjToBr.at(camTopic);
    paramBlockVec.push_back(TIME_OFFSET_CjToBr);

    auto &intri = parMagr->INTRI.Camera.at(camTopic);
    paramBlockVec.push_back(intri->FXAddress());
    paramBlockVec.push_back(intri->FYAddress());
    paramBlockVec.push_back(intri->CXAddress());
    paramBlockVec.push_back(intri->CYAddress());
    paramBlockVec.push_back(intri->DistCoeffAddress());

    // pass to problem
    this->AddResidualBlock(costFunc, new ceres::HuberLoss(3.0 /* 3 * sigma as the outliers */),
                           paramBlockVec);
    this->SetManifold(SO3_CjToBr, QUATER_MANIFOLD.get());

    if (!IsOptionWith(Opt::OPT_SO3_CjToBr, option)) {
        this->SetParameterBlockConstant(SO3_CjToBr);
    }

    if (!IsOptionWith(Opt::OPT_POS_CjInBr, option)) {
        this->SetParameterBlockConstant(POS_CjInBr);
    }

    if (!IsOptionWith(Opt::OPT_TO_CjToBr, option)) {
        this->SetParameterBlockConstant(TIME_OFFSET_CjToBr);
    } else {
        // set bound
        this->SetParameterLowerBound(TIME_OFFSET_CjToBr, 0, -Configor::Prior::TimeOffsetPadding);
        this->SetParameterUpperBound(TIME_OFFSET_CjToBr, 0, Configor::Prior::TimeOffsetPadding);
    }

    if (!IsOptionWith(Opt::OPT_CAM_FOCAL_LEN, option)) {
        this->SetParameterBlockConstant(intri->FXAddress());
        this->SetParameterBlockConstant(intri->FYAddress());
    }

    if (!IsOptionWith(Opt::OPT_CAM_PRINCIPAL_POINT, option)) {
        this->SetParameterBlockConstant(intri->CXAddress());
        this->SetParameterBlockConstant(intri->CYAddress());
    }

    if (!IsOptionWith(Opt::OPT_CAM_DIST_COEFFS, option)) {
        this->SetParameterBlockConstant(intri->DistCoeffAddress());
    }
}

/**
 * param blocks:
 * [ SO3_CjToW | POS_CjInW | FX | FY | CX | CY | DIST_COEFFS ]
 */
void Estimator::AddVisualDiscreteProjectionFactor(Sophus::SO3d *SO3_CjToW,
                                                  Eigen::Vector3d *POS_CjInW,
                                                  const std::string &camTopic,
                                                  const VisualProjectionPairPtr &pair,
                                                  Opt option,
                                                  double weight) {
    // create a cost function
    auto costFunc = VisualDiscreteProjectionFactor::Create(pair, weight);

    // SO3_CjToW
    costFunc->AddParameterBlock(4);
    // POS_CjInW
    costFunc->AddParameterBlock(3);

    // fx, fy, cx, cy
    costFunc->AddParameterBlock(1);
    costFunc->AddParameterBlock(1);
    costFunc->AddParameterBlock(1);
    costFunc->AddParameterBlock(1);

    // k1, k2, k3, p1, p2
    costFunc->AddParameterBlock(5);

    costFunc->SetNumResiduals(2);

    // organize the param block vector
    std::vector<double *> paramBlockVec;
    paramBlockVec.push_back(SO3_CjToW->data());
    paramBlockVec.push_back(POS_CjInW->data());

    auto &intri = parMagr->INTRI.Camera.at(camTopic);
    paramBlockVec.push_back(intri->FXAddress());
    paramBlockVec.push_back(intri->FYAddress());
    paramBlockVec.push_back(intri->CXAddress());
    paramBlockVec.push_back(intri->CYAddress());
    paramBlockVec.push_back(intri->DistCoeffAddress());

    // pass to problem
    this->AddResidualBlock(costFunc, new ceres::HuberLoss(3.0 /* 3 * sigma as the outliers */),
                           paramBlockVec);
    this->SetManifold(SO3_CjToW->data(), QUATER_MANIFOLD.get());

    if (!IsOptionWith(Opt::OPT_CAM_FOCAL_LEN, option)) {
        this->SetParameterBlockConstant(intri->FXAddress());
        this->SetParameterBlockConstant(intri->FYAddress());
    }

    if (!IsOptionWith(Opt::OPT_CAM_PRINCIPAL_POINT, option)) {
        this->SetParameterBlockConstant(intri->CXAddress());
        this->SetParameterBlockConstant(intri->CYAddress());
    }

    if (!IsOptionWith(Opt::OPT_CAM_DIST_COEFFS, option)) {
        this->SetParameterBlockConstant(intri->DistCoeffAddress());
    }
}

/**
 * param blocks:
 * [ SO3_Sen1ToRef | SO3_Sen2ToRef ]
 */
void Estimator::AddPriorExtriSO3Constraint(const Sophus::SO3d &SO3_Sen1ToSen2,
                                           Sophus::SO3d *SO3_Sen1ToRef,
                                           Sophus::SO3d *SO3_Sen2ToRef,
                                           double weight) {
    // create a cost function
    auto costFunc = PriorExtriSO3Factor::Create(SO3_Sen1ToSen2, weight);

    // SO3_Sen1ToRef
    costFunc->AddParameterBlock(4);
    // SO3_Sen2ToRef
    costFunc->AddParameterBlock(4);

    // set Residuals
    costFunc->SetNumResiduals(3);

    // organize the param block vector
    std::vector<double *> paramBlockVec;

    // SO3_Sen1ToRef
    paramBlockVec.push_back(SO3_Sen1ToRef->data());
    // SO3_Sen2ToRef
    paramBlockVec.push_back(SO3_Sen2ToRef->data());

    // pass to problem
    this->AddResidualBlock(costFunc, nullptr, paramBlockVec);

    this->SetManifold(SO3_Sen1ToRef->data(), QUATER_MANIFOLD.get());
    this->SetManifold(SO3_Sen2ToRef->data(), QUATER_MANIFOLD.get());
}
/**
 * param blocks:
 * [ POS_Sen1InRef | SO3_Sen2ToRef | POS_Sen2InRef ]
 */
void Estimator::AddPriorExtriPOSConstraint(const Eigen::Vector3d &POS_Sen1InSen2,
                                           Eigen::Vector3d *POS_Sen1InRef,
                                           Sophus::SO3d *SO3_Sen2ToRef,
                                           Eigen::Vector3d *POS_Sen2InRef,
                                           double weight) {
    // create a cost function
    auto costFunc = PriorExtriPOSFactor::Create(POS_Sen1InSen2, weight);

    // POS_Sen1InRef
    costFunc->AddParameterBlock(3);
    // SO3_Sen2ToRef
    costFunc->AddParameterBlock(4);
    // POS_Sen2InRef
    costFunc->AddParameterBlock(3);

    // set Residuals
    costFunc->SetNumResiduals(3);

    // organize the param block vector
    std::vector<double *> paramBlockVec;

    // POS_Sen1InRef
    paramBlockVec.push_back(POS_Sen1InRef->data());
    // SO3_Sen2ToRef
    paramBlockVec.push_back(SO3_Sen2ToRef->data());
    // POS_Sen2InRef
    paramBlockVec.push_back(POS_Sen2InRef->data());

    // pass to problem
    this->AddResidualBlock(costFunc, nullptr, paramBlockVec);

    this->SetManifold(SO3_Sen2ToRef->data(), QUATER_MANIFOLD.get());
}
/**
 * param blocks:
 * [ TO_Sen1ToRef | TO_Sen2ToRef ]
 */
void Estimator::AddPriorTimeOffsetConstraint(const double &TO_Sen1ToSen2,
                                             double *TO_Sen1ToRef,
                                             double *TO_Sen2ToRef,
                                             double weight) {
    // create a cost function
    auto costFunc = PriorTimeOffsetFactor::Create(TO_Sen1ToSen2, weight);

    // TO_Sen1ToRef
    costFunc->AddParameterBlock(1);
    // TO_Sen2ToRef
    costFunc->AddParameterBlock(1);

    // set Residuals
    costFunc->SetNumResiduals(1);

    // organize the param block vector
    std::vector<double *> paramBlockVec;

    // TO_Sen1ToRef
    paramBlockVec.push_back(TO_Sen1ToRef);
    // TO_Sen2ToRef
    paramBlockVec.push_back(TO_Sen2ToRef);

    // pass to problem
    this->AddResidualBlock(costFunc, nullptr, paramBlockVec);
}

void Estimator::AddRegularizationL2Constraint(PosSplineType &posSpline, Opt option, double weight) {
    if (!IsOptionWith(Opt::OPT_SCALE_SPLINE, option)) {
        return;
    }
    for (int i = 0; i < static_cast<int>(posSpline.GetKnots().size()); ++i) {
        if (!this->HasParameterBlock(posSpline.GetKnot(i).data())) {
            continue;
        }
        RegularizationL2Factor<3>::AddToSolver(this, &posSpline.GetKnot(i), weight);
    }
}

void Estimator::AddRegularizationL2Constraint(So3SplineType &so3Spline, Opt option, double weight) {
    if (!IsOptionWith(Opt::OPT_SO3_SPLINE, option)) {
        return;
    }
    for (int i = 0; i < static_cast<int>(so3Spline.GetKnots().size()); ++i) {
        if (!this->HasParameterBlock(so3Spline.GetKnot(i).data())) {
            continue;
        }
        So3RegularizationL2Factor::AddToSolver(this, &so3Spline.GetKnot(i), QUATER_MANIFOLD.get(),
                                               weight);
    }
}
}  // namespace ns_ekalibr
