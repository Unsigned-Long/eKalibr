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

#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#include "ceres/ceres.h"
#include "config/configor.h"
#include "ctraj/core/spline_bundle.h"
#include "sensor/imu.hpp"
#include <ctraj/core/pose.hpp>

namespace ns_ekalibr {
class CalibParamManager;
using CalibParamManagerPtr = std::shared_ptr<CalibParamManager>;
struct VisualProjectionPair;
using VisualProjectionPairPtr = std::shared_ptr<VisualProjectionPair>;
struct VisualProjectionCircleBasedPair;
using VisualProjectionCircleBasedPairPtr = std::shared_ptr<VisualProjectionCircleBasedPair>;

using namespace magic_enum::bitwise_operators;

enum class OptOption : std::uint64_t {
    /**
     * @brief options
     */
    NONE = static_cast<std::uint64_t>(1) << 0,
    OPT_SO3_SPLINE = static_cast<std::uint64_t>(1) << 1,
    OPT_SCALE_SPLINE = static_cast<std::uint64_t>(1) << 2,

    OPT_SO3_BiToBr = static_cast<std::uint64_t>(1) << 3,
    OPT_SO3_CjToBr = static_cast<std::uint64_t>(1) << 4,

    OPT_POS_BiInBr = static_cast<std::uint64_t>(1) << 5,
    OPT_POS_CjInBr = static_cast<std::uint64_t>(1) << 6,

    OPT_TO_BiToBr = static_cast<std::uint64_t>(1) << 7,
    OPT_TO_CjToBr = static_cast<std::uint64_t>(1) << 8,

    OPT_GYRO_BIAS = static_cast<std::uint64_t>(1) << 9,
    OPT_GYRO_MAP_COEFF = static_cast<std::uint64_t>(1) << 10,
    OPT_ACCE_BIAS = static_cast<std::uint64_t>(1) << 11,
    OPT_ACCE_MAP_COEFF = static_cast<std::uint64_t>(1) << 12,
    OPT_SO3_AtoG = static_cast<std::uint64_t>(1) << 13,

    OPT_GRAVITY = static_cast<std::uint64_t>(1) << 14,

    OPT_CAM_FOCAL_LEN = static_cast<std::uint64_t>(1) << 15,
    OPT_CAM_PRINCIPAL_POINT = static_cast<std::uint64_t>(1) << 16,
    OPT_CAM_DIST_COEFFS = static_cast<std::uint64_t>(1) << 17,

    ALL = OPT_SO3_SPLINE | OPT_SCALE_SPLINE | OPT_SO3_BiToBr | OPT_SO3_CjToBr | OPT_POS_BiInBr |
          OPT_POS_CjInBr | OPT_TO_BiToBr | OPT_TO_CjToBr | OPT_GYRO_BIAS | OPT_GYRO_MAP_COEFF |
          OPT_ACCE_BIAS | OPT_ACCE_MAP_COEFF | OPT_SO3_AtoG | OPT_GRAVITY | OPT_CAM_FOCAL_LEN |
          OPT_CAM_PRINCIPAL_POINT | OPT_CAM_DIST_COEFFS
};

class Estimator : public ceres::Problem {
public:
    using Ptr = std::shared_ptr<Estimator>;
    using SplineBundleType = ns_ctraj::SplineBundle<Configor::Prior::SplineOrder>;
    using So3SplineType = ns_ctraj::So3Spline<Configor::Prior::SplineOrder>;
    using PosSplineType = ns_ctraj::RdSpline<3, Configor::Prior::SplineOrder>;
    using SplineMetaType = ns_ctraj::SplineMeta<Configor::Prior::SplineOrder>;
    using Opt = OptOption;

private:
    CalibParamManagerPtr parMagr;

    // manifolds
    static std::shared_ptr<ceres::EigenQuaternionManifold> QUATER_MANIFOLD;
    static std::shared_ptr<ceres::SphereManifold<3>> GRAVITY_MANIFOLD;

public:
    Estimator(CalibParamManagerPtr calibParamManager);

    static Ptr Create(const CalibParamManagerPtr &calibParamManager);

    static ceres::Problem::Options DefaultProblemOptions();

    static ceres::Solver::Options DefaultSolverOptions(int threadNum = -1,
                                                       bool toStdout = true,
                                                       bool useCUDA = false);

    ceres::Solver::Summary Solve(
        const ceres::Solver::Options &options = Estimator::DefaultSolverOptions());

    Eigen::MatrixXd GetHessianMatrix(const std::vector<double *> &consideredParBlocks,
                                     int numThread = 1);

    void PrintParameterInfo() const;

    void SetIMUParamsConstant(const std::string &refIMUTopic);

    void SetEvCamParamsConstant(const std::string &refCamTopic);

protected:
    void AddSo3KnotsData(std::vector<double *> &paramBlockVec,
                         const So3SplineType &spline,
                         const SplineMetaType &splineMeta,
                         bool setToConst);

    void AddRdKnotsData(std::vector<double *> &paramBlockVec,
                        const PosSplineType &spline,
                        const SplineMetaType &splineMeta,
                        bool setToConst);

    static Eigen::MatrixXd CRSMatrix2EigenMatrix(const ceres::CRSMatrix *jacobian_crs_matrix);

    std::optional<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> InertialVelIntegration(
        const So3SplineType &so3Spline,
        const std::vector<IMUFrame::Ptr> &data,
        const std::string &imuTopic,
        double sTimeByBi,
        double eTimeByBi) const;

    std::optional<std::pair<std::pair<Eigen::Vector3d, Eigen::Matrix3d>,
                            std::pair<Eigen::Vector3d, Eigen::Matrix3d>>>
    InertialPosIntegration(const So3SplineType &so3Spline,
                           const std::vector<IMUFrame::Ptr> &data,
                           const std::string &imuTopic,
                           double sTimeByBi,
                           double eTimeByBi) const;

    std::pair<std::vector<std::pair<double, Eigen::Vector3d>>,
              std::vector<std::pair<double, Eigen::Matrix3d>>>
    InertialIntegrationBase(const So3SplineType &so3Spline,
                            const std::vector<IMUFrame::Ptr> &data,
                            const std::string &imuTopic,
                            double sTimeByBi,
                            double eTimeByBi) const;

    static auto ExtractIMUDataPiece(const std::vector<IMUFrame::Ptr> &data, double st, double et) {
        auto sIter = std::find_if(data.begin(), data.end(), [st](const IMUFrame::Ptr &frame) {
            return frame->GetTimestamp() > st;
        });
        auto eIter = std::find_if(data.rbegin(), data.rend(), [et](const IMUFrame::Ptr &frame) {
                         return frame->GetTimestamp() < et;
                     }).base();
        return std::pair(sIter, eIter);
    }

public:
    void AddIMUGyroMeasurement(const So3SplineType &so3Spline,
                               const IMUFrame::Ptr &imuFrame,
                               const std::string &topic,
                               Opt option,
                               double gyroWeight);

    void AddHandEyeRotAlignment(const So3SplineType &so3Spline,
                                const std::string &camTopic,
                                double tLastByCj,
                                double tCurByCj,
                                const Sophus::SO3d &so3LastCjToW,
                                const Sophus::SO3d &so3CurCjToW,
                                Opt option,
                                double weight);

    void AddSo3SplineAlignToWorldConstraint(const So3SplineType &so3Spline,
                                            Sophus::SO3d *SO3_Br0ToW,
                                            const std::string &camTopic,
                                            double timeByCj,
                                            const Sophus::SO3d &SO3_CurCjToW,
                                            Opt option,
                                            double weight);

    void AddEventInertialAlignment(const So3SplineType &so3Spline,
                                   const std::vector<IMUFrame::Ptr> &data,
                                   const std::string &camTopic,
                                   const std::string &imuTopic,
                                   const ns_ctraj::Posed &sPose,
                                   const ns_ctraj::Posed &ePose,
                                   Eigen::Vector3d *sVel,
                                   Eigen::Vector3d *eVel,
                                   Opt option,
                                   double weight);

    void AddIMUAcceMeasurement(const So3SplineType &so3Spline,
                               const PosSplineType &posSpline,
                               const IMUFrame::Ptr &imuFrame,
                               const std::string &topic,
                               Opt option,
                               double acceWeight);

    void AddPositionConstraint(const PosSplineType &posSpline,
                               double timeByBr,
                               const Eigen::Vector3d &pos,
                               Opt option,
                               double weight);

    void AddSo3Constraint(const So3SplineType &so3Spline,
                          double timeByBr,
                          const Sophus::SO3d &so3,
                          Opt option,
                          double weight);

    void AddVisualProjectionFactor(const So3SplineType &so3Spline,
                                   const PosSplineType &posSpline,
                                   const std::string &camTopic,
                                   const VisualProjectionPairPtr &pair,
                                   Opt option,
                                   double weight);

    void AddVisualDiscreteProjectionFactor(Sophus::SO3d *SO3_CjToW,
                                           Eigen::Vector3d *POS_CjInW,
                                           const std::string &camTopic,
                                           const VisualProjectionPairPtr &pair,
                                           Opt option,
                                           double weight);

    void AddPosLinearTailConstraint(PosSplineType &posSpline,
                                    Opt option,
                                    double weight,
                                    int count = Configor::Prior::SplineOrder);

    void AddSo3LinearTailConstraint(So3SplineType &so3Spline,
                                    Opt option,
                                    double weight,
                                    int count = Configor::Prior::SplineOrder);

    void AddPosLinearHeadConstraint(PosSplineType &posSpline,
                                    Opt option,
                                    double weight,
                                    int count = Configor::Prior::SplineOrder);

    void AddSo3LinearHeadConstraint(So3SplineType &so3Spline,
                                    Opt option,
                                    double weight,
                                    int count = Configor::Prior::SplineOrder);

    void AddVisualProjectionCircleBasedFactor(const So3SplineType &so3Spline,
                                              const PosSplineType &posSpline,
                                              const std::string &camTopic,
                                              const VisualProjectionCircleBasedPairPtr &pair,
                                              Opt option,
                                              double weight);
};
}  // namespace ns_ekalibr

#endif  // ESTIMATOR_H
