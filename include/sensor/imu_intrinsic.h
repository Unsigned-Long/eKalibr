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

#ifndef IMU_INTRINSIC_HPP
#define IMU_INTRINSIC_HPP

#include "sensor/imu.hpp"
#include "util/utils.h"
#include "util/cereal_archive_helper.hpp"

namespace ns_ekalibr {

struct BiasMapCoeff {
    Eigen::Vector3d BIAS;
    /**
     * MAP_COEFF: [v1, v2, v3, v4, v5, v6]^T
     * mapMatrix:
     *   v1 & v4 & v5
     *    0 & v2 & v6
     *    0 &  0 & v3
     * f(measure) = mapMat * f(real) + bias
     */
    Eigen::Vector6d MAP_COEFF;

    // organize the vector to a matrix
    [[nodiscard]] Eigen::Matrix3d MapMatrix() const;

    BiasMapCoeff();

    void Clear();

    // Serialization
    template <class Archive>
    void serialize(Archive &archive) {
        archive(CEREAL_NVP(BIAS), CEREAL_NVP(MAP_COEFF));
    }

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct IMUIntrinsics {
    using Ptr = std::shared_ptr<IMUIntrinsics>;

    // trans radian angle to degree angle
    constexpr static double RAD_TO_DEG = 180.0 / M_PI;
    // trans degree angle to radian angle
    constexpr static double DEG_TO_RAD = M_PI / 180.0;

    BiasMapCoeff ACCE, GYRO;
    Sophus::SO3d SO3_AtoG;

    IMUIntrinsics();

    static Ptr Create();

    void Clear();

    [[nodiscard]] static IMUFramePtr KinematicsToInertialMes(double time,
                                                             const Eigen::Vector3d &linAcceInW,
                                                             const Eigen::Vector3d &angVelInW,
                                                             const Sophus::SO3d &so3CurToW,
                                                             const Eigen::Vector3d &gravityInW);

    [[nodiscard]] static std::tuple<double, Eigen::Vector3d, Eigen::Vector3d>
    InertialMesToKinematics(double time,
                            const IMUFramePtr &frame,
                            const Sophus::SO3d &so3CurToW,
                            const Eigen::Vector3d &gravityInW);

    [[nodiscard]] Eigen::Vector3d InvolveForceIntri(const Eigen::Vector3d &force) const;

    [[nodiscard]] Eigen::Vector3d InvolveGyroIntri(const Eigen::Vector3d &gyro) const;

    [[nodiscard]] IMUFramePtr InvolveIntri(const IMUFramePtr &frame) const;

    [[nodiscard]] Eigen::Vector3d RemoveForceIntri(const Eigen::Vector3d &force) const;

    [[nodiscard]] Eigen::Vector3d RemoveGyroIntri(const Eigen::Vector3d &gyro) const;

    [[nodiscard]] IMUFramePtr RemoveIntri(const IMUFramePtr &frame) const;

    // quaternion
    [[nodiscard]] Eigen::Quaterniond Q_AtoG() const;

    // euler angles
    [[nodiscard]] Eigen::Vector3d EULER_AtoG_RAD() const;

    [[nodiscard]] Eigen::Vector3d EULER_AtoG_DEG() const;

    // save the parameters to file using cereal library
    void Save(const std::string &filename,
              CerealArchiveType::Enum archiveType = CerealArchiveType::Enum::YAML) const;

    // load the parameters from file using cereal library
    static Ptr Load(const std::string &filename,
                    CerealArchiveType::Enum archiveType = CerealArchiveType::Enum::YAML);

    // Serialization
    template <class Archive>
    void serialize(Archive &archive) {
        archive(CEREAL_NVP(ACCE), CEREAL_NVP(GYRO), CEREAL_NVP(SO3_AtoG));
    }
};

}  // namespace ns_ekalibr

#endif  // IMU_INTRINSIC_HPP
