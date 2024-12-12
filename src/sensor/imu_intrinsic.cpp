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

#include "sensor/imu_intrinsic.h"
#include "ctraj/core/imu.h"

namespace ns_ekalibr {
/**
 * BiasMapCoeff
 */
Eigen::Matrix3d BiasMapCoeff::MapMatrix() const {
    Eigen::Matrix3d mat = Eigen::Matrix3d::Zero();
    mat(0, 0) = MAP_COEFF(0), mat(1, 1) = MAP_COEFF(1), mat(2, 2) = MAP_COEFF(2);
    mat(0, 1) = MAP_COEFF(3);
    mat(0, 2) = MAP_COEFF(4);
    mat(1, 2) = MAP_COEFF(5);
    return mat;
}

BiasMapCoeff::BiasMapCoeff() { Clear(); }

void BiasMapCoeff::Clear() {
    BIAS = Eigen::Vector3d::Zero();
    MAP_COEFF = Eigen::Vector6d::Zero();
    MAP_COEFF(0) = 1.0;
    MAP_COEFF(1) = 1.0;
    MAP_COEFF(2) = 1.0;
}

/**
 * IMUIntrinsics
 */
IMUIntrinsics::IMUIntrinsics() { Clear(); }

IMUIntrinsics::Ptr IMUIntrinsics::Create() { return std::make_shared<IMUIntrinsics>(); }

void IMUIntrinsics::Clear() {
    ACCE.Clear();
    GYRO.Clear();
    SO3_AtoG = Sophus::SO3d();
}

IMUFrame::Ptr IMUIntrinsics::KinematicsToInertialMes(double time,
                                                     const Eigen::Vector3d& linAcceInW,
                                                     const Eigen::Vector3d& angVelInW,
                                                     const Sophus::SO3d& so3CurToW,
                                                     const Eigen::Vector3d& gravityInW) {
    Eigen::Vector3d force = so3CurToW.inverse() * (linAcceInW - gravityInW);
    Eigen::Vector3d angVel = so3CurToW.inverse() * angVelInW;
    return IMUFrame::Create(time, angVel, force);
}

std::tuple<double, Eigen::Vector3d, Eigen::Vector3d> IMUIntrinsics::InertialMesToKinematics(
    double time,
    const IMUFramePtr& frame,
    const Sophus::SO3d& so3CurToW,
    const Eigen::Vector3d& gravityInW) {
    Eigen::Vector3d acceInW = so3CurToW * frame->GetAcce() + gravityInW;
    Eigen::Vector3d angVelInW = so3CurToW * frame->GetGyro();
    return {time, acceInW, angVelInW};
}

Eigen::Vector3d IMUIntrinsics::InvolveForceIntri(const Eigen::Vector3d& force) const {
    return ACCE.MapMatrix() * force + ACCE.BIAS;
}

Eigen::Vector3d IMUIntrinsics::InvolveGyroIntri(const Eigen::Vector3d& gyro) const {
    return GYRO.MapMatrix() * (SO3_AtoG * gyro) + GYRO.BIAS;
}

IMUFramePtr IMUIntrinsics::InvolveIntri(const IMUFramePtr& frame) const {
    return IMUFrame::Create(frame->GetTimestamp(), InvolveGyroIntri(frame->GetGyro()),
                            InvolveForceIntri(frame->GetAcce()));
}

Eigen::Vector3d IMUIntrinsics::RemoveForceIntri(const Eigen::Vector3d& force) const {
    return ACCE.MapMatrix().inverse() * (force - ACCE.BIAS);
}

Eigen::Vector3d IMUIntrinsics::RemoveGyroIntri(const Eigen::Vector3d& gyro) const {
    return SO3_AtoG.inverse().matrix() * GYRO.MapMatrix().inverse() * (gyro - GYRO.BIAS);
}

IMUFramePtr IMUIntrinsics::RemoveIntri(const IMUFramePtr& frame) const {
    return IMUFrame::Create(frame->GetTimestamp(), RemoveGyroIntri(frame->GetGyro()),
                            RemoveForceIntri(frame->GetAcce()));
}

Eigen::Quaterniond IMUIntrinsics::Q_AtoG() const { return SO3_AtoG.unit_quaternion(); }

Eigen::Vector3d IMUIntrinsics::EULER_AtoG_RAD() const {
    return Q_AtoG().toRotationMatrix().eulerAngles(2, 1, 0);
}

Eigen::Vector3d IMUIntrinsics::EULER_AtoG_DEG() const {
    auto euler = EULER_AtoG_RAD();
    for (int i = 0; i != 3; ++i) {
        euler(i) *= IMUIntrinsics::RAD_TO_DEG;
    }
    return euler;
}

void IMUIntrinsics::Save(const std::string& filename, CerealArchiveType::Enum archiveType) const {
    std::ofstream file(filename, std::ios::out);
    auto ar = GetOutputArchiveVariant(file, archiveType);
    SerializeByOutputArchiveVariant(ar, archiveType, cereal::make_nvp("Intrinsics", *this));
}

IMUIntrinsics::Ptr IMUIntrinsics::Load(const std::string& filename,
                                       CerealArchiveType::Enum archiveType) {
    auto intri = Create();
    std::ifstream file(filename, std::ios::in);
    auto ar = GetInputArchiveVariant(file, archiveType);
    SerializeByInputArchiveVariant(ar, archiveType, cereal::make_nvp("Intrinsics", *intri));
    return intri;
}
}  // namespace ns_ekalibr