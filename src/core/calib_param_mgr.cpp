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

#include "core/calib_param_mgr.h"
#include "sensor/imu_intrinsic.h"
#include "config/configor.h"
#include "veta/camera/pinhole_brown.h"
#include "util/status.hpp"
#include "tiny-viewer/core/viewer.h"
#include "tiny-viewer/object/imu.h"
#include "tiny-viewer/object/camera.h"
#include "tiny-viewer/entity/line.h"
#include "tiny-viewer/core/pose.hpp"
#include "util/utils_tpl.hpp"
#include <viewer/viewer.h>

namespace ns_ekalibr {
// ------------------------
// static initialized filed
// ------------------------
const std::string CalibParamManager::Header::Software = "eKalibr";
const std::string CalibParamManager::Header::Version = "1.0.0";
const std::string CalibParamManager::Header::Address =
    "https://github.com/Unsigned-Long/eKalibr.git";

CalibParamManager::CalibParamManager(const std::vector<std::string> &imuTopics,
                                     const std::vector<std::string> &cameraTopics)
    : EXTRI(),
      TEMPORAL(),
      INTRI(),
      GRAVITY() {
    for (const auto &topic : imuTopics) {
        EXTRI.SO3_BiToBr[topic] = Sophus::SO3d();
        EXTRI.POS_BiInBr[topic] = Eigen::Vector3d::Zero();
        TEMPORAL.TO_BiToBr[topic] = 0.0;
        INTRI.IMU[topic] = nullptr;
    }
    for (const auto &topic : cameraTopics) {
        EXTRI.SO3_CjToBr[topic] = Sophus::SO3d();
        EXTRI.POS_CjInBr[topic] = Eigen::Vector3d::Zero();
        TEMPORAL.TO_CjToBr[topic] = 0.0;
        INTRI.Camera[topic] = nullptr;
    }
    GRAVITY = Eigen::Vector3d(0.0, 0.0, -9.8);
}

CalibParamManager::Ptr CalibParamManager::Create(const std::vector<std::string> &imuTopics,
                                                 const std::vector<std::string> &cameraTopics) {
    return std::make_shared<CalibParamManager>(imuTopics, cameraTopics);
}

CalibParamManager::Ptr CalibParamManager::InitParamsFromConfigor() {
    spdlog::info("initialize calibration parameter manager using configor...");

    auto parMarg = CalibParamManager::Create(ExtractKeysAsVec(Configor::DataStream::IMUTopics),
                                             ExtractKeysAsVec(Configor::DataStream::EventTopics));

    // intrinsics
    for (const auto &[topic, config] : Configor::DataStream::IMUTopics) {
        parMarg->INTRI.IMU.at(topic) = IMUIntrinsics::Create();
    }
    for (const auto &[topic, config] : Configor::DataStream::EventTopics) {
        parMarg->INTRI.Camera.at(topic) = ns_veta::PinholeIntrinsicBrownT2::Create(
            config.Width, config.Height, 0.0, 0.0, config.Width * 0.5, config.Height * 0.5);
    }

    // align to the negative 'z' axis
    parMarg->GRAVITY = Eigen::Vector3d(0.0, 0.0, -Configor::Prior::GravityNorm);

    spdlog::info("initialize calibration parameter manager using configor finished.");
    return parMarg;
}

void CalibParamManager::ShowParamStatus() {
    std::stringstream stream;
#define ITEM(name) fmt::format(fmt::emphasis::bold | fmt::fg(fmt::color::green), name)
#define PARAM(name) fmt::format(fmt::emphasis::bold | fmt::fg(fmt::color::steel_blue), name)
#define STREAM_PACK(obj) stream << "-- " << obj << std::endl;

    constexpr std::size_t n = 71;

    STREAM_PACK(std::string(25, '-'))
    STREAM_PACK(ITEM("calibration parameters") << " --")
    STREAM_PACK(std::string(n, '-'))

    if (!Configor::DataStream::IMUTopics.empty()) {
        // -------------------------
        STREAM_PACK(ITEM("EXTRI"))
        // -------------------------
        STREAM_PACK("")

#define OUTPUT_EXTRINSICS(SENSOR1, IDX1, SENSOR2, IDX2)                                           \
    const auto EULER = EXTRI.EULER_##SENSOR1##IDX1##To##SENSOR2##IDX2##_DEG(topic);               \
    STREAM_PACK(PARAM("EULER_" #SENSOR1 #IDX1 "To" #SENSOR2 #IDX2 ": ")                           \
                << FormatValueVector<double>({"Xr", "Yp", "Zy"}, {EULER(0), EULER(1), EULER(2)})) \
                                                                                                  \
    const auto POS = EXTRI.POS_##SENSOR1##IDX1##In##SENSOR2##IDX2.at(topic);                      \
    STREAM_PACK(PARAM("  POS_" #SENSOR1 #IDX1 "In" #SENSOR2 #IDX2 ": ")                           \
                << FormatValueVector<double>({"Px", "Py", "Pz"}, {POS(0), POS(1), POS(2)}))

        // imus
        for (const auto &[topic, _] : Configor::DataStream::IMUTopics) {
            STREAM_PACK("IMU: '" << topic << "'")
            OUTPUT_EXTRINSICS(B, i, B, r)
            STREAM_PACK("")
        }

        // cameras
        for (const auto &[topic, _] : Configor::DataStream::EventTopics) {
            STREAM_PACK("Camera: '" << topic << "'")
            OUTPUT_EXTRINSICS(C, j, B, r)
            STREAM_PACK("")
        }

        STREAM_PACK(std::string(n, '-'))

        // ----------------------------
        STREAM_PACK(ITEM("TEMPORAL"))
        // ----------------------------
        STREAM_PACK("")

#define OUTPUT_TEMPORAL(SENSOR1, IDX1, SENSOR2, IDX2)                                           \
    const auto TO = TEMPORAL.TO_##SENSOR1##IDX1##To##SENSOR2##IDX2.at(topic);                   \
    STREAM_PACK(                                                                                \
        fmt::format("{}: {:+011.6f} (s)", PARAM("TO_" #SENSOR1 #IDX1 "To" #SENSOR2 #IDX2), TO)) \
                                                                                                \
    // imus
        for (const auto &[topic, _] : Configor::DataStream::IMUTopics) {
            STREAM_PACK("IMU: '" << topic << "'")
            OUTPUT_TEMPORAL(B, i, B, r)
            STREAM_PACK("")
        }

        // cameras
        for (const auto &[topic, _] : Configor::DataStream::EventTopics) {
            STREAM_PACK("Camera: '" << topic << "'")
            OUTPUT_TEMPORAL(C, j, B, r)
            STREAM_PACK("")
        }

        STREAM_PACK(std::string(n, '-'))
    }
    // -------------------------
    STREAM_PACK(ITEM("INTRI"))
    // -------------------------
    STREAM_PACK("")

    // imus
    for (const auto &[topic, _] : Configor::DataStream::IMUTopics) {
        STREAM_PACK("IMU: '" << topic << "'")
        const auto &ACCE = INTRI.IMU.at(topic)->ACCE;
        const auto &GYRO = INTRI.IMU.at(topic)->GYRO;
        // imu
        STREAM_PACK(PARAM("ACCE      BIAS: ") << FormatValueVector<double>(
                        {"Bx", "By", "Bz"}, {ACCE.BIAS(0), ACCE.BIAS(1), ACCE.BIAS(2)}))
        STREAM_PACK(
            PARAM("ACCE MAP COEFF: ") << FormatValueVector<double>(
                {"00", "11", "22"}, {ACCE.MAP_COEFF(0), ACCE.MAP_COEFF(1), ACCE.MAP_COEFF(2)}))
        STREAM_PACK(
            PARAM("                ") << FormatValueVector<double>(
                {"01", "02", "12"}, {ACCE.MAP_COEFF(3), ACCE.MAP_COEFF(4), ACCE.MAP_COEFF(5)}))

        STREAM_PACK("")

        STREAM_PACK(PARAM("GYRO      BIAS: ") << FormatValueVector<double>(
                        {"Bx", "By", "Bz"}, {GYRO.BIAS(0), GYRO.BIAS(1), GYRO.BIAS(2)}))
        STREAM_PACK(
            PARAM("GYRO MAP COEFF: ") << FormatValueVector<double>(
                {"00", "11", "22"}, {GYRO.MAP_COEFF(0), GYRO.MAP_COEFF(1), GYRO.MAP_COEFF(2)}))
        STREAM_PACK(
            PARAM("                ") << FormatValueVector<double>(
                {"01", "02", "12"}, {GYRO.MAP_COEFF(3), GYRO.MAP_COEFF(4), GYRO.MAP_COEFF(5)}))

        STREAM_PACK("")

        const auto EULER_AtoG = INTRI.IMU.at(topic)->EULER_AtoG_DEG();
        STREAM_PACK(PARAM("EULER AtoG DEG: ") << FormatValueVector<double>(
                        {"Xr", "Yp", "Zy"}, {EULER_AtoG(0), EULER_AtoG(1), EULER_AtoG(2)}))
        STREAM_PACK("")
    }

    // cameras
    for (const auto &[topic, intri] : INTRI.Camera) {
        STREAM_PACK("Camera: '" << topic << "'")
        const auto &pars = intri->GetParams();

        STREAM_PACK(
            PARAM("IMAGE     SIZE: ") << FormatValueVector<double>(
                {" w", " h"}, {(double)intri->imgWidth, (double)intri->imgHeight}, "{:+011.6f}"))

        STREAM_PACK(PARAM("FOCAL   LENGTH: ")
                    << FormatValueVector<double>({"fx", "fy"}, {pars.at(0), pars.at(1)}))

        STREAM_PACK(PARAM("PRINCIP  POINT: ")
                    << FormatValueVector<double>({"cx", "cy"}, {pars.at(2), pars.at(3)}))

        STREAM_PACK("")
        STREAM_PACK(PARAM("DISTO   PARAMS: ") << FormatValueVector<double>(
                        {"k1", "k2", "k3"}, {pars.at(4), pars.at(5), pars.at(6)}))
        STREAM_PACK(PARAM("                ")
                    << FormatValueVector<double>({"p1", "p2"}, {pars.at(7), pars.at(8)}))

        STREAM_PACK("")
    }

    STREAM_PACK(std::string(n, '-'))

    if (!Configor::DataStream::IMUTopics.empty()) {
        // ------------------------------
        STREAM_PACK(ITEM("OTHER FIELDS"))
        // ------------------------------
        STREAM_PACK("")
        STREAM_PACK(PARAM("GRAVITY IN MAP: ") << FormatValueVector<double>(
                        {"Gx", "Gy", "Gz"}, {GRAVITY(0), GRAVITY(1), GRAVITY(2)}))

        STREAM_PACK(std::string(n, '-'))
    }
    spdlog::info("the detail calibration parameters are below: \n{}", stream.str());

#undef ITEM
#undef PARAM
}

Viewer &CalibParamManager::VisualizationSensors(Viewer &viewer,
                                                const Sophus::SE3f &SE3_RefToWorld,
                                                const float &pScale) const {
    return viewer.AddEntityLocal(EntitiesForVisualization(SE3_RefToWorld, pScale));
}

void CalibParamManager::Save(const std::string &filename,
                             CerealArchiveType::Enum archiveType) const {
    std::ofstream file(filename, std::ios::out);
    auto ar = GetOutputArchiveVariant(file, archiveType);
    SerializeByOutputArchiveVariant(ar, archiveType, cereal::make_nvp("CalibParam", *this));
}

CalibParamManager::Ptr CalibParamManager::Load(const std::string &filename,
                                               CerealArchiveType::Enum archiveType) {
    auto calibParamManager = CalibParamManager::Create();
    std::ifstream file(filename, std::ios::in);
    auto ar = GetInputArchiveVariant(file, archiveType);
    SerializeByInputArchiveVariant(ar, archiveType,
                                   cereal::make_nvp("CalibParam", *calibParamManager));
    return calibParamManager;
}

std::vector<ns_viewer::Entity::Ptr> CalibParamManager::EntitiesForVisualization(
    const Sophus::SE3f &SE3_RefToWorld, const float &pScale) const {
#define IMU_SIZE 0.02
#define CAMERA_SIZE 0.04

    std::vector<ns_viewer::Entity::Ptr> entities;

    // reference imu
    auto SE3_BrToW = SE3_RefToWorld;
    SE3_BrToW.translation() *= pScale;
    auto refIMU =
        ns_viewer::IMU::Create(ns_viewer::Posef(SE3_BrToW.so3().matrix(), SE3_BrToW.translation()),
                               IMU_SIZE, ns_viewer::Colour(0.3f, 0.3f, 0.3f, 1.0f));
    entities.push_back(refIMU);

    // imus
    for (const auto &[topic, _] : Configor::DataStream::IMUTopics) {
        auto SE3_BiToW = SE3_RefToWorld * EXTRI.SE3_BiToBr(topic).cast<float>();
        SE3_BiToW.translation() *= pScale;
        auto imu = ns_viewer::IMU::Create(
            ns_viewer::Posef(SE3_BiToW.so3().matrix(), SE3_BiToW.translation()), IMU_SIZE,
            ns_viewer::Colour::Red());
        auto line = ns_viewer::Line::Create(SE3_BrToW.translation(), SE3_BiToW.translation(),
                                            ns_viewer::Colour::Black());
        entities.push_back(imu);
        entities.push_back(line);
    }

    // cameras
    for (const auto &[topic, _] : Configor::DataStream::EventTopics) {
        auto SE3_CjToW = SE3_RefToWorld * EXTRI.SE3_CjToBr(topic).cast<float>();
        SE3_CjToW.translation() *= pScale;
        auto camera = ns_viewer::CubeCamera::Create(
            ns_viewer::Posef(SE3_CjToW.so3().matrix(), SE3_CjToW.translation()), CAMERA_SIZE,
            ns_viewer::Colour::Blue());
        auto line = ns_viewer::Line::Create(SE3_BrToW.translation(), SE3_CjToW.translation(),
                                            ns_viewer::Colour::Black());
        entities.push_back(camera);
        entities.push_back(line);
    }
#undef IMU_SIZE
#undef CAMERA_SIZE

    return entities;
}

ns_veta::PinholeIntrinsic::Ptr CalibParamManager::ParIntri::LoadCameraIntri(
    const std::string &filename, CerealArchiveType::Enum archiveType) {
    auto intri = ns_veta::PinholeIntrinsic::Create(0, 0, 0, 0, 0, 0);
    std::ifstream file(filename, std::ios::in);
    auto ar = GetInputArchiveVariant(file, archiveType);
    try {
        SerializeByInputArchiveVariant(ar, archiveType, cereal::make_nvp("Intrinsics", intri));
    } catch (const cereal::Exception &exception) {
        throw Status(Status::CRITICAL,
                     "The configuration file '{}' for 'PinholeIntrinsic' is "
                     "outdated or broken, and can not be loaded in iKalibr using cereal!!! "
                     "To make it right, please refer to our latest configuration file "
                     "template released at "
                     "https://github.com/Unsigned-Long/iKalibr/blob/master/config/"
                     "cam-intri-pinhole-fisheye.yaml and "
                     "https://github.com/Unsigned-Long/iKalibr/blob/master/config/"
                     "cam-intri-pinhole-brown.yaml, and then fix your custom configuration file. "
                     "Detailed cereal exception information: \n'{}'",
                     filename, exception.what());
    }
    return intri;
}

IMUIntrinsics::Ptr CalibParamManager::ParIntri::LoadIMUIntri(const std::string &filename,
                                                             CerealArchiveType::Enum archiveType) {
    auto intri = IMUIntrinsics::Create();
    std::ifstream file(filename, std::ios::in);
    auto ar = GetInputArchiveVariant(file, archiveType);
    try {
        SerializeByInputArchiveVariant(ar, archiveType, cereal::make_nvp("Intrinsics", intri));
    } catch (const cereal::Exception &exception) {
        throw Status(Status::CRITICAL,
                     "The configuration file '{}' for 'IMUIntrinsics' is "
                     "outdated or broken, and can not be loaded in iKalibr using cereal!!! "
                     "To make it right, please refer to our latest configuration file "
                     "template released at "
                     "https://github.com/Unsigned-Long/iKalibr/blob/master/config/imu-intri.yaml, "
                     "and then fix your custom configuration file. Detailed cereal exception "
                     "information: \n'{}'",
                     filename, exception.what());
    }
    return intri;
}

void CalibParamManager::ParIntri::SaveCameraIntri(const ns_veta::PinholeIntrinsic::Ptr &intri,
                                                  const std::string &filename,
                                                  CerealArchiveType::Enum archiveType) {
    std::ofstream file(filename, std::ios::out);
    auto ar = GetOutputArchiveVariant(file, archiveType);
    SerializeByOutputArchiveVariant(ar, archiveType, cereal::make_nvp("Intrinsics", intri));
}

void CalibParamManager::ParIntri::SaveIMUIntri(const IMUIntrinsics::Ptr &intri,
                                               const std::string &filename,
                                               CerealArchiveType::Enum archiveType) {
    std::ofstream file(filename, std::ios::out);
    auto ar = GetOutputArchiveVariant(file, archiveType);
    SerializeByOutputArchiveVariant(ar, archiveType, cereal::make_nvp("Intrinsics", intri));
}
}  // namespace ns_ekalibr
