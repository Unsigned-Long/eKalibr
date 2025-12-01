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

#include "sensor/sensor_model.h"
#include "spdlog/fmt/fmt.h"
#include "util/enum_cast.hpp"
#include "util/status.hpp"

namespace ns_ekalibr {

std::string IMUModel::UnsupportedIMUModelMsg(const std::string& modelStr) {
    return fmt::format(
        "Unsupported IMU Type: '{}'. "
        "Currently supported IMU types are: \n"
        "1.              SBG_IMU: https://github.com/SBG-Systems/sbg_ros_driver.git\n"
        "2.           SENSOR_IMU: gyro unit (rad/s), acce unit (m/s^2), "
        "https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Imu.html\n"
        "3.         SENSOR_IMU_G: gyro unit (rad/s), acce unit (G)\n"
        "4.     SENSOR_IMU_G_NEG: gyro unit (rad/s), acce unit (-G)\n"
        "5.       SENSOR_IMU_DEG: gyro unit (deg/s), acce unit (m/s^2), "
        "https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Imu.html\n"
        "6.     SENSOR_IMU_DEG_G: gyro unit (deg/s), acce unit (G)\n"
        "7. SENSOR_IMU_DEG_G_NEG: gyro unit (deg/s), acce unit (-G)\n"
        "...\n"
        "If you need to use other IMU types, "
        "please 'Issues' us on the profile of the github repository.",
        modelStr);
}

IMUModelType IMUModel::FromString(const std::string& modelStr) {
    IMUModelType model;
    try {
        model = EnumCast::stringToEnum<IMUModelType>(modelStr);
    } catch (...) {
        throw Status(Status::ERROR, UnsupportedIMUModelMsg(modelStr));
    }
    return model;
}
std::string IMUModel::ToString(IMUModelType model) {
    std::string modelStr;
    try {
        modelStr = EnumCast::enumToString(model);
    } catch (...) {
        throw Status(Status::ERROR, UnsupportedIMUModelMsg(modelStr));
    }
    return modelStr;
}

std::string EventModel::UnsupportedEventModelMsg(const std::string& modelStr) {
    return fmt::format(
        "Unsupported Event Camera Type: '{}'. "
        "Currently supported event camera types are: \n"
        "1. PROPHESEE_EVENT: https://github.com/prophesee-ai/prophesee_ros_wrapper.git\n"
        "2.       DVS_EVENT: https://github.com/uzh-rpg/rpg_dvs_ros.git\n"
        "...\n"
        "If you need to use other event camera types, "
        "please 'Issues' us on the profile of the github repository.",
        modelStr);
}

EventModelType EventModel::FromString(const std::string& modelStr) {
    EventModelType model;
    try {
        model = EnumCast::stringToEnum<EventModelType>(modelStr);
    } catch (...) {
        throw Status(Status::ERROR, UnsupportedEventModelMsg(modelStr));
    }
    return model;
}

std::string EventModel::ToString(EventModelType model) {
    std::string modelStr;
    try {
        modelStr = EnumCast::enumToString(model);
    } catch (...) {
        throw Status(Status::ERROR, UnsupportedEventModelMsg(modelStr));
    }
    return modelStr;
}
std::string FrameModel::UnsupportedFrameModelMsg(const std::string& modelStr) {
    return fmt::format(
        "Unsupported frame model: '{}'. "
        "Currently supported frame models are: \n"
        "1. SENSOR_IMAGE: https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html\n"
        "2. SENSOR_IMAGE_COMP: "
        "https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/CompressedImage.html\n"
        "...\n",
        modelStr);
}

FrameModel::FrameModelType FrameModel::FromString(const std::string& modelStr) {
    FrameModelType model;
    try {
        model = EnumCast::stringToEnum<FrameModelType>(modelStr);
    } catch (...) {
        throw Status(Status::ERROR, UnsupportedFrameModelMsg(modelStr));
    }
    return model;
}

std::string FrameModel::ToString(const FrameModelType& model) {
    std::string modelStr;
    try {
        modelStr = EnumCast::enumToString(model);
    } catch (...) {
        throw Status(Status::ERROR, UnsupportedFrameModelMsg(modelStr));
    }
    return modelStr;
}

std::string CirclePattern::UnsupportedCirclePatternMsg(const std::string& modelStr) {
    return fmt::format(
        "Unsupported circle pattern: '{}'. "
        "Currently supported circle patterns are: \n"
        "1.  SYMMETRIC_GRID: https://www.mathworks.com/help/vision/ug/calibration-patterns.html\n"
        "2. ASYMMETRIC_GRID: https://www.mathworks.com/help/vision/ug/calibration-patterns.html\n"
        "...\n",
        modelStr);
}

CirclePatternType CirclePattern::FromString(const std::string& modelStr) {
    CirclePatternType model;
    try {
        model = EnumCast::stringToEnum<CirclePatternType>(modelStr);
    } catch (...) {
        throw Status(Status::ERROR, UnsupportedCirclePatternMsg(modelStr));
    }
    return model;
}

std::string CirclePattern::ToString(const CirclePatternType& model) {
    std::string modelStr;
    try {
        modelStr = EnumCast::enumToString(model);
    } catch (...) {
        throw Status(Status::ERROR, UnsupportedCirclePatternMsg(modelStr));
    }
    return modelStr;
}

}  // namespace ns_ekalibr
