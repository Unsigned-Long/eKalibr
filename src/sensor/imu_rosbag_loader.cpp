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

#include "sensor/imu_rosbag_loader.h"
#include "ekalibr/SbgImuData.h"
#include "util/status.hpp"
#include "sensor_msgs/Imu.h"
#include "rosbag/message_instance.h"
#include "rosbag/view.h"
#include "util/tqdm.h"

namespace ns_ekalibr {
std::map<std::string, std::vector<IMUFrame::Ptr>> LoadIMUDataFromROSBag(
    rosbag::Bag *bag,
    const std::map<std::string, std::string> &topics,
    double gravityNorm,
    const ros::Time &begTime,
    const ros::Time &endTime) {
    if (topics.empty() || bag == nullptr) {
        return {};
    }
    auto view = rosbag::View();

    std::vector<std::string> topicsToQuery;
    // add topics to vector
    for (const auto &[topic, _] : topics) {
        topicsToQuery.push_back(topic);
    }
    view.addQuery(*bag, rosbag::TopicQuery(topicsToQuery), begTime, endTime);

    // create data loader
    std::map<std::string, IMUDataLoader::Ptr> imuDataLoaders;
    std::map<std::string, std::vector<IMUFrame::Ptr>> imuMes;

    auto MessageNumInTopic = [](const rosbag::Bag *bag, const std::string &topic,
                                const ros::Time &begTime, const ros::Time &endTime) {
        auto view = rosbag::View();
        view.addQuery(*bag, rosbag::TopicQuery({topic}), begTime, endTime);
        return view.size();
    };

    // get type enum from the string
    for (const auto &[topic, type] : topics) {
        imuDataLoaders.insert({topic, IMUDataLoader::GetLoader(type, gravityNorm)});
        // reserve tp speed up the data loading
        auto size = MessageNumInTopic(bag, topic, begTime, endTime);
        if (size > 0) {
            imuMes[topic].reserve(size);
        }
    }

    // read raw data
    auto bar = std::make_shared<tqdm>();
    int idx = 0;
    for (auto iter = view.begin(); iter != view.end(); ++iter, ++idx) {
        bar->progress(idx, static_cast<int>(view.size()));
        const auto &item = *iter;
        const std::string &topic = item.getTopic();
        if (imuDataLoaders.cend() != imuDataLoaders.find(topic)) {
            // an event array
            auto mes = imuDataLoaders.at(topic)->UnpackData(item);
            if (mes == nullptr) {
                continue;
            }
            if (!imuMes.at(topic).empty() &&
                imuMes.at(topic).back()->GetTimestamp() >= mes->GetTimestamp()) {
                spdlog::warn(
                    "imu measurement time disorder detected in topic '{}', "
                    "skip the measurement at time '{:.6f}' (s)!",
                    topic, mes->GetTimestamp());
                continue;
            }
            imuMes.at(topic).push_back(mes);
        }
    }
    bar->finish();

    return imuMes;
}

IMUDataLoader::IMUDataLoader(IMUModelType imuModel)
    : _imuModel(imuModel) {}

IMUDataLoader::Ptr IMUDataLoader::GetLoader(const std::string &imuModelStr,
                                            const double gravityNorm) {
    // try extract radar model
    IMUModelType imuModel = IMUModel::FromString(imuModelStr);

    IMUDataLoader::Ptr dataLoader;

    switch (imuModel) {
        case IMUModelType::SBG_IMU:
            dataLoader = SbgIMULoader::Create(imuModel);
            break;
        case IMUModelType::SENSOR_IMU:
            dataLoader = SensorIMULoader::Create(imuModel, 1.0, 1.0);
            break;
        case IMUModelType::SENSOR_IMU_G:
            dataLoader = SensorIMULoader::Create(imuModel, 1.0, gravityNorm);
            break;
        case IMUModelType::SENSOR_IMU_G_NEG:
            dataLoader = SensorIMULoader::Create(imuModel, 1.0, -gravityNorm);
            break;
        case IMUModelType::SENSOR_IMU_DEG:
            dataLoader = SensorIMULoader::Create(imuModel, DEG_TO_RAD, 1.0);
            break;
        case IMUModelType::SENSOR_IMU_DEG_G:
            dataLoader = SensorIMULoader::Create(imuModel, DEG_TO_RAD, gravityNorm);
            break;
        case IMUModelType::SENSOR_IMU_DEG_G_NEG:
            dataLoader = SensorIMULoader::Create(imuModel, DEG_TO_RAD, -gravityNorm);
            break;
        default:
            throw Status(Status::WARNING, IMUModel::UnsupportedIMUModelMsg(imuModelStr));
    }
    return dataLoader;
}

IMUModelType IMUDataLoader::GetIMUModel() const { return _imuModel; }

// ---------------
// SensorIMULoader
// ---------------
SensorIMULoader::SensorIMULoader(IMUModelType imuModel, double g2StdUnit, double a2StdUnit)
    : IMUDataLoader(imuModel),
      g2StdUnit(g2StdUnit),
      a2StdUnit(a2StdUnit) {}

SensorIMULoader::Ptr SensorIMULoader::Create(IMUModelType imuModel,
                                             double g2StdUnit,
                                             double a2StdUnit) {
    return std::make_shared<SensorIMULoader>(imuModel, g2StdUnit, a2StdUnit);
}

IMUFrame::Ptr SensorIMULoader::UnpackData(const rosbag::MessageInstance &msgInstance) {
    // imu data item
    sensor_msgs::ImuConstPtr msg = msgInstance.instantiate<sensor_msgs::Imu>();

    CheckMessage<sensor_msgs::Imu>(msg);

    Eigen::Vector3d acce =
        a2StdUnit * Eigen::Vector3d(msg->linear_acceleration.x, msg->linear_acceleration.y,
                                    msg->linear_acceleration.z);
    Eigen::Vector3d gyro =
        g2StdUnit *
        Eigen::Vector3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    if (msg->header.stamp.isZero()) {
        Status(Status::WARNING, "inertial measurement with zero timestamp exists!!!");
    }
    return IMUFrame::Create(msg->header.stamp.toSec(), gyro, acce);
}

// ------------
// SbgIMULoader
// ------------
SbgIMULoader::SbgIMULoader(IMUModelType imuModel)
    : IMUDataLoader(imuModel) {}

SbgIMULoader::Ptr SbgIMULoader::Create(IMUModelType imuModel) {
    return std::make_shared<SbgIMULoader>(imuModel);
}

IMUFrame::Ptr SbgIMULoader::UnpackData(const rosbag::MessageInstance &msgInstance) {
    // imu data item
    ekalibr::SbgImuData::ConstPtr msg = msgInstance.instantiate<ekalibr::SbgImuData>();

    CheckMessage<ekalibr::SbgImuData>(msg);

    Eigen::Vector3d acce = Eigen::Vector3d(msg->accel.x, msg->accel.y, msg->accel.z);
    Eigen::Vector3d gyro = Eigen::Vector3d(msg->gyro.x, msg->gyro.y, msg->gyro.z);
    if (msg->header.stamp.isZero()) {
        return IMUFrame::Create(msg->time_stamp * 1E-6, gyro, acce);
    } else {
        return IMUFrame::Create(msg->header.stamp.toSec(), gyro, acce);
    }
}
}  // namespace ns_ekalibr
