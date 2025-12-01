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

#include "sensor/frame_rosbag_loader.h"
#include "util/status.hpp"
#include "rosbag/message_instance.h"
#include "cv_bridge/cv_bridge.h"
#include "rosbag/view.h"
#include "util/tqdm.h"
#include "filesystem"

std::map<std::string, std::vector<ns_ekalibr::Frame::Ptr>> ns_ekalibr::LoadFramesFromROSBag(
    rosbag::Bag* bag,
    const std::map<std::string, std::string>& topics,
    const ros::Time& begTime,
    const ros::Time& endTime) {
    if (topics.empty() || bag == nullptr) {
        return {};
    }
    auto view = rosbag::View();
    std::vector<std::string> topicsToQuery;
    // add topics to vector
    for (const auto& [topic, _] : topics) {
        topicsToQuery.push_back(topic);
    }
    view.addQuery(*bag, rosbag::TopicQuery(topicsToQuery), begTime, endTime);
    // create data loader
    std::map<std::string, FrameDataLoader::Ptr> frameDataLoaders;
    std::map<std::string, std::vector<Frame::Ptr>> frameMes;

    auto MessageNumInTopic = [](const rosbag::Bag* bag, const std::string& topic,
                                const ros::Time& begTime, const ros::Time& endTime) {
        auto view = rosbag::View();
        std::vector<std::string> topicsToQuery = {topic};
        view.addQuery(*bag, rosbag::TopicQuery(topicsToQuery), begTime, endTime);
        return view.size();
    };

    // get type enum from the string
    for (const auto& [topic, type] : topics) {
        frameDataLoaders.insert({topic, FrameDataLoader::GetLoader(type)});
        // reserve tp speed up the data loading
        auto size = MessageNumInTopic(bag, topic, begTime, endTime);
        if (size > 0) {
            frameMes[topic].reserve(size);
        }
    }

    // read raw data
    auto bar = std::make_shared<tqdm>();
    int idx = 0;
    for (auto iter = view.begin(); iter != view.end(); ++iter, ++idx) {
        bar->progress(idx, static_cast<int>(view.size()));
        const auto& item = *iter;
        const std::string& topic = item.getTopic();
        if (frameDataLoaders.find(topic) != frameDataLoaders.end()) {
            auto frame = frameDataLoaders[topic]->UnpackData(item);
            if (frame == nullptr) {
                continue;
            }
            if (!frameMes[topic].empty() &&
                frameMes[topic].back()->GetTimestamp() >= frame->GetTimestamp()) {
                spdlog::warn(
                    "the frame data in topic '{}' is not time-ordered, "
                    "skip the current data at time '{:.6f}' (s)!",
                    topic, frame->GetTimestamp());
                continue;
            }
            frameMes[topic].push_back(frame);
        }
    }
    bar->finish();
    return frameMes;
}

std::map<std::string, std::vector<ns_ekalibr::Frame::Ptr>> ns_ekalibr::LoadFramesFromROSBag(
    const std::string& bagPath,
    const std::map<std::string, std::string>& topics,
    double beginTime,
    double duration) {
    // open the ros bag
    auto bag = std::make_unique<rosbag::Bag>();
    if (!std::filesystem::exists(bagPath)) {
        spdlog::error("the ros bag path '{}' is invalid!", bagPath);
    } else {
        bag->open(bagPath, rosbag::BagMode::Read);
    }

    // using a temp view to check the time range of the source ros bag
    auto viewTemp = rosbag::View();

    std::vector<std::string> topicsToQuery;
    // add topics to vector
    for (const auto& [topic, _] : topics) {
        topicsToQuery.push_back(topic);
    }

    viewTemp.addQuery(*bag, rosbag::TopicQuery(topicsToQuery));
    auto begTime = viewTemp.getBeginTime();
    auto endTime = viewTemp.getEndTime();
    spdlog::info("source data duration: from '{:.5f}' to '{:.5f}'.", begTime.toSec(),
                 endTime.toSec());

    // adjust the data time range
    if (beginTime > 0.0) {
        begTime += ros::Duration(beginTime);
        if (begTime > endTime) {
            spdlog::warn(
                "begin time '{:.5f}' is out of the bag's data range, set begin time to '{:.5f}'.",
                begTime.toSec(), viewTemp.getBeginTime().toSec());
            begTime = viewTemp.getBeginTime();
        }
    }
    if (duration > 0.0) {
        endTime = begTime + ros::Duration(duration);
        if (endTime > viewTemp.getEndTime()) {
            spdlog::warn(
                "end time '{:.5f}' is out of the bag's data range, set end time to '{:.5f}'.",
                endTime.toSec(), viewTemp.getEndTime().toSec());
            endTime = viewTemp.getEndTime();
        }
    }
    spdlog::info("expect data duration: from '{:.5f}' to '{:.5f}'.", begTime.toSec(),
                 endTime.toSec());

    auto mes = LoadFramesFromROSBag(bag.get(), topics, begTime, endTime);
    bag->close();

    return mes;
}

ns_ekalibr::FrameDataLoader::FrameDataLoader(FrameModelType& model)
    : _model(model) {}

ns_ekalibr::FrameDataLoader::Ptr ns_ekalibr::FrameDataLoader::GetLoader(
    const std::string& modelStr) {
    FrameModelType model = FrameModel::FromString(modelStr);
    FrameDataLoader::Ptr loader = nullptr;
    switch (model) {
        case FrameModelType::SENSOR_IMAGE:
            loader = SensorImageLoader::Create(model);
            break;
        case FrameModelType::SENSOR_IMAGE_COMP:
            loader = SensorImageCompLoader::Create(model);
            break;
        default:
            throw Status(Status::ERROR, FrameModel::UnsupportedFrameModelMsg(modelStr));
    }
    return loader;
}
ns_ekalibr::FrameModelType ns_ekalibr::FrameDataLoader::GetFrameModel() const { return _model; }

// -----------------
// SensorImageLoader
// -----------------
ns_ekalibr::SensorImageLoader::SensorImageLoader(FrameModelType model)
    : FrameDataLoader(model) {}

ns_ekalibr::SensorImageLoader::Ptr ns_ekalibr::SensorImageLoader::Create(FrameModelType model) {
    return std::make_shared<SensorImageLoader>(model);
}

ns_ekalibr::Frame::Ptr ns_ekalibr::SensorImageLoader::UnpackData(
    const rosbag::MessageInstance& msgInstance) {
    sensor_msgs::ImagePtr msg = msgInstance.instantiate<sensor_msgs::Image>();

    CheckMessage<sensor_msgs::Image>(msg);

    if (msg->encoding != sensor_msgs::image_encodings::MONO8 &&
        msg->encoding != sensor_msgs::image_encodings::BGR8) {
        throw Status(Status::ERROR,
                     fmt::format("Unsupported image encoding type: '{}'", msg->encoding));
    }

    cv::Mat img;
    cv_bridge::toCvCopy(msg)->image.copyTo(img);

    // check if is gray or color image
    if (img.channels() != 1 && img.channels() != 3) {
        throw Status(Status::ERROR,
                     fmt::format("Unsupported image channels number: '{}'", img.channels()));
    }
    // check 8u
    if (img.depth() != CV_8U) {
        throw Status(Status::ERROR, fmt::format("Unsupported image depth type: '{}'", img.depth()));
    }

    return Frame::Create(msg->header.stamp.toSec(), img);
}

// ---------------------
// SensorImageCompLoader
// ---------------------
ns_ekalibr::SensorImageCompLoader::SensorImageCompLoader(FrameModelType model)
    : FrameDataLoader(model) {}

ns_ekalibr::SensorImageCompLoader::Ptr ns_ekalibr::SensorImageCompLoader::Create(
    FrameModelType model) {
    return std::make_shared<SensorImageCompLoader>(model);
}

ns_ekalibr::Frame::Ptr ns_ekalibr::SensorImageCompLoader::UnpackData(
    const rosbag::MessageInstance& msgInstance) {
    sensor_msgs::CompressedImageConstPtr msg =
        msgInstance.instantiate<sensor_msgs::CompressedImage>();

    CheckMessage<sensor_msgs::CompressedImage>(msg);

    cv::Mat img;
    cv_bridge::toCvCopy(msg)->image.copyTo(img);

    // check if is gray or color image
    if (img.channels() != 1 && img.channels() != 3) {
        throw Status(Status::ERROR,
                     fmt::format("Unsupported image channels number: '{}'", img.channels()));
    }
    // check 8u
    if (img.depth() != CV_8U) {
        throw Status(Status::ERROR, fmt::format("Unsupported image depth type: '{}'", img.depth()));
    }

    return Frame::Create(msg->header.stamp.toSec(), img);
}