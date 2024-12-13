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

#include "core/calib_solver.h"
#include "util/utils.h"
#include <utility>
#include "sensor/event_rosbag_loader.h"
#include "sensor/imu_rosbag_loader.h"
#include "config/configor.h"
#include "pangolin/display/display.h"
#include "viewer/viewer.h"
#include "util/status.hpp"
#include "spdlog/spdlog.h"
#include "tiny-viewer/core/pose.hpp"
#include "rosbag/bag.h"
#include "filesystem"
#include "rosbag/view.h"

namespace ns_ekalibr {
CalibSolver::CalibSolver(CalibParamManagerPtr parMgr)
    : _parMgr(std::move(parMgr)),
      _viewer(Configor::Preference::Visualization
                  ? Viewer::Create(Configor::Preference::MaxEntityCountInViewer)
                  : nullptr),
      _solveFinished(false),
      _viewCamPose(Eigen::Matrix3f::Identity(), {0.0f, 0.0f, -4.0f}) {
    // organize the default solver option
    _ceresOption.minimizer_type = ceres::TRUST_REGION;
    _ceresOption.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    _ceresOption.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    _ceresOption.minimizer_progress_to_stdout = false;
}

CalibSolver::Ptr CalibSolver::Create(const CalibParamManagerPtr &parMgr) {
    return std::make_shared<CalibSolver>(parMgr);
}

CalibSolver::~CalibSolver() {
    // solving is not performed or not finished as an exception is thrown
    if (Configor::Preference::Visualization && !_solveFinished) {
        pangolin::QuitAll();
    }
    // solving is finished (when use 'pangolin::QuitAll()', the window not quit immediately)
    while (Configor::Preference::Visualization && _viewer->IsActive()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void CalibSolver::LoadDataFromRosBag() {
    /**
     * load raw event data from rosbag
     */

    // open the ros bag
    auto bag = std::make_unique<rosbag::Bag>();
    if (!std::filesystem::exists(Configor::DataStream::BagPath)) {
        spdlog::error("the ros bag path '{}' is invalid!", Configor::DataStream::BagPath);
    } else {
        bag->open(Configor::DataStream::BagPath, rosbag::BagMode::Read);
    }

    // using a temp view to check the time range of the source ros bag
    auto viewTemp = rosbag::View();

    std::vector<std::string> topicsToQuery;
    std::map<std::string, std::string> imuTopicTypeMap, evTopicTypeMap;
    // add topics to vector
    for (const auto &[topic, info] : Configor::DataStream::EventTopics) {
        topicsToQuery.push_back(topic);
        evTopicTypeMap[topic] = info.Type;
    }
    for (const auto &[topic, info] : Configor::DataStream::IMUTopics) {
        topicsToQuery.push_back(topic);
        imuTopicTypeMap[topic] = info.Type;
    }

    viewTemp.addQuery(*bag, rosbag::TopicQuery(topicsToQuery));
    auto begTime = viewTemp.getBeginTime();
    auto endTime = viewTemp.getEndTime();
    spdlog::info("source data duration: from '{:.5f}' to '{:.5f}'.", begTime.toSec(),
                 endTime.toSec());

    // adjust the data time range
    if (Configor::DataStream::BeginTime > 0.0) {
        begTime += ros::Duration(Configor::DataStream::BeginTime);
        if (begTime > endTime) {
            spdlog::warn(
                "begin time '{:.5f}' is out of the bag's data range, set begin time to '{:.5f}'.",
                begTime.toSec(), viewTemp.getBeginTime().toSec());
            begTime = viewTemp.getBeginTime();
        }
    }
    if (Configor::DataStream::Duration > 0.0) {
        endTime = begTime + ros::Duration(Configor::DataStream::Duration);
        if (endTime > viewTemp.getEndTime()) {
            spdlog::warn(
                "end time '{:.5f}' is out of the bag's data range, set end time to '{:.5f}'.",
                endTime.toSec(), viewTemp.getEndTime().toSec());
            endTime = viewTemp.getEndTime();
        }
    }
    spdlog::info("expect data duration: from '{:.5f}' to '{:.5f}'.", begTime.toSec(),
                 endTime.toSec());

    spdlog::info("loading event data from rosbag...");
    _evMes = LoadEventsFromROSBag(bag.get(), evTopicTypeMap, begTime, endTime);
    spdlog::info("loading imu data from rosbag...");
    _imuMes = LoadIMUDataFromROSBag(bag.get(), imuTopicTypeMap, begTime, endTime);

    bag->close();

    /**
     * select event data range
     */
    std::list<double> sTimeList, eTimeList;
    for (const auto &[topic, mes] : _evMes) {
        sTimeList.push_back(mes.front()->GetTimestamp());
        eTimeList.push_back(mes.back()->GetTimestamp());
    }
    for (const auto &[topic, mes] : _imuMes) {
        sTimeList.push_back(mes.front()->GetTimestamp());
        eTimeList.push_back(mes.back()->GetTimestamp());
    }
    _dataRawTimestamp.first = *std::max_element(sTimeList.begin(), sTimeList.end());
    _dataRawTimestamp.second = *std::min_element(eTimeList.begin(), eTimeList.end());

    for (const auto &[topic, _] : Configor::DataStream::EventTopics) {
        // remove event data arrays that are before the start time stamp
        EraseSeqHeadData(
            _evMes.at(topic),
            [this](const EventArray::Ptr &ary) {
                return ary->GetTimestamp() > _dataRawTimestamp.first - 1E-9;
            },
            "the event data is invalid, there is no data intersection between sensors.");

        // remove event data arrays that are after the end time stamp
        EraseSeqTailData(
            _evMes.at(topic),
            [this](const EventArray::Ptr &ary) {
                return ary->GetTimestamp() < _dataRawTimestamp.second + 1E-9;
            },
            "the event data is invalid, there is no data intersection between sensors.");
    }
    for (const auto &[topic, _] : Configor::DataStream::IMUTopics) {
        // remove imu data arrays that are before the start time stamp
        EraseSeqHeadData(
            _imuMes.at(topic),
            [this](const IMUFrame::Ptr &ary) {
                return ary->GetTimestamp() > _dataRawTimestamp.first - 1E-9;
            },
            "the imu data is invalid, there is no data intersection between sensors.");

        // remove imu data arrays that are after the end time stamp
        EraseSeqTailData(
            _imuMes.at(topic),
            [this](const IMUFrame::Ptr &ary) {
                return ary->GetTimestamp() < _dataRawTimestamp.second + 1E-9;
            },
            "the imu data is invalid, there is no data intersection between sensors.");
    }

    /**
     * align mes data
     */
    // all time stamps minus  '_rawStartTimestamp'
    _dataAlignedTimestamp.first = 0.0;
    _dataAlignedTimestamp.second = _dataRawTimestamp.second - _dataRawTimestamp.first;
    for (const auto &[eventTopic, mes] : _evMes) {
        for (const auto &array : mes) {
            // array
            array->SetTimestamp(array->GetTimestamp() - _dataRawTimestamp.first);
            // targets
            for (auto &tar : array->GetEvents()) {
                tar->SetTimestamp(tar->GetTimestamp() - _dataRawTimestamp.first);
            }
        }
    }
    for (const auto &[imuTopic, mes] : _imuMes) {
        for (const auto &array : mes) {
            // array
            array->SetTimestamp(array->GetTimestamp() - _dataRawTimestamp.first);
        }
    }

    this->OutputDataStatus();
}

void CalibSolver::OutputDataStatus() const {
    spdlog::info("calibration data info:");
    spdlog::info("raw start time: '{:+010.5f}' (s), raw end time: '{:+010.5f}' (s)",
                 _dataRawTimestamp.first, _dataRawTimestamp.second);
    spdlog::info("aligned start time: '{:+010.5f}' (s), aligned end time: '{:+010.5f}' (s)",
                 _dataAlignedTimestamp.first, _dataAlignedTimestamp.second);
    for (const auto &[topic, mes] : _evMes) {
        spdlog::info(
            "Event topic: '{}', data size: '{:06}', time span: from '{:+010.5f}' to '{:+010.5f}' "
            "(s)",
            topic, mes.size(), mes.front()->GetTimestamp(), mes.back()->GetTimestamp());
    }
    for (const auto &[topic, mes] : _imuMes) {
        spdlog::info(
            "IMU topic: '{}', data size: '{:06}', time span: from '{:+010.5f}' to '{:+010.5f}' "
            "(s)",
            topic, mes.size(), mes.front()->GetTimestamp(), mes.back()->GetTimestamp());
    }
}

std::string CalibSolver::GetDiskPathOfExtractedGridPatterns(const std::string &topic) {
    const std::string dir = Configor::DataStream::OutputPath + "/" + topic;
    if (!TryCreatePath(dir)) {
        spdlog::info(
            "find directory '{}' to save/load extracted circle grid patterns of '{}' failed!!!",
            dir, topic);
        return {};
    }
    const auto &e = Configor::Preference::FileExtension.at(Configor::Preference::OutputDataFormat);
    return dir + "/patterns" + e;
}

std::string CalibSolver::GetDiskPathOfOpenCVIntrinsicCalibRes(const std::string &topic) {
    const std::string dir = Configor::DataStream::OutputPath + "/" + topic;
    if (!TryCreatePath(dir)) {
        spdlog::info(
            "find directory '{}' to save/load extracted circle grid patterns of '{}' failed!!!",
            dir, topic);
        return {};
    }
    const auto &e = Configor::Preference::FileExtension.at(Configor::Preference::OutputDataFormat);
    return dir + "/intrinsics" + e;
}

void CalibSolver::EraseSeqHeadData(std::vector<EventArrayPtr> &seq,
                                   std::function<bool(const EventArrayPtr &)> pred,
                                   const std::string &errorMsg) const {
    auto iter = std::find_if(seq.begin(), seq.end(), std::move(pred));
    if (iter == seq.end()) {
        // find failed
        this->OutputDataStatus();
        throw Status(Status::ERROR, errorMsg);
    } else {
        // adjust
        seq.erase(seq.begin(), iter);
    }
}

void CalibSolver::EraseSeqTailData(std::vector<EventArrayPtr> &seq,
                                   std::function<bool(const EventArrayPtr &)> pred,
                                   const std::string &errorMsg) const {
    auto iter = std::find_if(seq.rbegin(), seq.rend(), pred);
    if (iter == seq.rend()) {
        // find failed
        this->OutputDataStatus();
        throw Status(Status::ERROR, errorMsg);
    } else {
        // adjust
        seq.erase(iter.base(), seq.end());
    }
}

void CalibSolver::EraseSeqHeadData(std::vector<IMUFramePtr> &seq,
                                   std::function<bool(const IMUFramePtr &)> pred,
                                   const std::string &errorMsg) const {
    auto iter = std::find_if(seq.begin(), seq.end(), std::move(pred));
    if (iter == seq.end()) {
        // find failed
        this->OutputDataStatus();
        throw Status(Status::ERROR, errorMsg);
    } else {
        // adjust
        seq.erase(seq.begin(), iter);
    }
}

void CalibSolver::EraseSeqTailData(std::vector<IMUFramePtr> &seq,
                                   std::function<bool(const IMUFramePtr &)> pred,
                                   const std::string &errorMsg) const {
    auto iter = std::find_if(seq.rbegin(), seq.rend(), pred);
    if (iter == seq.rend()) {
        // find failed
        this->OutputDataStatus();
        throw Status(Status::ERROR, errorMsg);
    } else {
        // adjust
        seq.erase(iter.base(), seq.end());
    }
}
}  // namespace ns_ekalibr
