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
#include "config/configor.h"
#include "pangolin/display/display.h"
#include "viewer/viewer.h"
#include "util/status.hpp"
#include "spdlog/spdlog.h"
#include "tiny-viewer/core/pose.hpp"

namespace ns_ekalibr {
CalibSolver::CalibSolver()
    : _viewer(Configor::Preference::Visualization
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

CalibSolver::Ptr CalibSolver::Create() { return std::make_shared<CalibSolver>(); }

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

void CalibSolver::LoadEventData() {
    /**
     * load raw event data from rosbag
     */
    std::map<std::string, std::string> evTopicTypeMap;
    for (const auto &[topic, info] : Configor::DataStream::EventTopics) {
        evTopicTypeMap[topic] = info.Type;
    }

    _evMes = LoadEventsFromROSBag(Configor::DataStream::BagPath,  // bag path
                                  evTopicTypeMap,                 // map contains [topic,type] pairs
                                  Configor::DataStream::BeginTime,  // begin time
                                  Configor::DataStream::Duration);  // duration

    /**
     * select event data range
     */
    _evDataRawTimestamp.first = std::max_element(_evMes.begin(), _evMes.end(),
                                                 [](const auto &p1, const auto &p2) {
                                                     return p1.second.front()->GetTimestamp() <
                                                            p2.second.front()->GetTimestamp();
                                                 })
                                    ->second.front()
                                    ->GetTimestamp();

    _evDataRawTimestamp.second = std::min_element(_evMes.begin(), _evMes.end(),
                                                  [](const auto &p1, const auto &p2) {
                                                      return p1.second.back()->GetTimestamp() <
                                                             p2.second.back()->GetTimestamp();
                                                  })
                                     ->second.back()
                                     ->GetTimestamp();

    for (const auto &[topic, _] : Configor::DataStream::EventTopics) {
        // remove event data arrays that are before the start time stamp
        EraseSeqHeadData(
            _evMes.at(topic),
            [this](const EventArray::Ptr &ary) {
                return ary->GetTimestamp() > _evDataRawTimestamp.first - 1E-3;
            },
            "the event data is invalid, there is no data intersection between event cameras.");

        // remove event data arrays that are after the end time stamp
        EraseSeqTailData(
            _evMes.at(topic),
            [this](const EventArray::Ptr &ary) {
                return ary->GetTimestamp() < _evDataRawTimestamp.second + 1E-3;
            },
            "the event data is invalid, there is no data intersection between event cameras.");
    }

    /**
     * align mes data
     */
    // all time stamps minus  '_rawStartTimestamp'
    _evDataAlignedTimestamp.first = 0.0;
    _evDataAlignedTimestamp.second = _evDataRawTimestamp.second - _evDataRawTimestamp.first;
    for (const auto &[eventTopic, mes] : _evMes) {
        for (const auto &array : mes) {
            // array
            array->SetTimestamp(array->GetTimestamp() - _evDataRawTimestamp.first);
            // targets
            for (auto &tar : array->GetEvents()) {
                tar->SetTimestamp(tar->GetTimestamp() - _evDataRawTimestamp.first);
            }
        }
    }

    this->OutputDataStatus();
}

void CalibSolver::OutputDataStatus() const {
    spdlog::info("calibration data info:");
    spdlog::info("raw start time: '{:+010.5f}' (s), raw end time: '{:+010.5f}' (s)",
                 _evDataRawTimestamp.first, _evDataRawTimestamp.second);
    spdlog::info("aligned start time: '{:+010.5f}' (s), aligned end time: '{:+010.5f}' (s)",
                 _evDataAlignedTimestamp.first, _evDataAlignedTimestamp.second);
    for (const auto &[topic, mes] : _evMes) {
        spdlog::info(
            "Event topic: '{}', data size: '{:06}', time span: from '{:+010.5f}' to '{:+010.5f}' "
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
}  // namespace ns_ekalibr
