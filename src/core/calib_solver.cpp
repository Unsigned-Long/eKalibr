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
#include "util/utils_tpl.hpp"
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
#include "core/estimator.h"
#include "core/ceres_callback.h"
#include <core/circle_grid.h>

namespace ns_ekalibr {
CalibSolver::CalibSolver(CalibParamManagerPtr parMgr)
    : _parMgr(std::move(parMgr)),
      _viewer(nullptr),
      _solveFinished(false) {
    // viewer
    if (Configor::Preference::Visualization) {
        _viewer = Viewer::Create(Configor::Preference::MaxEntityCountInViewer);
    }

    // grid 3d
    const auto &pattern = Configor::Prior::CirclePattern;
    auto circlePattern = CirclePattern::FromString(pattern.Type);
    _grid3d = CircleGrid3D::Create(pattern.Rows, pattern.Cols,
                                   pattern.SpacingMeters /*unit: meters*/, circlePattern);

    // pass the 'CeresViewerCallBack' to ceres option so that update the viewer after every
    // iteration in ceres
    _ceresOption = Estimator::DefaultSolverOptions(-1, /*use all threads*/ true, false);
    if (Configor::Preference::Visualization) {
        _ceresOption.callbacks.push_back(new CeresViewerCallBack(_viewer, _grid3d));
        _ceresOption.update_state_every_iteration = true;
    }
    // output spatiotemporal parameters after each iteration if needed
    if (IsOptionWith(OutputOption::ParamInEachIter, Configor::Preference::Outputs)) {
        _ceresOption.callbacks.push_back(new CeresDebugCallBack(_parMgr));
        _ceresOption.update_state_every_iteration = true;
    }
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

CalibSolver::So3SplineType CalibSolver::CreateSo3Spline(double st, double et, double so3Dt) {
    auto so3SplineInfo = ns_ctraj::SplineInfo(Configor::Preference::SO3_SPLINE,
                                              ns_ctraj::SplineType::So3Spline, st, et, so3Dt);
    auto bundle = SplineBundleType::Create({so3SplineInfo});
    spdlog::info("create so3 spline: start time: '{:07.3f}', end time: '{:07.3f}', dt : '{:07.3f}'",
                 st, et, so3Dt);
    return bundle->GetSo3Spline(Configor::Preference::SO3_SPLINE);
}

CalibSolver::PosSplineType CalibSolver::CreatePosSpline(double st, double et, double posDt) {
    auto posSplineInfo = ns_ctraj::SplineInfo(Configor::Preference::SCALE_SPLINE,
                                              ns_ctraj::SplineType::RdSpline, st, et, posDt);
    auto bundle = SplineBundleType::Create({posSplineInfo});
    spdlog::info("create pos spline: start time: '{:07.3f}', end time: '{:07.3f}', dt : '{:07.3f}'",
                 st, et, posDt);
    return bundle->GetRdSpline(Configor::Preference::SCALE_SPLINE);
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

int CalibSolver::IsTimeInValidSegment(double timeByBr) const {
    return IsTimeInSegment(timeByBr, _validTimeSegments);
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

std::list<std::list<double>> CalibSolver::ContinuousGridTrackingSegments(
    const std::string &camTopic,
    const double neighborTimeDistThd,
    const double minSegmentThd) const {
    const auto &grid2dVec = _extractedPatterns.at(camTopic)->GetGrid2d();

    std::list<std::list<double>> segments;
    std::list<double> curSegment{grid2dVec.front()->timestamp};

    for (auto jIter = std::next(grid2dVec.cbegin()); jIter != grid2dVec.cend(); jIter++) {
        const auto &jGrid = *jIter;
        const auto &iGrid = *std::prev(jIter);
        if (jGrid->timestamp - iGrid->timestamp < neighborTimeDistThd) {
            curSegment.push_back(jGrid->timestamp);
        } else {
            // the last segment is a valid one
            if (curSegment.size() >= 2 && curSegment.back() - curSegment.front() > minSegmentThd) {
                segments.push_back(curSegment);
            }
            // create a new segment
            curSegment = {jGrid->timestamp};
        }
    }

    // the last one
    if (curSegment.size() >= 2 && curSegment.back() - curSegment.front() > minSegmentThd) {
        segments.push_back(curSegment);
    }
    return segments;
}

std::vector<std::pair<double, double>> CalibSolver::ContinuousSegments(
    const std::list<std::pair<double, double>> &segBoundary, const double neighborTimeDistThd) {
    if (segBoundary.empty()) return {};

    std::vector sortedSegments(segBoundary.begin(), segBoundary.end());
    std::sort(sortedSegments.begin(), sortedSegments.end());

    std::list<std::pair<double, double>> mergedSegments;

    std::pair<double, double> currentSegment = sortedSegments[0];

    for (size_t i = 1; i < sortedSegments.size(); ++i) {
        const auto &nextSegment = sortedSegments[i];

        if (nextSegment.first < currentSegment.second + neighborTimeDistThd) {
            currentSegment.second = std::max(currentSegment.second, nextSegment.second);
        } else {
            mergedSegments.push_back(currentSegment);
            currentSegment = nextSegment;
        }
    }

    mergedSegments.push_back(currentSegment);

    return {mergedSegments.cbegin(), mergedSegments.cend()};
}

int CalibSolver::IsTimeInSegment(
    double t, const std::vector<std::pair<double, double>> &mergedSegmentsBoundary) {
    // Perform binary search to find the segment that might contain time point t
    auto it = std::lower_bound(
        mergedSegmentsBoundary.begin(), mergedSegmentsBoundary.end(), t,
        [](const std::pair<double, double> &seg, double time) { return seg.first < time; });

    // If no segment found, return false
    if (it == mergedSegmentsBoundary.begin()) {
        return -1;
    }

    --it;  // Move back to the segment that may contain the time point t

    // Check if time point t is within the current segment
    if (t >= it->first && t <= it->second) {
        return std::distance(mergedSegmentsBoundary.begin(), it);
    } else {
        return -1;
    }
}
}  // namespace ns_ekalibr
