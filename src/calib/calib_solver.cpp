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

#include "calib/calib_solver.h"
#include "util/utils.h"
#include "util/utils_tpl.hpp"
#include "sensor/event_rosbag_loader.h"
#include "sensor/imu_rosbag_loader.h"
#include "sensor/frame_rosbag_loader.h"
#include "config/configor.h"
#include "pangolin/display/display.h"
#include "viewer/viewer.h"
#include "util/status.hpp"
#include "spdlog/spdlog.h"
#include "tiny-viewer/core/pose.hpp"
#include "rosbag/bag.h"
#include "filesystem"
#include "rosbag/view.h"
#include "calib/estimator.h"
#include "calib/ceres_callback.h"
#include <calib/calib_param_mgr.h>
#include <core/circle_grid.h>
#include "tiny-viewer/entity/utils.h"
#include "factor/visual_projection_factor.hpp"
#include "calib/spat_temp_priori.h"
#include <core/time_varying_ellipse.h>

namespace ns_ekalibr {
CalibSolver::CalibSolver(CalibParamManagerPtr parMgr)
    : _parMgr(std::move(parMgr)),
      _viewer(nullptr),
      _solveFinished(false),
      _priori(nullptr) {
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
    _ceresOption = Estimator::DefaultSolverOptions(-1, /*use all threads*/ true, false /*cuda*/);
    if (Configor::Preference::Visualization) {
        _ceresOption.callbacks.push_back(new CeresViewerCallBack(_viewer));
        _ceresOption.update_state_every_iteration = true;
    }
    // output spatiotemporal parameters after each iteration if needed
    if (IsOptionWith(OutputOption::ParamInEachIter, Configor::Preference::Outputs)) {
        _ceresOption.callbacks.push_back(new CeresDebugCallBack(_parMgr));
        _ceresOption.update_state_every_iteration = true;
    }

    // spatial and temporal priori
    if (std::filesystem::exists(Configor::Prior::SpatTempPrioriPath)) {
        _priori = SpatialTemporalPriori::Load(Configor::Prior::SpatTempPrioriPath);
        _priori->CheckValidityWithConfigor();
        spdlog::info("priori about spatial and temporal parameters are given: '{}'",
                     Configor::Prior::SpatTempPrioriPath);
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
    std::map<std::string, std::string> imuTopicTypeMap, evTopicTypeMap, frameTopicTypeMap;
    // add topics to vector
    for (const auto &[topic, info] : Configor::DataStream::EventTopics) {
        /**
         * The following logic flow is very unprofessional; we only use it for temporary processing
         * of frame data.
         */
        if (info.Type == FrameModel::ToString(FrameModelType::SENSOR_IMAGE) ||
            info.Type == FrameModel::ToString(FrameModelType::SENSOR_IMAGE_COMP)) {
            frameTopicTypeMap[topic] = info.Type;
            continue;
        }
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

    if (!evTopicTypeMap.empty()) {
        spdlog::info("loading event data from rosbag...");
        _evMes = LoadEventsFromROSBag(bag.get(), evTopicTypeMap, begTime, endTime);

        for (const auto &[topic, _] : Configor::DataStream::EventTopics) {
            if (auto iter = _evMes.find(topic); iter == _evMes.end() || iter->second.empty()) {
                throw Status(Status::CRITICAL,
                             "there is no data in topic '{}'! "
                             "check your configure file and rosbag!",
                             topic);
            }
        }
    }
    if (!imuTopicTypeMap.empty()) {
        spdlog::info("loading imu data from rosbag...");
        _imuMes = LoadIMUDataFromROSBag(bag.get(), imuTopicTypeMap, Configor::Prior::GravityNorm,
                                        begTime, endTime);

        for (const auto &[topic, _] : Configor::DataStream::IMUTopics) {
            if (auto iter = _imuMes.find(topic); iter == _imuMes.end() || iter->second.empty()) {
                throw Status(Status::CRITICAL,
                             "there is no data in topic '{}'! "
                             "check your configure file and rosbag!",
                             topic);
            }
        }
    }
    if (!frameTopicTypeMap.empty()) {
        spdlog::info("loading frame data from rosbag...");
        _frameMes = LoadFramesFromROSBag(bag.get(), frameTopicTypeMap, begTime, endTime);

        for (const auto &[topic, _] : frameTopicTypeMap) {
            if (auto iter = _frameMes.find(topic);
                iter == _frameMes.end() || iter->second.empty()) {
                throw Status(Status::CRITICAL,
                             "there is no data in topic '{}'! "
                             "check your configure file and rosbag!",
                             topic);
            }
        }
    }
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
    for (const auto &[topic, mes] : _frameMes) {
        sTimeList.push_back(mes.front()->GetTimestamp());
        eTimeList.push_back(mes.back()->GetTimestamp());
    }
    _dataRawTimestamp.first = *std::max_element(sTimeList.begin(), sTimeList.end());
    _dataRawTimestamp.second = *std::min_element(eTimeList.begin(), eTimeList.end());

    for (const auto &[topic, _] : _evMes) {
        // remove event data arrays that are before the start time stamp
        EraseSeqHeadData<EventArray>(
            _evMes.at(topic),
            [this](const EventArray::Ptr &ary) {
                return ary->GetTimestamp() > _dataRawTimestamp.first - 1E-9;
            },
            "the event data is invalid, there is no data intersection between sensors.");

        // remove event data arrays that are after the end time stamp
        EraseSeqTailData<EventArray>(
            _evMes.at(topic),
            [this](const EventArray::Ptr &ary) {
                return ary->GetTimestamp() < _dataRawTimestamp.second + 1E-9;
            },
            "the event data is invalid, there is no data intersection between sensors.");
    }
    for (const auto &[topic, _] : _imuMes) {
        // remove imu data arrays that are before the start time stamp
        EraseSeqHeadData<IMUFrame>(
            _imuMes.at(topic),
            [this](const IMUFrame::Ptr &ary) {
                return ary->GetTimestamp() > _dataRawTimestamp.first - 1E-9;
            },
            "the imu data is invalid, there is no data intersection between sensors.");

        // remove imu data arrays that are after the end time stamp
        EraseSeqTailData<IMUFrame>(
            _imuMes.at(topic),
            [this](const IMUFrame::Ptr &ary) {
                return ary->GetTimestamp() < _dataRawTimestamp.second + 1E-9;
            },
            "the imu data is invalid, there is no data intersection between sensors.");
    }
    for (const auto &[topic, _] : _frameMes) {
        // remove frame data arrays that are before the start time stamp
        EraseSeqHeadData<Frame>(
            _frameMes.at(topic),
            [this](const Frame::Ptr &ary) {
                return ary->GetTimestamp() > _dataRawTimestamp.first - 1E-9;
            },
            "the frame data is invalid, there is no data intersection between sensors.");

        // remove frame data arrays that are after the end time stamp
        EraseSeqTailData<Frame>(
            _frameMes.at(topic),
            [this](const Frame::Ptr &ary) {
                return ary->GetTimestamp() < _dataRawTimestamp.second + 1E-9;
            },
            "the frame data is invalid, there is no data intersection between sensors.");
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
    for (const auto &[frameTopic, mes] : _frameMes) {
        for (const auto &frame : mes) {
            // frame
            frame->SetTimestamp(frame->GetTimestamp() - _dataRawTimestamp.first);
        }
    }

    // compute imu frequency
    for (const auto &[topic, data] : _imuMes) {
        double dt = data.back()->GetTimestamp() - data.front()->GetTimestamp();
        _imuFrequency[topic] = static_cast<int>(static_cast<int>(data.size()) / dt);
        spdlog::info(
            "imu topic '{}', frequency: '{:04}' (Hz), acce weight: {:.3f}, gyro weight: {:.3f}",
            topic, _imuFrequency.at(topic),
            Configor::DataStream::IMUTopics.at(topic).AcceWeight(_imuFrequency.at(topic)),
            Configor::DataStream::IMUTopics.at(topic).GyroWeight(_imuFrequency.at(topic)));
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

CalibSolver::So3SplineType CalibSolver::CreateSo3Spline(double st,
                                                        double et,
                                                        double so3Dt,
                                                        bool printInfo) {
    auto so3SplineInfo = ns_ctraj::SplineInfo(Configor::Preference::SO3_SPLINE,
                                              ns_ctraj::SplineType::So3Spline, st, et, so3Dt);
    auto bundle = SplineBundleType::Create({so3SplineInfo});
    const auto &so3Spline = bundle->GetSo3Spline(Configor::Preference::SO3_SPLINE);
    if (printInfo) {
        spdlog::info(
            "create so3 spline: start time: '{:07.3f}:[{:07.3f}]', end time: "
            "'{:07.3f}:[{:07.3f}]', dt: '{:07.3f}'",
            so3Spline.MinTime(), st, so3Spline.MaxTime(), et, so3Dt);
    }

    return so3Spline;
}

CalibSolver::PosSplineType CalibSolver::CreatePosSpline(double st,
                                                        double et,
                                                        double posDt,
                                                        bool printInfo) {
    auto posSplineInfo = ns_ctraj::SplineInfo(Configor::Preference::SCALE_SPLINE,
                                              ns_ctraj::SplineType::RdSpline, st, et, posDt);
    auto bundle = SplineBundleType::Create({posSplineInfo});
    const auto &posSpline = bundle->GetRdSpline(Configor::Preference::SCALE_SPLINE);
    if (printInfo) {
        spdlog::info(
            "create pos spline: start time: '{:07.3f}:[{:07.3f}]', end time: "
            "'{:07.3f}:[{:07.3f}]', dt: '{:07.3f}'",
            posSpline.MinTime(), st, posSpline.MaxTime(), et, posDt);
    }
    return posSpline;
}

void CalibSolver::CreateSplineSegments(double dtSo3, double dtPos, bool printInfo) {
    _splineSegments.clear();
    _splineSegments.reserve(_validTimeSegments.size());
    for (auto &[st, et] : _validTimeSegments) {
        auto so3Spline = CreateSo3Spline(st, et, dtSo3, printInfo);
        auto posSpline = CreatePosSpline(st, et, dtPos, printInfo);
        _splineSegments.emplace_back(so3Spline, posSpline);

        st = std::min(so3Spline.MinTime(), posSpline.MinTime());
        et = std::max(so3Spline.MaxTime(), posSpline.MaxTime());
    }
}

std::optional<int> CalibSolver::IsTimeInValidSegment(double timeByBr) const {
    auto idx = IsTimeInSegment(timeByBr, _validTimeSegments);
    if (idx < 0 || idx >= static_cast<int>(_splineSegments.size())) {
        return {};
    } else {
        return idx;
    }
}

void CalibSolver::CreateVisualProjPairsSyncPointBased() {
    // create visual projection pairs
    for (const auto &[topic, patterns] : _extractedPatterns) {
        const auto &grid3d = patterns->GetGrid3d();
        const auto &grid2dVec = patterns->GetGrid2d();

        auto &pairs = _evSyncPointProjPairs[topic];
        pairs.clear();
        pairs.reserve(grid2dVec.size() * grid3d->points.size());

        for (const auto &grid2d : grid2dVec) {
            for (int i = 0; i < static_cast<int>(grid2d->centers.size()); ++i) {
                if (!grid2d->cenValidity.at(i)) {
                    continue;
                }
                const auto &center = grid2d->centers.at(i);
                const Eigen::Vector2d pixel(center.x, center.y);

                const auto &point3d = grid3d->points.at(i);
                const Eigen::Vector3d point(point3d.x, point3d.y, point3d.z);

                pairs.push_back(VisualProjectionPair::Create(grid2d->timestamp, point, pixel));
            }
        }
        spdlog::info("create 'VisualProjectionPair:SyncPointBased' count for camera '{}': {}",
                     topic, pairs.size());
    }
}

void CalibSolver::CreateVisualProjPairsAsyncPointBased(double dt) {
    for (const auto &[topic, rawEvsVecOfGrids] : _rawEventsOfExtractedPatterns) {
        auto &corrList = _evAsyncPointProjPairs[topic];
        corrList.clear();
        for (const auto &[grid2dIdx, rawEvsOfGrids] : rawEvsVecOfGrids) {
            // correspondences of each grid
            for (int i = 0; i < static_cast<int>(rawEvsOfGrids.size()); i++) {
                const auto &[tvCircle, evAry] = rawEvsOfGrids.at(i);
                if (tvCircle == nullptr) {
                    continue;
                }
                for (double t = tvCircle->st; t < tvCircle->et; t += dt) {
                    const auto &point3d = _grid3d->points.at(i);
                    const Eigen::Vector3d point(point3d.x, point3d.y, point3d.z);
                    corrList.push_back(VisualProjectionPair::Create(t, point, tvCircle->PosAt(t)));
                }
            }
        }
        spdlog::info("constructed 'VisualProjectionPair:AsyncPointBased' count for camera '{}': {}",
                     topic, corrList.size());
    }
}

double CalibSolver::BreakTimelineToSegments(double maxNeighborThd,
                                            double minLengthThd,
                                            const std::optional<std::string> &evTopic,
                                            bool optInfo) {
    /**
     * Due to the possibility that the checkerboard may be intermittently tracked (potentially due
     * to insufficient stimulation leading to an inadequate number of events, or the checkerboard
     * moving out of the field of view), it is necessary to identify the continuous segments for
     * subsequent calibration.
     */
    std::list<std::pair<double, double>> segBoundary;
    for (const auto &[topic, _] : Configor::DataStream::EventTopics) {
        if (evTopic != std::nullopt && topic != *evTopic) {
            continue;
        }
        const auto &TO_CjToBr = _parMgr->TEMPORAL.TO_CjToBr.at(topic);
        auto segments = this->ContinuousGridTrackingSegments(topic, maxNeighborThd /*neighbor*/,
                                                             minLengthThd /*len*/);
        for (const auto &segment : segments) {
            segBoundary.emplace_back(segment.front() + TO_CjToBr, segment.back() + TO_CjToBr);
        }
    }
    _validTimeSegments = ContinuousSegments(segBoundary, maxNeighborThd /*neighbor*/);
    std::stringstream ss;
    double sumTime = 0.0;
    for (const auto &[sTime, eTime] : _validTimeSegments) {
        sumTime += eTime - sTime;
        if (optInfo) {
            ss << fmt::format("({:.3f}, {:.3f}) ", sTime, eTime);
        }
    }
    if (optInfo) {
        const double percent =
            sumTime / (_dataAlignedTimestamp.second - _dataAlignedTimestamp.first);
        spdlog::info(
            "'maxNeighborThd': {}, 'minLengthThd': {}, finding valid time segment count: {}, "
            "percent "
            "in total data piece: {:.2f}%, details:\n{}",
            maxNeighborThd, minLengthThd, _validTimeSegments.size(), std::min(percent, 1.0) * 100.0,
            ss.str());
    }

    return sumTime;
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
        return static_cast<int>(std::distance(mergedSegmentsBoundary.begin(), it));
    } else {
        return -1;
    }
}
}  // namespace ns_ekalibr
