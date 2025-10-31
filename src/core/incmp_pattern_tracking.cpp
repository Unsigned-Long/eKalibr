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

#include "core/incmp_pattern_tracking.h"
#include "spdlog/spdlog.h"
#include "core/circle_grid.h"
#include "sensor/event.h"
#include <config/configor.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "util/utils_tpl.hpp"
#include "core/circle_extractor.h"
#include <core/time_varying_ellipse.h>
#include <opencv2/flann/flann.hpp>
#include <util/status.hpp>
#include <calib/calib_solver_io.h>

namespace ns_ekalibr {

InCmpPatternTracker::InCmpPatternTracker(int cen_num_thd_for_each_in_cmp_pattern,
                                         double dist_thd_to_track_cen,
                                         bool visualization,
                                         bool save)
    : cenNumThdForEachInCmpPattern(cen_num_thd_for_each_in_cmp_pattern),
      distThdToTrackCen(dist_thd_to_track_cen),
      visualization(visualization),
      save(save) {}

InCmpPatternTracker::Ptr InCmpPatternTracker::Create(int cen_num_thd_for_each_in_cmp_pattern,
                                                     double dist_thd_to_track_cen,
                                                     bool visualization,
                                                     bool save) {
    return std::make_shared<InCmpPatternTracker>(cen_num_thd_for_each_in_cmp_pattern,
                                                 dist_thd_to_track_cen, visualization, save);
}

std::set<int> InCmpPatternTracker::Tracking(
    const std::string& topic,
    const CircleGridPatternPtr& pattern,
    std::map<int, ExtractedCirclesVec>& tvCirclesWithRawEvs) {
    spdlog::info(
        "InCmpPatternTracker::Tracking: 'cenNumThdForEachInCmpPattern': '{}', 'distThdToTrackCen': "
        "'{:.5f}' (pixels)",
        cenNumThdForEachInCmpPattern, distThdToTrackCen);

    const auto& grid2ds = std::vector(pattern->GetGrid2d().cbegin(), pattern->GetGrid2d().cend());
    std::map<int, CircleGrid2D::Ptr> circleGrid2ds;

    bool isAscendingOrder = true;
    std::set<int> processedIncmpGridIds, trackedGridIdx;
    for (const auto& grid2d : grid2ds) {
        if (grid2d->isComplete) {
            trackedGridIdx.insert(grid2d->id);
        }
    }
    int trackingLoopCount = 0;
    while (true) {
        int newIncmpGridTrackedCount = 0;
        for (int i = 1; i < static_cast<int>(grid2ds.size()) - 3; i++) {
            auto grid1 = grid2ds.at(i), grid2 = grid2ds.at(i + 1), grid3 = grid2ds.at(i + 2);
            if (trackedGridIdx.count(grid1->id) == 0 ||  // 'grid1' is not tracked
                trackedGridIdx.count(grid2->id) == 0 ||  // 'grid2' is not tracked
                trackedGridIdx.count(grid3->id) == 0) {  // 'grid3' is not tracked
                continue;
            }
            const int headIdx = i - 1, tailIdx = i + 3;
            const auto& gridToTrack = isAscendingOrder ? grid2ds.at(tailIdx) : grid2ds.at(headIdx);
            if (trackedGridIdx.count(gridToTrack->id) != 0) {
                // this grid has been tracked
                continue;
            }
            if (processedIncmpGridIds.count(gridToTrack->id) != 0) {
                // this grid has been processed
                continue;
            }
            if (static_cast<int>(gridToTrack->centers.size()) < cenNumThdForEachInCmpPattern) {
                // no need to processed
                processedIncmpGridIds.insert(gridToTrack->id);
                continue;
            }
            if (!isAscendingOrder) {
                auto gridTmp = grid1;
                grid1 = grid3;
                grid3 = gridTmp;
            }
            // spdlog::info("'isAscendingOrder': {}, try to track 2d grid: [{}, {}, {}]->[{}]",
            //              static_cast<int>(isAscendingOrder), grid1->id, grid2->id, grid3->id,
            //              gridToTrack->id);

            auto incmpGridPatternIdx = TryToTrackInCmpGridPattern(topic, grid1, grid2, grid3,
                                                                  gridToTrack, tvCirclesWithRawEvs);
            // good count (tracked centers)
            const std::size_t trackedCount =
                std::count_if(incmpGridPatternIdx.cbegin(), incmpGridPatternIdx.cend(),
                              [](int value) { return value >= 0; });

            // tracked success
            if (static_cast<int>(trackedCount) >= cenNumThdForEachInCmpPattern) {
                // store
                auto oldCenters = gridToTrack->centers;
                gridToTrack->centers.resize(incmpGridPatternIdx.size());
                gridToTrack->cenValidity.resize(incmpGridPatternIdx.size());

                const auto& rawEvsOfGridToTrack = tvCirclesWithRawEvs.at(gridToTrack->id);
                ExtractedCirclesVec newRawEvsOfGridToTrack(incmpGridPatternIdx.size());

                for (std::size_t j = 0; j < incmpGridPatternIdx.size(); j++) {
                    auto cenIdxInOldCenters = incmpGridPatternIdx.at(j);
                    if (cenIdxInOldCenters >= 0) {
                        // tracked
                        gridToTrack->centers.at(j) = oldCenters.at(cenIdxInOldCenters);
                        gridToTrack->cenValidity.at(j) = 1;
                        newRawEvsOfGridToTrack.at(j) = rawEvsOfGridToTrack.at(cenIdxInOldCenters);
                    } else {
                        // not tracked
                        gridToTrack->centers.at(j) = cv::Point2f(-1.0f, -1.0f);
                        gridToTrack->cenValidity.at(j) = 0;
                        newRawEvsOfGridToTrack.at(j) = {};
                    }
                }
                tvCirclesWithRawEvs.at(gridToTrack->id) = newRawEvsOfGridToTrack;

                trackedGridIdx.insert(gridToTrack->id);

                newIncmpGridTrackedCount += 1;
                // break continuous tracking
                i += 1;

                // spdlog::info("grid '{}' is tracked, with valid '{}'>'{}' centers.",
                // gridToTrack->id, trackedCount, cenNumThdForEachInCmpPattern);
            } else {
                // spdlog::warn("grid '{}' is not tracked: valid '{}'<'{}' centers.",
                // gridToTrack->id, trackedCount, cenNumThdForEachInCmpPattern);
            }

            processedIncmpGridIds.insert(gridToTrack->id);
        }
        ++trackingLoopCount;
        isAscendingOrder = !isAscendingOrder;
        // spdlog::info("'newIncmpGridTrackedCount' in this tracking loop: {}",
        //              newIncmpGridTrackedCount);
        if (newIncmpGridTrackedCount == 0 && trackingLoopCount >= 2) {
            break;
        }
    }

    std::set<int> tackedIncmpGridIdx;
    for (const auto& grid2d : grid2ds) {
        if (!grid2d->isComplete && trackedGridIdx.count(grid2d->id) != 0) {
            tackedIncmpGridIdx.insert(grid2d->id);
        }
    }

    return tackedIncmpGridIdx;
}

std::vector<int> InCmpPatternTracker::TryToTrackInCmpGridPattern(
    const std::string& topic,
    const CircleGrid2DPtr& grid1,
    const CircleGrid2DPtr& grid2,
    const CircleGrid2DPtr& grid3,
    const CircleGrid2DPtr& gridToTrack,
    const std::map<int, ExtractedCirclesVec>& tvCirclesWithRawEvs) const {
    std::array<cv::Mat, 4> mats;
    if (visualization) {
        mats[0] = EventCircleExtractor::CreateSAEWithTVEllipses(
            topic, tvCirclesWithRawEvs.at(grid1->id), grid1);
        mats[1] = EventCircleExtractor::CreateSAEWithTVEllipses(
            topic, tvCirclesWithRawEvs.at(grid2->id), grid2);
        mats[2] = EventCircleExtractor::CreateSAEWithTVEllipses(
            topic, tvCirclesWithRawEvs.at(grid3->id), grid3);
        mats[3] = EventCircleExtractor::CreateSAEWithTVEllipses(
            topic, tvCirclesWithRawEvs.at(gridToTrack->id), gridToTrack);
    }
    auto size = static_cast<int>(grid1->centers.size());
    assert(size == grid2->centers.size());
    assert(size == grid3->centers.size());

    cv::Mat data(static_cast<int>(gridToTrack->centers.size()), 2, CV_32F);
    for (int i = 0; i < static_cast<int>(gridToTrack->centers.size()); ++i) {
        data.at<float>(i, 0) = gridToTrack->centers[i].x;
        data.at<float>(i, 1) = gridToTrack->centers[i].y;
    }
    cv::flann::Index kdtree(data, cv::flann::KDTreeIndexParams(1));

    // the nearest point index, and corresponding distance (pixels)
    std::vector<std::optional<std::pair<std::size_t, double>>> nearestPts(size, std::nullopt);

    for (int i = 0; i < size; i++) {
        if (!grid1->cenValidity[i] || !grid2->cenValidity[i] || !grid3->cenValidity[i]) {
            continue;
        }
        const auto &c1 = grid1->centers[i], c2 = grid2->centers[i], c3 = grid3->centers[i];

        std::array<double, 3> tData{grid1->timestamp, grid2->timestamp, grid3->timestamp};
        std::array<double, 3> xData{c1.x, c2.x, c3.x};
        std::array<double, 3> yData{c1.y, c2.y, c3.y};
        double xPred = LagrangePolynomial<double, 3>(gridToTrack->timestamp, tData, xData);
        double yPred = LagrangePolynomial<double, 3>(gridToTrack->timestamp, tData, yData);
        auto pPred = cv::Point2f(static_cast<float>(xPred), static_cast<float>(yPred));

        cv::Mat query = (cv::Mat_<float>(1, 2) << pPred.x, pPred.y);
        std::vector<int> indices(1);
        std::vector<float> dists(1);
        kdtree.knnSearch(query, indices, dists, 1);
        float distance = std::sqrt(dists[0]);
        if (distance < distThdToTrackCen) {
            nearestPts.at(i) = {indices[0], distance};
        }

        if (visualization) {
            // const double dt = std::abs(gridToTrack->timestamp - grid3->timestamp);
            const double dt = 0.0;
            if (grid1->timestamp > grid3->timestamp) {
                DrawTrace(mats[3], grid3->timestamp, grid2->timestamp, grid1->timestamp, dt, c3, c2,
                          c1, 1.0);
            } else {
                DrawTrace(mats[3], grid1->timestamp, grid2->timestamp, grid3->timestamp, dt, c1, c2,
                          c3, 1.0);
            }
            DrawKeypointOnCVMat(mats[3], pPred, false, cv::Scalar(255, 255, 255));
        }
    }

    // Map to store the minimum distance and corresponding index position for each unique index
    std::unordered_map<std::size_t, std::pair<double, std::size_t>> indexToMinDistance;

    // First pass: Identify the minimum distance for each index
    for (std::size_t i = 0; i < nearestPts.size(); ++i) {
        if (nearestPts[i].has_value()) {
            auto [index, distance] = nearestPts[i].value();
            // Update if the index is new or the current distance is smaller
            if (indexToMinDistance.find(index) == indexToMinDistance.end() ||
                distance < indexToMinDistance[index].first) {
                indexToMinDistance[index] = {distance, i};
            }
        }
    }

    // Second pass: Set all non-minimum distance entries to std::nullopt
    for (std::size_t i = 0; i < nearestPts.size(); ++i) {
        if (nearestPts[i].has_value()) {
            auto [index, distance] = nearestPts[i].value();
            // Check if the current entry is not the one with the minimum distance
            if (indexToMinDistance[index].second != i) {
                nearestPts[i] = std::nullopt;  // Set to nullopt if not the closest
            }
        }
    }

    std::vector<int> incmpGridPatternIdx(size, -1);
    for (int i = 0; i < size; i++) {
        if (nearestPts[i].has_value()) {
            incmpGridPatternIdx.at(i) = static_cast<int>(nearestPts.at(i)->first);
        }
    }

    if (visualization) {
        for (int i = 0; i < size; i++) {
            if (incmpGridPatternIdx.at(i) < 0) {
                continue;
            }
            cv::Point2f pTracked = gridToTrack->centers.at(incmpGridPatternIdx.at(i));
            cv::Point2f p3 = grid3->centers.at(i);
            DrawLineOnCVMat(mats[3], pTracked, p3, cv::Scalar(255, 0, 0));
            // spdlog::info("nearest distance: '{:.3f}' (pixels)", nearestPts.at(i)->second);
        }
        cv::Mat mTmp1, mTmp2;
        cv::hconcat(mats[0], mats[1], mTmp1);
        cv::hconcat(mats[2], mats[3], mTmp2);
        cv::hconcat(mTmp1, mTmp2, mTmp1);
        cv::imshow("Tracking Incomplete 2D Grids", mTmp1);
        if (save) {
            CalibSolverIO::SaveIncmpGridTracking(topic, mTmp1, gridToTrack->id);
        }
        cv::waitKey(1);
    }
    return incmpGridPatternIdx;
}

void InCmpPatternTracker::DrawTrace(cv::Mat& img,
                                    double t1,
                                    double t2,
                                    double t3,
                                    double timePadding,
                                    const cv::Point2f& p1,
                                    const cv::Point2f& p2,
                                    const cv::Point2f& p3,
                                    const int pixelDist) {
    assert(t1 < t2);
    assert(t2 < t3);
    const double sTime = t1 - timePadding;
    const double eTime = t3 + timePadding;

    std::array<double, 3> tData{t1, t2, t3};
    std::array<double, 3> xData{p1.x, p2.x, p3.x};
    std::array<double, 3> yData{p1.y, p2.y, p3.y};
    auto vx = LagrangePolynomialTripleMidFOD(tData, xData);
    auto vy = LagrangePolynomialTripleMidFOD(tData, yData);
    const double deltaTime = pixelDist / std::sqrt(vx * vx + vy * vy);

    double xLast = LagrangePolynomial<double, 3>(sTime, tData, xData);
    double yLast = LagrangePolynomial<double, 3>(sTime, tData, yData);
    for (double t = sTime + deltaTime; t < eTime;) {
        double x = LagrangePolynomial<double, 3>(t, tData, xData);
        double y = LagrangePolynomial<double, 3>(t, tData, yData);
        DrawLineOnCVMat(img, cv::Point2d(xLast, yLast), cv::Point2d(x, y), cv::Scalar(0, 255, 0));
        t += deltaTime;
        xLast = x;
        yLast = y;
    }
    for (int i = 0; i < 3; ++i) {
        DrawKeypointOnCVMat(img, cv::Point2d(xData[i], yData[i]), false, cv::Scalar(0, 255, 0));
    }
}
}  // namespace ns_ekalibr