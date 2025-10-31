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
#include "sensor/event.h"
#include "opencv4/opencv2/highgui.hpp"
#include "core/circle_extractor.h"
#include "core/circle_grid.h"
#include "filesystem"
#include "viewer/viewer.h"
#include "core/sae.h"
#include "spdlog/spdlog.h"
#include "util/tqdm.h"
#include "core/visual_distortion.h"
#include "calib/calib_param_mgr.h"
#include <core/time_varying_ellipse.h>
#include <tiny-viewer/object/landmark.h>
#include <util/status.hpp>
#include <veta/camera/pinhole.h>
#include "calib/calib_solver_io.h"
#include "core/incmp_pattern_tracking.h"

namespace ns_ekalibr {
void CalibSolver::GridPatternTracking(bool tryLoadAndSaveRes) {
    const double decay = Configor::Prior::DecayTimeOfActiveEvents;
    const auto &pattern = Configor::Prior::CirclePattern;
    auto circlePattern = CirclePattern::FromString(pattern.Type);
    auto patternSize = cv::Size(pattern.Cols, pattern.Rows);

    const auto &nfConfig = Configor::Prior::NormFlowEstimator;
    // nfConfig.WinSizeInPlaneFit >= 1
    const auto neighborNormFlowDist = nfConfig.WinSizeInPlaneFit * 2 - 1;

    _grid3d = CircleGrid3D::Create(pattern.Rows, pattern.Cols,
                                   pattern.SpacingMeters /*unit: meters*/, circlePattern);

    std::map<std::string, bool> patternLoadFromFile;
    const ns_viewer::Posef initViewCamPose(Eigen::Matrix3f::Identity(), {0.0f, 0.0f, -4.0f});
    if (Configor::Preference::Visualization) {
        _viewer->ClearViewer();
        _viewer->ResetViewerCamera();
    }
    /**
     * tracking complete grid patterns
     */
    std::unordered_map<std::string, std::unordered_map<int, cv::Mat>>
        SAEMapTrackedCirclesGridBackUp;

    constexpr bool VisualizationSaveForDebug = false;

    for (const auto &[topic, eventMes] : _evMes) {
        if (tryLoadAndSaveRes) {
            // try load
            auto [gridPatternPath, rawEvsPath] =
                CalibSolverIO::GetDiskPathOfExtractedGridPatterns(topic);
            if (std::filesystem::exists(gridPatternPath) && std::filesystem::exists(rawEvsPath)) {
                // try load '_rawEventsOfExtractedPatterns'
                spdlog::info(
                    "try to load existing raw events of extracted circles of grid patterns for "
                    "camera '{}' from '{}'...",
                    topic, rawEvsPath);
                auto rawEvsOfPattern = CalibSolverIO::LoadRawEventsOfExtractedPatterns(
                    rawEvsPath, _dataRawTimestamp.first, CerealArchiveType::Enum::BINARY);

                // try load '_extractedPatterns'
                spdlog::info(
                    "try to load existing extracted circles grid patterns for camera '{}' from "
                    "'{}'...",
                    topic, gridPatternPath);
                auto curPattern = CircleGridPattern::Load(gridPatternPath, _dataRawTimestamp.first,
                                                          Configor::Preference::OutputDataFormat);

                if (!rawEvsOfPattern.empty() && curPattern != nullptr) {
                    // select in time-range pattern
                    auto idsOfRemoved = curPattern->RemoveGrid2DOutOfTimeRange(
                        _dataRawTimestamp.first, _dataRawTimestamp.second);

                    // filter '_rawEventsOfExtractedPatterns'
                    for (int grid2dId : idsOfRemoved) {
                        rawEvsOfPattern.erase(grid2dId);
                    }

                    // assign
                    _extractedPatterns[topic] = curPattern;
                    _rawEventsOfExtractedPatterns[topic] = rawEvsOfPattern;
                    patternLoadFromFile[topic] = true;
                    spdlog::info(
                        "load extracted circles grid patterns and raw events for camera '{}' "
                        "success! details:\n{}\nif you want to use a different configuration for "
                        "circle grid extraction, please delete existing file at '{}' and '{}'",
                        topic, curPattern->InfoString(), rawEvsPath, gridPatternPath);
                    continue;
                }
            }

            // failed
            spdlog::info(
                "try to load extracted circles grid patterns failed! perform norm-flow-based "
                "circle grid identification for camera '{}'",
                topic);
        } else {
            spdlog::info("perform norm-flow-based circle grid identification for camera '{}'",
                         topic);
        }

        const auto &config = Configor::DataStream::EventTopics.at(topic);
        auto sae = ActiveEventSurface::Create(config.Width, config.Height, 0.01);

        double lastUpdateTime = eventMes.front()->GetTimestamp();
        auto bar = std::make_shared<tqdm>();

        auto curPattern = CircleGridPattern::Create(_grid3d, _dataRawTimestamp.first);
        std::map<int, ExtractedCirclesVec> rawEvsOfPattern;
        int grid2dIdx = 0;

        for (int i = 0; i < static_cast<int>(eventMes.size()); i++) {
            bar->progress(i, static_cast<int>(eventMes.size()));

            for (const auto &event : eventMes.at(i)->GetEvents()) {
                /**
                 * create sae (surface of active events)
                 */
                sae->GrabEvent(event, true);
                const auto timeLatest = sae->GetTimeLatest();

                if (timeLatest - eventMes.front()->GetTimestamp() < 0.05 ||
                    timeLatest - lastUpdateTime < decay) {
                    continue;
                } else {
                    lastUpdateTime = timeLatest;
                }

                // auto dts = sae->DecayTimeSurface(true, 0, decay);
                // cv::imshow("Decay Surface Of Active Events", dts);

                /**
                 * estimate norm flows using created sae
                 */
                auto nfPack = EventNormFlow(sae).ExtractNormFlows(
                    decay,                          // decay seconds for time surface
                    nfConfig.WinSizeInPlaneFit,     // window size to fit local planes
                    neighborNormFlowDist,           // distance between neighbor norm flows
                    nfConfig.RansacInlierRatioThd,  // the ratio, for ransac and in-range candidates
                    nfConfig.EventToPlaneTimeDistThd,  // the point to plane threshold in temporal
                                                       // domain, unit (s)
                    nfConfig.RansacMaxIterations);     // ransac iteration count

                /**
                 * extract circle grid pattern
                 */
                auto circleExtractor = EventCircleExtractor::Create(
                    true, /* create sea mats */
                    Configor::Prior::CircleExtractor.ValidClusterAreaThd,
                    Configor::Prior::CircleExtractor.CircleClusterPairDirThd,
                    Configor::Prior::CircleExtractor.PointToCircleDistThd);

                auto [isCmp, centers, rawEvs] = circleExtractor->ExtractCirclesGrid(
                    nfPack, patternSize, circlePattern, true, _viewer);

                auto grid2d = CircleGrid2D::Create(
                    grid2dIdx, nfPack->timestamp, centers,
                    std::vector<std::uint8_t>(centers.size(), isCmp ? 1 : 0), isCmp);
                curPattern->AddGrid2d(grid2d);
                /**
                 * distortion in 'res->second' is not considered, i.e., they are raw ones from
                 * input events
                 */
                rawEvsOfPattern.insert({grid2dIdx, rawEvs});

                CalibSolverIO::SaveSAEMaps(topic, circleExtractor, grid2dIdx, nfPack->tsImg,
                                           sae->GetAccumulatedEventImg(true));

                /**
                 * for incomplete grid patterns, we will try to track it. we save the time
                 * 'SAEMapExtractCirclesGrid' for subsequent drawing:
                 *  (1) for complete grid pattern, 'SAEMapExtractCirclesGrid' has been drawn using
                 * opencv api: 'cv::drawChessboardCorners'.
                 *  (2) for incomplete grid pattern, 'SAEMapExtractCirclesGrid' is the clean, just a
                 * clean time-surface map
                 */
                SAEMapTrackedCirclesGridBackUp[topic][grid2dIdx] =
                    circleExtractor->SAEMapExtractCirclesGrid();

                if (Configor::Preference::Visualization) {
                    // to save more information, set the parameter as 'true'
                    nfPack->Visualization(decay, VisualizationSaveForDebug, grid2dIdx);
                    circleExtractor->Visualization(VisualizationSaveForDebug, grid2dIdx, topic);

                    auto ptScale = Configor::Preference::EventViewerSpatialTemporalScale;
                    auto t = -timeLatest * ptScale.second;
                    ns_viewer::Posef curViewCamPose = initViewCamPose;
                    curViewCamPose.translation(0) = float(config.Width * 0.5 * ptScale.first);
                    curViewCamPose.translation(1) = float(config.Height * 0.5 * ptScale.first);
                    curViewCamPose.translation(2) = float(t + initViewCamPose.translation(2));
                    _viewer->SetCamView(curViewCamPose);
                    // CalibSolverIO::SaveTinyViewerOnRender(topic, grid2dIdx);
                    cv::waitKey(1);
                }
                ++grid2dIdx;
            }
        }

        bar->finish();
        if (Configor::Preference::Visualization) {
            _viewer->ClearViewer();
            _viewer->ResetViewerCamera();
            cv::destroyAllWindows();
        }

        spdlog::info("extracted circle grid pattern count for camera '{}' finished! details:\n{}",
                     topic, curPattern->InfoString());

        _extractedPatterns[topic] = curPattern;
        _rawEventsOfExtractedPatterns[topic] = rawEvsOfPattern;
        patternLoadFromFile[topic] = false;
    }

    /**
     * tracking incomplete grid patterns
     */
    for (const auto &[topic, curPattern] : _extractedPatterns) {
        if (patternLoadFromFile.at(topic)) {
            continue;
        }
        spdlog::info("tracking incomplete grid patterns for camera '{}'...", topic);

        // compute the average distance of first two centers of each complete grid pattern
        double avgDist = 0.0;
        int count = 0;
        for (const auto &grid2d : curPattern->GetGrid2d()) {
            if (!grid2d->isComplete) {
                continue;
            }
            avgDist += cv::norm(grid2d->centers.at(0) - grid2d->centers.at(1));
            count++;
        }
        avgDist /= count;

        auto &rawEvsOfPattern = _rawEventsOfExtractedPatterns.at(topic);
        auto gridSize = static_cast<int>(curPattern->GetGrid3d()->points.size());
        auto tracker = InCmpPatternTracker::Create(
            // for those tracked incmp grids, their center num should be larger than this value
            std::max(static_cast<int>(gridSize * 0.4), 4),
            avgDist * 0.15,  // only the distance smaller than this val would be considered tracked
            Configor::Preference::Visualization,
            VisualizationSaveForDebug  // only for debug
        );
        auto trackedIncmpGridIds = tracker->Tracking(topic, curPattern, rawEvsOfPattern);

        auto &grid2ds = curPattern->GetGrid2d();
        int compNum = 0, inCompTrackedNum = 0, inCompNotTrackedNum = 0;
        spdlog::info("refine tracked incomplete grids, time-varying circles to ellipses...");
        auto bar = std::make_shared<tqdm>();
        const int total = static_cast<int>(grid2ds.size());
        for (auto iter = grid2ds.cbegin(); iter != grid2ds.cend();) {
            bar->progress(compNum + inCompTrackedNum + inCompNotTrackedNum, total);
            const auto &grid2d = *iter;
            if (grid2d->isComplete) {
                ++compNum;
                ++iter;
                continue;
            }
            if (trackedIncmpGridIds.count(grid2d->id) == 0) {
                // incomplete but not tracked, erase
                rawEvsOfPattern.erase(grid2d->id);
                iter = grid2ds.erase(iter);
                ++inCompNotTrackedNum;
            } else {
                // incomplete and tracked, refine circle to ellipse
                auto &verifiedCircles = rawEvsOfPattern.at(grid2d->id);

#pragma omp parallel for
                for (int i = 0; i < static_cast<int>(grid2d->centers.size()); ++i) {
                    if (!grid2d->cenValidity[i]) {
                        continue;
                    }
                    // refine time-varying circle to time-varying ellipse
                    verifiedCircles.at(i).first =
                        EventCircleExtractor::RefineTimeVaryingCircleToEllipse(
                            verifiedCircles.at(i).first,   // initialized time-varying circle
                            verifiedCircles.at(i).second,  // events
                            Configor::Prior::CircleExtractor.PointToCircleDistThd);
                    // update the center
                    auto c = verifiedCircles.at(i).first->EllipseAt(grid2d->timestamp);
                    grid2d->centers.at(i) = cv::Vec2f(c->c(0), c->c(1));
                }
                if (!patternLoadFromFile.at(topic)) {
                    // for each tracked incomplete grid pattern, we draw the track results
                    grid2d->DrawCenters(SAEMapTrackedCirclesGridBackUp[topic][grid2d->id],
                                        patternSize);
                    if (Configor::Preference::Visualization) {
                        cv::imshow("Tracked Incomplete Grid Pattern",
                                   SAEMapTrackedCirclesGridBackUp[topic][grid2d->id]);
                        cv::waitKey(1);
                    }
                }
                ++inCompTrackedNum;
                ++iter;
            }
        }
        bar->finish();
        spdlog::info(
            "complete grids: {}, incomplete but tracked grids: {}, incomplete and not tracked "
            "grids: {}",
            compNum, inCompTrackedNum, inCompNotTrackedNum);
    }
    cv::destroyAllWindows();

    for (const auto &[key, SAEMapTrackedCirclesGrid] : SAEMapTrackedCirclesGridBackUp) {
        CalibSolverIO::SaveSAEMaps(key, SAEMapTrackedCirclesGrid);
    }

    /**
     * save circle grid patterns to disk
     */
    for (const auto &[topic, patterns] : _extractedPatterns) {
        if (!tryLoadAndSaveRes) {
            continue;
        }
        if (patternLoadFromFile.at(topic)) {
            continue;
        }
        auto [gridPatternPath, rawEvsPath] =
            CalibSolverIO::GetDiskPathOfExtractedGridPatterns(topic);

        spdlog::info("saving extracted circle grid patterns of '{}' to path: '{}'...", topic,
                     gridPatternPath);
        if (!patterns->Save(gridPatternPath, Configor::Preference::OutputDataFormat)) {
            spdlog::warn("failed to save patterns of '{}'!!!", topic);
        } else {
            spdlog::info("saved extracted patterns of '{}' to path finished!", topic);
        }

        spdlog::info("saving raw events of extracted circle grid patterns of '{}' to path: '{}'...",
                     topic, rawEvsPath);
        if (!CalibSolverIO::SaveRawEventsOfExtractedPatterns(
                _rawEventsOfExtractedPatterns.at(topic), rawEvsPath, _dataRawTimestamp.first,
                CerealArchiveType::Enum::BINARY)) {
            spdlog::warn("failed to save raw events of patterns of '{}'!!!", topic);
        } else {
            spdlog::info("saved raw events of extracted patterns of '{}' to path finished!", topic);
        }
    }
}

}  // namespace ns_ekalibr