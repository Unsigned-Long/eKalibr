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
#include "core/sae.h"
#include "sensor/event.h"
#include <config/configor.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

namespace ns_ekalibr {

std::set<int> InCmpPatternTracker::Tracking(
    const std::string& topic,
    const CircleGridPatternPtr& pattern,
    int cenNumThdForEachInCmpPattern,
    double distThdToTrackCen,
    const std::map<int, ExtractedCirclesVec>& tvCirclesWithRawEvs) {
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
        bool newIncmpGridTracked = false;
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
            if (!isAscendingOrder) {
                auto gridTmp = grid1;
                grid1 = grid3;
                grid3 = gridTmp;
            }
            spdlog::info("'isAscendingOrder': {}, try to track 2d grid: [{}, {}, {}]->[{}]",
                         static_cast<int>(isAscendingOrder), grid1->id, grid2->id, grid3->id,
                         gridToTrack->id);

            bool trackSuccess = TryToTrackInCmpGridPattern(topic, grid1, grid2, grid3, gridToTrack,
                                                           distThdToTrackCen, tvCirclesWithRawEvs);
            if (trackSuccess) {
                trackedGridIdx.insert(gridToTrack->id);
                newIncmpGridTracked = true;
            }

            processedIncmpGridIds.insert(gridToTrack->id);
            std::cin.get();
        }
        ++trackingLoopCount;
        isAscendingOrder = !isAscendingOrder;
        if (!newIncmpGridTracked && trackingLoopCount >= 2) {
            break;
        }
    }
    spdlog::info("finished...");
    std::cin.get();
    std::cin.get();

    return {};
}

bool InCmpPatternTracker::TryToTrackInCmpGridPattern(
    const std::string& topic,
    const CircleGrid2DPtr& grid1,
    const CircleGrid2DPtr& grid2,
    const CircleGrid2DPtr& grid3,
    const CircleGrid2DPtr& gridToTrack,
    double distThdToTrackCen,
    const std::map<int, ExtractedCirclesVec>& tvCirclesWithRawEvs) {
    auto m1 = CreateSAE(topic, tvCirclesWithRawEvs.at(grid1->id));
    auto m2 = CreateSAE(topic, tvCirclesWithRawEvs.at(grid2->id));
    auto m3 = CreateSAE(topic, tvCirclesWithRawEvs.at(grid3->id));
    auto m = CreateSAE(topic, tvCirclesWithRawEvs.at(gridToTrack->id));
    cv::imshow("grid1", m1);
    cv::imshow("grid2", m2);
    cv::imshow("grid3", m3);
    cv::imshow("grid to track", m);
    cv::waitKey(0);
    return false;
}

cv::Mat InCmpPatternTracker::CreateSAE(const std::string& topic,
                                       const ExtractedCirclesVec& tvCirclesWithRawEvs) {
    const auto& config = Configor::DataStream::EventTopics.at(topic);
    auto sae = ActiveEventSurface::Create(config.Width, config.Height, 0.01);
    for (const auto& [tvEllipse, evs] : tvCirclesWithRawEvs) {
        sae->GrabEvent(evs);
    }
    // CV_8UC1
    auto tsImg = sae->DecayTimeSurface(true, 0, Configor::Prior::DecayTimeOfActiveEvents);
    // CV_8UC3
    cv::cvtColor(tsImg, tsImg, cv::COLOR_GRAY2BGR);
    return tsImg;
}
}  // namespace ns_ekalibr