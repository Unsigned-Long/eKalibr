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

#ifndef INCMP_PATTERN_TRACKING_H
#define INCMP_PATTERN_TRACKING_H

#include "util/utils.h"

namespace ns_ekalibr {

struct CircleGridPattern;
using CircleGridPatternPtr = std::shared_ptr<CircleGridPattern>;
struct CircleGrid2D;
using CircleGrid2DPtr = std::shared_ptr<CircleGrid2D>;
struct TimeVaryingEllipse;
using TimeVaryingEllipsePtr = std::shared_ptr<TimeVaryingEllipse>;
struct EventArray;
using EventArrayPtr = std::shared_ptr<EventArray>;

struct InCmpPatternTracker {
    // for a tracked 2d grid pattern
    using ExtractedCirclesVec = std::vector<std::pair<TimeVaryingEllipsePtr, EventArrayPtr>>;

public:
    static std::set<int> Tracking(const std::string& topic,
                                  const CircleGridPatternPtr& pattern,
                                  int cenNumThdForEachInCmpPattern,
                                  double distThdToTrackCen,
                                  std::map<int, ExtractedCirclesVec>& tvCirclesWithRawEvs);

    static cv::Mat CreateSAEWithCircles(const std::string& topic,
                                        const ExtractedCirclesVec& tvCirclesWithRawEvs,
                                        const CircleGrid2DPtr& grid);

protected:
    static std::vector<int> TryToTrackInCmpGridPattern(
        const std::string& topic,
        const CircleGrid2DPtr& grid1,
        const CircleGrid2DPtr& grid2,
        const CircleGrid2DPtr& grid3,
        const CircleGrid2DPtr& gridToTrack,
        double distThdToTrackCen,
        const std::map<int, ExtractedCirclesVec>& tvCirclesWithRawEvs);

    static cv::Mat CreateSAE(const std::string& topic,
                             const ExtractedCirclesVec& tvCirclesWithRawEvs);

    static void DrawTrace(cv::Mat& img,
                          double t1,
                          double t2,
                          double t3,
                          double timePadding,
                          const cv::Point2f& p1,
                          const cv::Point2f& p2,
                          const cv::Point2f& p3,
                          const int pixelDist);
};
}  // namespace ns_ekalibr

#endif  // INCMP_PATTERN_TRACKING_H
