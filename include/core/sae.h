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

#ifndef SAE_H
#define SAE_H

#include "Eigen/Dense"
#include "opencv4/opencv2/core.hpp"

namespace ns_ekalibr {
struct Event;
using EventPtr = std::shared_ptr<Event>;
struct EventArray;
using EventArrayPtr = std::shared_ptr<EventArray>;

class ActiveEventSurface {
public:
    using Ptr = std::shared_ptr<ActiveEventSurface>;

private:
    const double FILTER_THD;
    int w, h;

    Eigen::MatrixXd _sae[2];        // save sae
    Eigen::MatrixXd _saeLatest[2];  // save previous sae
    double _timeLatest;

    cv::Mat _accEventImg;

public:
    explicit ActiveEventSurface(int w, int h, double filterThd = 0.01);

    static Ptr Create(int w, int h, double filterThd = 0.01);

    void GrabEvent(const EventPtr &event, bool drawEventMat = false);

    void GrabEvent(const EventArrayPtr &events, bool drawAccumulatedEventMat = false);

    [[nodiscard]] cv::Mat GetAccumulatedEventImg(bool resetMat);

    cv::Mat DecayTimeSurface(bool ignorePolarity = false,
                             int medianBlurKernelSize = 0,
                             double decaySec = 0.02);

    std::pair<cv::Mat, cv::Mat> RawTimeSurface(bool ignorePolarity = false);

    [[nodiscard]] double GetTimeLatest() const;
};
}  // namespace ns_ekalibr

#endif  // SAE_H
