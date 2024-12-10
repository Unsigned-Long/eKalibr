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

#include "core/sae.h"
#include "sensor/event.h"
#include "opencv4/opencv2/imgproc.hpp"

namespace ns_ekalibr {
ActiveEventSurface::ActiveEventSurface(int w, int h, double filterThd)
    : FILTER_THD(filterThd),
      w(w),
      h(h),
      _accEventImg(cv::Size(w, h), CV_8UC3, cv::Scalar(0, 0, 0)) {
    _sae[0] = Eigen::MatrixXd::Zero(w, h);
    _sae[1] = Eigen::MatrixXd::Zero(w, h);
    _saeLatest[0] = Eigen::MatrixXd::Zero(w, h);
    _saeLatest[1] = Eigen::MatrixXd::Zero(w, h);
    _timeLatest = 0.0;
}

ActiveEventSurface::Ptr ActiveEventSurface::Create(int w, int h, double filterThd) {
    return std::make_shared<ActiveEventSurface>(w, h, filterThd);
}

void ActiveEventSurface::GrabEvent(const Event::Ptr &event, bool drawEventMat) {
    const bool ep = event->GetPolarity();
    const std::uint16_t ex = event->GetPos()(0), ey = event->GetPos()(1);
    const double et = event->GetTimestamp();

    // update Surface of Active Events
    const int pol = ep ? 1 : 0;
    const int polInv = !ep ? 1 : 0;
    double &tLast = _saeLatest[pol](ex, ey);
    double &tLastInv = _saeLatest[polInv](ex, ey);

    if ((et > tLast + FILTER_THD) || (tLastInv > tLast)) {
        tLast = et;
        _sae[pol](ex, ey) = et;
    } else {
        tLast = et;
    }
    _timeLatest = et;

    if (drawEventMat) {
        // draw image
        _accEventImg.at<cv::Vec3b>(cv::Point2d(ex, ey)) =
            ep ? cv::Vec3b(255, 0, 0) : cv::Vec3b(0, 0, 255);
    }
}

void ActiveEventSurface::GrabEvent(const EventArray::Ptr &events, bool drawAccumulatedEventMat) {
    for (const auto &event : events->GetEvents()) {
        GrabEvent(event, drawAccumulatedEventMat);
    }
}

cv::Mat ActiveEventSurface::GetAccumulatedEventImg(bool resetMat) {
    auto mat = _accEventImg.clone();
    if (resetMat) {
        _accEventImg = cv::Mat(cv::Size(w, h), CV_8UC3, cv::Scalar(0, 0, 0));
    }
    return mat;
}

cv::Mat ActiveEventSurface::DecayTimeSurface(bool ignorePolarity,
                                        int medianBlurKernelSize,
                                        double decaySec) {
    // create exponential-decayed Time Surface map
    const auto imgSize = cv::Size(w, h);
    cv::Mat timeSurfaceMap = cv::Mat::zeros(imgSize, CV_64F);

    for (int y = 0; y < imgSize.height; ++y) {
        for (int x = 0; x < imgSize.width; ++x) {
            const double mostRecentStampAtCoord = std::max(_sae[1](x, y), _sae[0](x, y));

            const double dt = _timeLatest - mostRecentStampAtCoord;
            double expVal = std::exp(-dt / decaySec);

            if (!ignorePolarity) {
                double polarity = _sae[1](x, y) > _sae[0](x, y) ? 1.0 : -1.0;
                expVal *= polarity;
            }
            timeSurfaceMap.at<double>(y, x) = expVal;
        }
    }

    if (!ignorePolarity) {
        timeSurfaceMap = 255.0 * (timeSurfaceMap + 1.0) / 2.0;
    } else {
        timeSurfaceMap = 255.0 * timeSurfaceMap;
    }
    timeSurfaceMap.convertTo(timeSurfaceMap, CV_8U);

    if (medianBlurKernelSize > 0) {
        cv::medianBlur(timeSurfaceMap, timeSurfaceMap, 2 * medianBlurKernelSize + 1);
    }

    return timeSurfaceMap;
}

std::pair<cv::Mat, cv::Mat> ActiveEventSurface::RawTimeSurface(bool ignorePolarity) {
    // create exponential-decayed Time Surface map
    const auto imgSize = cv::Size(w, h);
    cv::Mat timeSurfaceMap = cv::Mat::zeros(imgSize, CV_64FC1);
    cv::Mat polarityMap = cv::Mat::zeros(imgSize, CV_8UC1);

    for (int y = 0; y < imgSize.height; ++y) {
        for (int x = 0; x < imgSize.width; ++x) {
            double mostRecentStampAtCoord = std::max(_sae[1](x, y), _sae[0](x, y));
            double polarity = _sae[1](x, y) > _sae[0](x, y) ? 1.0 : -1.0;
            polarityMap.at<uchar>(y, x) = polarity;
            if (!ignorePolarity) {
                mostRecentStampAtCoord *= polarity;
            }
            timeSurfaceMap.at<double>(y, x) = mostRecentStampAtCoord;
        }
    }
    return {timeSurfaceMap, polarityMap};
}

double ActiveEventSurface::GetTimeLatest() const { return _timeLatest; }
}  // namespace ns_ekalibr