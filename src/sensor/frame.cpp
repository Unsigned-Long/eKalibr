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

#include "sensor/frame.h"
#include "opencv2/imgproc.hpp"

ns_ekalibr::Frame::Frame(double timestamp, const cv::Mat& img)
    : _timestamp(timestamp),
      _img(img) {}

ns_ekalibr::Frame::Ptr ns_ekalibr::Frame::Create(double timestamp, const cv::Mat& img) {
    return std::make_shared<Frame>(timestamp, img);
}

double ns_ekalibr::Frame::GetTimestamp() const { return _timestamp; }

void ns_ekalibr::Frame::SetTimestamp(double timestamp) { _timestamp = timestamp; }

const cv::Mat& ns_ekalibr::Frame::GetImg() const { return _img; }

void ns_ekalibr::Frame::SetImg(const cv::Mat& img) { _img = img; }

cv::Mat ns_ekalibr::Frame::GetGrayImg() const {
    if (_img.channels() == 3) {
        cv::Mat grayImg;
        cv::cvtColor(_img, grayImg, cv::COLOR_BGR2GRAY);
        return grayImg;
    } else if (_img.channels() == 1) {
        return _img;
    } else {
        throw std::runtime_error("Unsupported image channels for grayscale conversion.");
    }
}

cv::Mat ns_ekalibr::Frame::GetColorImg() const {
    if (_img.channels() == 1) {
        cv::Mat colorImg;
        cv::cvtColor(_img, colorImg, cv::COLOR_GRAY2BGR);
        return colorImg;
    } else if (_img.channels() == 3) {
        return _img;
    } else {
        throw std::runtime_error("Unsupported image channels for color image conversion.");
    }
}