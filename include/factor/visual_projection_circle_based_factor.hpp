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

#ifndef VISUAL_PROJECTION_CIRCLE_BASED_FACTOR_HPP
#define VISUAL_PROJECTION_CIRCLE_BASED_FACTOR_HPP

#include "factor/visual_projection_factor.hpp"
#include "sensor/event.h"

namespace ns_ekalibr {
struct Circle3D {
    using Ptr = std::shared_ptr<Circle3D>;
    Eigen::Vector3d center;
    double radius;
    // [ xVec | yVec | zVec ]
    Eigen::Matrix3d orientation;
    /**
     * a point on this circle can be obtained by:
     * p = center + radius * xVec * cos(theta) + radius * yVec * sin(theta)
     */

    Circle3D(const Eigen::Vector3d &center, double radius, const Eigen::Matrix3d &orientation)
        : center(center),
          radius(radius),
          orientation(orientation) {}

    static Ptr CreateFromGridPattern(const Eigen::Vector3d &center, const double radius) {
        return std::make_shared<Circle3D>(center, radius, Eigen::Matrix3d::Identity());
    }
};

struct VisualProjectionCircleBasedPair {
public:
    using Ptr = std::shared_ptr<VisualProjectionCircleBasedPair>;

    Circle3D::Ptr circle;
    Event::Ptr ev;

    VisualProjectionCircleBasedPair(const Circle3D::Ptr &circle, const Event::Ptr &ev)
        : circle(circle),
          ev(ev) {}

    static Ptr Create(const Circle3D::Ptr &circle, const Event::Ptr &ev) {
        return std::make_shared<VisualProjectionCircleBasedPair>(circle, ev);
    }
};
}  // namespace ns_ekalibr

#endif  // VISUAL_PROJECTION_CIRCLE_BASED_FACTOR_HPP
