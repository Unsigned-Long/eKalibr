// iKalibr: Unified Targetless Spatiotemporal Calibration Framework
// Copyright 2024, the School of Geodesy and Geomatics (SGG), Wuhan University, China
// https://github.com/Unsigned-Long/iKalibr.git
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

#ifndef ELLIPSE_FITTING_HPP
#define ELLIPSE_FITTING_HPP

#include "ceres/dynamic_autodiff_cost_function.h"
#include "sensor/event.h"

namespace ns_ekalibr {

struct TimeVaryingCircleFittingFactor {
private:
    Eigen::Vector2d tVec;
    double ex, ey;
    double weight;

public:
    TimeVaryingCircleFittingFactor(const Event::Ptr &event, double weight)
        : tVec(event->GetTimestamp(), 1.0),
          ex(event->GetPos()(0)),
          ey(event->GetPos()(1)),
          weight(weight) {}

    static auto Create(const Event::Ptr &event, double weight) {
        return new ceres::DynamicAutoDiffCostFunction<TimeVaryingCircleFittingFactor>(
            new TimeVaryingCircleFittingFactor(event, weight));
    }

    static std::size_t TypeHashCode() { return typeid(TimeVaryingCircleFittingFactor).hash_code(); }

public:
    /**
     * param blocks:
     * [ cx: b, c | cy: b, c | m: b, c ]
     */
    template <class T>
    bool operator()(T const *const *params, T *residuals) const {
        Eigen::Map<const Eigen::Vector2<T>> cxParam(params[0]);
        Eigen::Map<const Eigen::Vector2<T>> cyParam(params[1]);
        Eigen::Map<const Eigen::Vector2<T>> mParam(params[2]);

        T cx = cxParam.dot(tVec.cast<T>());
        T cy = cyParam.dot(tVec.cast<T>());
        T m = mParam.dot(tVec.cast<T>());
        T r = m * m;

        T vx2 = (static_cast<T>(ex) - cx) * (static_cast<T>(ex) - cx);
        T vy2 = (static_cast<T>(ey) - cy) * (static_cast<T>(ey) - cy);

        residuals[0] = static_cast<T>(weight) * (vx2 + vy2 - r * r);

        return true;
    }
};
}  // namespace ns_ekalibr

#endif  // ELLIPSE_FITTING_HPP
