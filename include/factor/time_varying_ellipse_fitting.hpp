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

#ifndef TIME_VARYING_ELLIPSE_FITTING_HPP
#define TIME_VARYING_ELLIPSE_FITTING_HPP

#include "ceres/dynamic_autodiff_cost_function.h"
#include "sensor/event.h"

namespace ns_ekalibr {

struct TimeVaryingEllipseFittingFactor {
private:
    Eigen::Vector2d tVec;
    double ex, ey;
    double weight;

public:
    TimeVaryingEllipseFittingFactor(const Event::Ptr &event, double weight)
        : tVec(event->GetTimestamp(), 1.0),
          ex(event->GetPos()(0)),
          ey(event->GetPos()(1)),
          weight(weight) {}

    static auto Create(const Event::Ptr &event, double weight) {
        return new ceres::DynamicAutoDiffCostFunction<TimeVaryingEllipseFittingFactor>(
            new TimeVaryingEllipseFittingFactor(event, weight));
    }

    static std::size_t TypeHashCode() {
        return typeid(TimeVaryingEllipseFittingFactor).hash_code();
    }

public:
    /**
     * param blocks:
     * [ cx: b, c | cy: b, c | mx: b, c | my: b, c | theta: SO2 ]
     */
    template <class T>
    bool operator()(T const *const *params, T *residuals) const {
        Eigen::Map<const Eigen::Vector2<T>> cxParam(params[0]);
        Eigen::Map<const Eigen::Vector2<T>> cyParam(params[1]);
        Eigen::Map<const Eigen::Vector2<T>> mxParam(params[2]);
        Eigen::Map<const Eigen::Vector2<T>> myParam(params[3]);
        Eigen::Map<const Sophus::SO2<T>> theta(params[4]);

        T cx = cxParam.dot(tVec.cast<T>());
        T cy = cyParam.dot(tVec.cast<T>());
        T mx = mxParam.dot(tVec.cast<T>());
        T my = myParam.dot(tVec.cast<T>());

        T rx = mx * mx;
        T rx2 = rx * rx;

        T ry = my * my;
        T ry2 = ry * ry;

        Eigen::Vector2<T> p(static_cast<T>(ex), static_cast<T>(ey));
        Eigen::Vector2<T> t(cx, cy);
        Eigen::Vector2<T> v = theta * (p - t);

        T vx2 = v(0) * v(0);
        T vy2 = v(1) * v(1);

        residuals[0] = static_cast<T>(weight) * (vx2 * ry2 + vy2 * rx2 - rx2 * ry2);

        return true;
    }
};
}  // namespace ns_ekalibr

#endif  // TIME_VARYING_ELLIPSE_FITTING_HPP
