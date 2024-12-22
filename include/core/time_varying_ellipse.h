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

#ifndef TIME_VARYING_ELLIPSE_H
#define TIME_VARYING_ELLIPSE_H

#include "util/utils.h"
#include "cereal/cereal.hpp"
#include "sophus/so2.hpp"

namespace Sophus {
template <class Archive, typename ScaleTypes>
void serialize(Archive& archive, Sophus::SO2<ScaleTypes>& m) {
    archive(cereal::make_nvp("uc_x", m.data()[0]), cereal::make_nvp("uc_y", m.data()[1]));
}
}  // namespace Sophus

namespace ns_ekalibr {
struct EventArray;
using EventArrayPtr = std::shared_ptr<EventArray>;

struct Ellipse {
    using Ptr = std::shared_ptr<Ellipse>;
    Eigen::Vector2d c;
    Eigen::Vector2d r;
    Sophus::SO2d theta;

    Ellipse(const Eigen::Vector2d& c = Eigen::Vector2d::Zero(),
            const Eigen::Vector2d& r = Eigen::Vector2d::Zero(),
            const Sophus::SO2d& theta = Sophus::SO2d());

    static Ptr Create(const Eigen::Vector2d& c = Eigen::Vector2d::Zero(),
                      const Eigen::Vector2d& r = Eigen::Vector2d::Zero(),
                      const Sophus::SO2d& theta = Sophus::SO2d());

    double AvgRadius() const;

public:
    template <class Archive>
    void serialize(Archive& ar) {
        ar(CEREAL_NVP(c), CEREAL_NVP(r), CEREAL_NVP(theta));
    }
};

struct TimeVaryingEllipse {
    using Ptr = std::shared_ptr<TimeVaryingEllipse>;

    enum class TVType { NONE, CIRCLE, ELLIPSE };

    double st, et;
    Eigen::Vector2d cx;
    Eigen::Vector2d cy;

    Eigen::Vector2d mx;
    Eigen::Vector2d my;
    Sophus::SO2d theta;

    TVType type = TVType::NONE;

public:
    TimeVaryingEllipse(double st = -1.0,
                       double et = -1.0,
                       Eigen::Vector2d cx = Eigen::Vector2d::Zero(),
                       Eigen::Vector2d cy = Eigen::Vector2d::Zero(),
                       Eigen::Vector2d mx = Eigen::Vector2d::Zero(),
                       Eigen::Vector2d my = Eigen::Vector2d::Zero(),
                       const Sophus::SO2d& theta = Sophus::SO2d(),
                       TVType type = TVType::NONE);

    static Ptr Create(double st,
                      double et,
                      const Eigen::Vector2d& cx,
                      const Eigen::Vector2d& cy,
                      const Eigen::Vector2d& m);

    static Ptr Create(double st,
                      double et,
                      const Eigen::Vector2d& cx,
                      const Eigen::Vector2d& cy,
                      const Eigen::Vector2d& mx,
                      const Eigen::Vector2d& my,
                      const Sophus::SO2d& theta);

public:
    Eigen::Vector2d PosAt(double t) const;

    std::vector<Eigen::Vector3d> PosVecAt(double dt) const;

    double RadiusAt(double t) const;

    double EllipseAxs1At(double t) const;

    double EllipseAxs2At(double t) const;

    Ellipse::Ptr EllipseAt(double t) const;

public:
    void FitTimeVaryingCircle(const EventArrayPtr& ary1,
                              const EventArrayPtr& ary2,
                              double avgDistThd);

    void FittingTimeVaryingEllipse(const EventArrayPtr& ary, double avgDistThd);

public:
    friend std::ostream& operator<<(std::ostream& os, const TimeVaryingEllipse& obj) {
        switch (obj.type) {
            case TVType::NONE:
                return os << "TVType::NONE";
                break;
            case TVType::CIRCLE:
                return os << "TVType::CIRCLE, " << "st: " << obj.st << " et: " << obj.et
                          << " cx: " << obj.cx.transpose() << " cy: " << obj.cy.transpose()
                          << " mx: " << obj.mx.transpose();
                break;
            case TVType::ELLIPSE:
                return os << "TVType::ELLIPSE, " << "st: " << obj.st << " et: " << obj.et
                          << " cx: " << obj.cx.transpose() << " cy: " << obj.cy.transpose()
                          << " mx: " << obj.mx.transpose() << " my: " << obj.my.transpose()
                          << " theta: " << obj.theta.unit_complex().transpose();
                break;
        }
        return os;
    }

public:
    template <class Archive>
    void serialize(Archive& ar) {
        ar(CEREAL_NVP(st), CEREAL_NVP(et), CEREAL_NVP(cx), CEREAL_NVP(cy), CEREAL_NVP(mx),
           CEREAL_NVP(my), CEREAL_NVP(theta));
    }
};

}  // namespace ns_ekalibr

#endif  // TIME_VARYING_ELLIPSE_H
