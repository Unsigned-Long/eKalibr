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

#include "core/time_varying_ellipse.h"
#include "util/status.hpp"
#include "list"
#include "factor/time_varying_ellipse_fitting.hpp"
#include "factor/time_varying_circle_fitting.hpp"
#include "ceres/ceres.h"
#include "sophus/ceres_manifold.hpp"

#include <opencv2/imgproc.hpp>

namespace ns_ekalibr {

Ellipse::Ellipse(const Eigen::Vector2d& c, const Eigen::Vector2d& r, const Sophus::SO2d& theta)
    : c(c),
      r(r),
      theta(theta) {}

Ellipse::Ptr Ellipse::Create(const Eigen::Vector2d& c,
                             const Eigen::Vector2d& r,
                             const Sophus::SO2d& theta) {
    return std::make_shared<Ellipse>(c, r, theta);
}

double Ellipse::AvgRadius() const { return (r(0) + r(1)) * 0.5; }

/**
 * EventCircleTracking::TimeVaryingCircle
 */

TimeVaryingEllipse::TimeVaryingEllipse(double st,
                                       double et,
                                       Eigen::Vector2d cx,
                                       Eigen::Vector2d cy,
                                       Eigen::Vector2d mx,
                                       Eigen::Vector2d my,
                                       const Sophus::SO2d& theta,
                                       TVType type)
    : st(st),
      et(et),
      cx(std::move(cx)),
      cy(std::move(cy)),
      mx(std::move(mx)),
      my(std::move(my)),
      theta(theta),
      type(type) {}

TimeVaryingEllipse::Ptr TimeVaryingEllipse::CreateTvCircle(double st,
                                                           double et,
                                                           const Eigen::Vector2d& cx,
                                                           const Eigen::Vector2d& cy,
                                                           const Eigen::Vector2d& m) {
    return std::make_shared<TimeVaryingEllipse>(st, et, cx, cy, m, Eigen::Vector2d::Zero(),
                                                Sophus::SO2d(), TVType::CIRCLE);
}

TimeVaryingEllipse::Ptr TimeVaryingEllipse::CreateTvEllipse(double st,
                                                            double et,
                                                            const Eigen::Vector2d& cx,
                                                            const Eigen::Vector2d& cy,
                                                            const Eigen::Vector2d& mx,
                                                            const Eigen::Vector2d& my,
                                                            const Sophus::SO2d& theta) {
    return std::make_shared<TimeVaryingEllipse>(st, et, cx, cy, mx, my, theta, TVType::ELLIPSE);
}

Eigen::Vector2d TimeVaryingEllipse::PosAt(double t) const {
    Eigen::Vector2d tVec(t, 1.0);
    return {cx.dot(tVec), cy.dot(tVec)};
}

std::vector<Eigen::Vector3d> TimeVaryingEllipse::PosVecAt(double dt) const {
    std::list<Eigen::Vector3d> posList;
    double t = st;
    while (t < et) {
        // t, x, y
        Eigen::Vector2d p = PosAt(t);
        posList.push_back({t, p(0), p(1)});
        t += dt;
    }
    return std::vector<Eigen::Vector3d>{posList.cbegin(), posList.cend()};
}

double TimeVaryingEllipse::RadiusAt(double t) const {
    if (type != TVType::CIRCLE) {
        throw Status(Status::ERROR, "only 'TVType::CIRCLE' have radius!!!");
    }
    Eigen::Vector2d tVec(t, 1.0);
    double mVal = mx.dot(tVec);
    return mVal * mVal;
}

double TimeVaryingEllipse::EllipseAxs1At(double t) const {
    if (type != TVType::ELLIPSE) {
        throw Status(Status::ERROR, "only 'TVType::ELLIPSE' have axes!!!");
    }
    Eigen::Vector2d tVec(t, 1.0);
    double mxVal = mx.dot(tVec);
    return mxVal * mxVal;
}

double TimeVaryingEllipse::EllipseAxs2At(double t) const {
    if (type != TVType::ELLIPSE) {
        throw Status(Status::ERROR, "only 'TVType::ELLIPSE' have axes!!!");
    }
    Eigen::Vector2d tVec(t, 1.0);
    double myVal = my.dot(tVec);
    return myVal * myVal;
}

Ellipse::Ptr TimeVaryingEllipse::EllipseAt(double t) const {
    switch (type) {
        case TVType::NONE: {
            throw Status(Status::ERROR, "'TVType::NONE' can not generates ellipse!!!");
        } break;
        case TVType::CIRCLE: {
            const double radius = RadiusAt(t);
            return Ellipse::Create(PosAt(t), {radius, radius}, theta);
        } break;
        case TVType::ELLIPSE: {
            return Ellipse::Create(PosAt(t), {EllipseAxs1At(t), EllipseAxs2At(t)}, theta);
        } break;
    }
    return {};
}

void TimeVaryingEllipse::Draw(cv::Mat& mat,
                              double timestamp,
                              const std::optional<cv::Scalar>& color) const {
    assert(type != TVType::NONE);
    assert(mat.type() == CV_8UC3);

    Draw(mat, this->EllipseAt(timestamp), this->type, color);
}

void TimeVaryingEllipse::Draw(cv::Mat& mat,
                              const Ellipse::Ptr& e,
                              TVType type,
                              const std::optional<cv::Scalar>& color) {
    assert(type != TVType::NONE);
    assert(mat.type() == CV_8UC3);

    cv::Scalar colorVal;
    if (color.has_value()) {
        colorVal = *color;
    } else {
        switch (type) {
            case TVType::ELLIPSE: {
                colorVal = cv::Scalar(0, 255, 0);
            } break;
            case TVType::CIRCLE: {
                colorVal = cv::Scalar(0, 0, 255);
            } break;
            default: {
                colorVal = cv::Scalar(255, 255, 255);
            }
        }
    }
    switch (type) {
        case TVType::ELLIPSE: {
            constexpr double RAD_DEG = 180.0 / M_PI;
            cv::ellipse(mat, cv::Point2d(e->c(0), e->c(1)), cv::Size2d(e->r(0), e->r(1)),
                        -e->theta.log() * RAD_DEG, 0.0, 360.0, colorVal);
            DrawKeypointOnCVMat(mat, e->c, false, colorVal);
        } break;
        case TVType::CIRCLE: {
            cv::circle(mat, cv::Point2d(e->c(0), e->c(1)), e->AvgRadius(), colorVal, 1);
            DrawKeypointOnCVMat(mat, e->c, false, colorVal);
        } break;
        default: {
        }
    }
}

void TimeVaryingEllipse::FitTimeVaryingCircle(const EventArrayPtr& ary1,
                                              const EventArrayPtr& ary2,
                                              double avgDistThd) {
    if (type != TVType::CIRCLE) {
        throw Status(Status::ERROR, "'FitTimeVaryingCircle' only works for 'TVType::CIRCLE'");
    }
    ceres::Problem problem;

    auto AddResidualsToProblem = [this, &problem, &avgDistThd](const EventArray::Ptr& ary) {
        for (const auto& event : ary->GetEvents()) {
            auto cf = TimeVaryingCircleFittingFactor::Create(event, 1.0);
            cf->AddParameterBlock(2);
            cf->AddParameterBlock(2);
            cf->AddParameterBlock(2);
            cf->SetNumResiduals(1);

            std::vector<double*> params;
            params.push_back(this->cx.data());
            params.push_back(this->cy.data());
            params.push_back(this->mx.data());

            problem.AddResidualBlock(cf, new ceres::HuberLoss(avgDistThd * avgDistThd), params);
        }
    };

    AddResidualsToProblem(ary1);
    AddResidualsToProblem(ary2);

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = 30;
    // options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
}

void TimeVaryingEllipse::FittingTimeVaryingEllipse(const EventArrayPtr& ary, double avgDistThd) {
    if (type != TVType::ELLIPSE) {
        throw Status(Status::ERROR, "'FittingTimeVaryingEllipse' only works for 'TVType::ELLIPSE'");
    }
    ceres::Problem problem;

    for (const auto& event : ary->GetEvents()) {
        auto cf = TimeVaryingEllipseFittingFactor::Create(event, 1.0);
        cf->AddParameterBlock(2);
        cf->AddParameterBlock(2);
        cf->AddParameterBlock(2);
        cf->AddParameterBlock(2);
        cf->AddParameterBlock(2);
        cf->SetNumResiduals(1);

        std::vector<double*> params;
        params.push_back(this->cx.data());
        params.push_back(this->cy.data());
        params.push_back(this->mx.data());
        params.push_back(this->my.data());
        params.push_back(this->theta.data());

        problem.AddResidualBlock(cf, new ceres::HuberLoss(std::pow(avgDistThd, 4)), params);
    }

    problem.SetManifold(this->theta.data(), new Sophus::Manifold<Sophus::SO2>());

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = 50;
    // options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
}

}  // namespace ns_ekalibr