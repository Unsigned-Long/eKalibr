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

#include "calib/calib_solver.h"
#include "calib/calib_param_mgr.h"
#include "spdlog/spdlog.h"
#include "calib/estimator.h"

namespace ns_ekalibr {
void CalibSolver::RefineCameraIntrinsicsUsingRawEvents() {
    /**
     * Due to the possibility that the checkerboard may be intermittently tracked (potentially due
     * to insufficient stimulation leading to an inadequate number of events, or the checkerboard
     * moving out of the field of view), it is necessary to identify the continuous segments for
     * subsequent calibration.
     */
    this->BreakTimelineToSegments(0.5 /*neighbor*/, 1.0 /*len*/);
    double dtSpline = Configor::Prior::DecayTimeOfActiveEvents * 10.0;
    this->CreateSplineSegments(dtSpline, dtSpline);

    // fitting rough segments
    spdlog::info("fitting spline segments using camera poses...");
    auto estimator = Estimator::Create(_parMgr);
    for (const auto &[topic, poseVec] : _camPoses) {
        for (const auto &pose : poseVec) {
            auto idx = this->IsTimeInValidSegment(pose.timeStamp);
            if (idx < 0 || idx >= static_cast<int>(_splineSegments.size())) {
                continue;
            }
            estimator->AddSo3Constraint(_splineSegments.at(idx).first, pose.timeStamp, pose.so3,
                                        OptOption::OPT_SO3_SPLINE, 1.0);
            estimator->AddPositionConstraint(_splineSegments.at(idx).second, pose.timeStamp, pose.t,
                                             OptOption::OPT_SCALE_SPLINE, 1.0);
        }
    }
    for (auto &[so3Spline, posSpline] : _splineSegments) {
        estimator->AddSo3LinearHeadConstraint(so3Spline, OptOption::OPT_SO3_SPLINE, 1.0);
        estimator->AddPosLinearHeadConstraint(posSpline, OptOption::OPT_SCALE_SPLINE, 1.0);
        estimator->AddSo3LinearTailConstraint(so3Spline, OptOption::OPT_SO3_SPLINE, 1.0);
        estimator->AddPosLinearTailConstraint(posSpline, OptOption::OPT_SCALE_SPLINE, 1.0);
    }
    auto sum = estimator->Solve(_ceresOption);
    spdlog::info("here is the summary:\n{}\n", sum.BriefReport());

    // fitting small-knot-distance segments
    auto roughSplineSegments = _splineSegments;
    this->BreakTimelineToSegments(0.5 /*neighbor*/, 1.0 /*len*/);
    dtSpline = Configor::Prior::DecayTimeOfActiveEvents;
    this->CreateSplineSegments(dtSpline, dtSpline);

    estimator = Estimator::Create(_parMgr);
    auto opt = OptOption::OPT_SO3_SPLINE | OptOption::OPT_SCALE_SPLINE;
    for (const auto &[so3Spline, posSpline] : roughSplineSegments) {
        auto st = std::min(so3Spline.MinTime(), posSpline.MinTime());
        auto et = std::max(so3Spline.MaxTime(), posSpline.MaxTime());
        for (double t = st; t < et; t += 0.005) {
            if (!so3Spline.TimeStampInRange(t) || !posSpline.TimeStampInRange(t)) {
                continue;
            }
            auto idx = this->IsTimeInValidSegment(t);
            if (idx < 0 || idx >= static_cast<int>(_splineSegments.size())) {
                continue;
            }
            const auto so3 = so3Spline.Evaluate(t);
            const Eigen::Vector3d pos = posSpline.Evaluate(t);
            estimator->AddSo3Constraint(_splineSegments.at(idx).first, t, so3, opt, 1.0);
            estimator->AddPositionConstraint(_splineSegments.at(idx).second, t, pos, opt, 1.0);
        }
    }
    sum = estimator->Solve(_ceresOption);
    spdlog::info("here is the summary:\n{}\n", sum.BriefReport());

    // todo: using raw event to refine intrinsics and splines
    std::cin.get();
}

}  // namespace ns_ekalibr