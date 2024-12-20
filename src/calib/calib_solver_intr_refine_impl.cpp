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
#include <core/circle_grid.h>
#include "factor/visual_projection_circle_based_factor.hpp"
#include "util/utils_tpl.hpp"

namespace ns_ekalibr {
void CalibSolver::InitSplineSegmentsOfRefCamUsingCamPose(bool onlyRefCam,
                                                         double SEG_NEIGHBOR,
                                                         double SEG_LENGTH) {
    if (onlyRefCam) {
        this->BreakTimelineToSegments(SEG_NEIGHBOR, SEG_LENGTH, _refEvTopic);
    } else {
        this->BreakTimelineToSegments(SEG_NEIGHBOR, SEG_LENGTH, {});
    }
    const double dtRoughSpline = Configor::Prior::DecayTimeOfActiveEvents * 10.0;
    this->CreateSplineSegments(dtRoughSpline, dtRoughSpline);
    const auto opt = OptOption::OPT_SO3_SPLINE | OptOption::OPT_SCALE_SPLINE;

    // fitting rough spline segments
    spdlog::info("fitting rough spline segments using visual poses of cameras...");
    auto estimator = Estimator::Create(_parMgr);
    for (const auto &[topic, camPoseVec] : _camPoses) {
        if (onlyRefCam && topic != _refEvTopic) {
            continue;
        }
        auto SE3_CrToCj = _parMgr->EXTRI.SE3_CjToBr(topic).inverse();
        auto TO_CjToCr = _parMgr->TEMPORAL.TO_CjToBr.at(topic);
        for (const auto &pose : camPoseVec) {
            double time = pose.timeStamp + TO_CjToCr;
            auto idx = this->IsTimeInValidSegment(time);
            if (idx < 0 || idx >= static_cast<int>(_splineSegments.size())) {
                continue;
            }
            // from {Cr} to {W}
            auto SE3_CrToW = pose.se3() * SE3_CrToCj;
            estimator->AddSo3Constraint(_splineSegments.at(idx).first, time, SE3_CrToW.so3(),
                                        OptOption::OPT_SO3_SPLINE, 10.0);
            estimator->AddPositionConstraint(_splineSegments.at(idx).second, time,
                                             SE3_CrToW.translation(), OptOption::OPT_SCALE_SPLINE,
                                             10.0);
        }
    }
    for (auto &[so3Spline, posSpline] : _splineSegments) {
        estimator->AddSo3LinearConstraint(so3Spline, opt, 1.0);
        estimator->AddPosLinearConstraint(posSpline, opt, 1.0);
    }
    auto sum = estimator->Solve(_ceresOption, nullptr);
    spdlog::info("here is the summary:\n{}\n", sum.BriefReport());

    // fitting small-knot-distance segments
    auto roughSplineSegments = _splineSegments;
    if (onlyRefCam) {
        this->BreakTimelineToSegments(SEG_NEIGHBOR, SEG_LENGTH, _refEvTopic);
    } else {
        this->BreakTimelineToSegments(SEG_NEIGHBOR, SEG_LENGTH, {});
    }
    this->CreateSplineSegments(Configor::Prior::KnotTimeDist.So3Spline,
                               Configor::Prior::KnotTimeDist.ScaleSpline);

    spdlog::info(
        "fitting small-knot-distance spline segments using initialized rough spline "
        "segments...");
    estimator = Estimator::Create(_parMgr);
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
    for (auto &[so3Spline, posSpline] : _splineSegments) {
        estimator->AddSo3LinearConstraint(so3Spline, opt, 1.0);
        estimator->AddPosLinearConstraint(posSpline, opt, 1.0);
    }
    sum = estimator->Solve(_ceresOption, nullptr);
    spdlog::info("here is the summary:\n{}\n", sum.BriefReport());
}

void CalibSolver::RefineCameraIntrinsicsUsingRawEvents() {
    if (CirclePatternType::SYMMETRIC_GRID ==
        CirclePattern::FromString(Configor::Prior::CirclePattern.Type)) {
        spdlog::warn(
            "symmetric circle grid pattern cannot be used to perform motion-based visual intrinsic "
            "refinement due to 180-degree ambiguity!");
        return;
    }

    const double SEG_NEIGHBOR = Configor::Prior::DecayTimeOfActiveEvents * 2.5; /*neighbor*/
    const double SEG_LENGTH = Configor::Prior::DecayTimeOfActiveEvents * 5;     /*length*/

    /**
     * Here, we choose the event camera with the longest total duration as the reference camera
     * Modify: '_refEvTopic', '_validTimeSegments'
     */
    {
        _refEvTopic = Configor::DataStream::EventTopics.cbegin()->first;
        double timeSum = this->BreakTimelineToSegments(SEG_NEIGHBOR, SEG_LENGTH, _refEvTopic);
        if (Configor::DataStream::EventTopics.size() > 1) {
            for (const auto &[topic, _] : Configor::DataStream::EventTopics) {
                if (topic == _refEvTopic) {
                    continue;
                }
                double ts = this->BreakTimelineToSegments(SEG_NEIGHBOR, SEG_LENGTH, topic);
                if (ts > timeSum) {
                    _refEvTopic = topic;
                    timeSum = ts;
                }
            }
            // update the '_validTimeSegments' as those of the '_refEvTopic'
            this->BreakTimelineToSegments(SEG_NEIGHBOR, SEG_LENGTH, _refEvTopic);
        }
        spdlog::info("choose camera '{}' as the reference camera, tracked age: {:.3f}", _refEvTopic,
                     timeSum);
    }

    // initialize the spline segments using poses from the reference camera
    this->InitSplineSegmentsOfRefCamUsingCamPose(true, SEG_NEIGHBOR, SEG_LENGTH);

    // create visual projection pairs
    this->CreateVisualProjPairsAsyncPointBased();

    /**
     * We use batch optimization to refine the spline of the reference event camera.
     * Modify: '_splineSegments'
     */
    {
        auto estimator = Estimator::Create(_parMgr);
        spdlog::info(
            "use circle-based batch optimization to refine the spline segments of of the reference "
            "event camera...");
        auto opt = OptOption::OPT_SO3_SPLINE | OptOption::OPT_SCALE_SPLINE;
        // add circle-based visual projection pairs
        this->AddVisualProjPairsAsyncPointBasedToSplineSegments(estimator, _refEvTopic, opt, 1.0);
        for (auto &[so3Spline, posSpline] : _splineSegments) {
            estimator->AddSo3LinearConstraint(so3Spline, OptOption::OPT_SO3_SPLINE, 50.0);
            estimator->AddPosLinearConstraint(posSpline, OptOption::OPT_SCALE_SPLINE, 50.0);
        }
        auto sum = estimator->Solve(_ceresOption, nullptr);
        spdlog::info("here is the summary:\n{}\n", sum.BriefReport());
    }

    /**
     * initialize extrinsics and time offsets of other event cameras
     */
    {
        auto opt = OptOption::OPT_SO3_CjToBr | OptOption::OPT_POS_BiInBr | OptOption::OPT_TO_CjToBr;
        static constexpr double DESIRED_TIME_INTERVAL = 0.1 /* 0.1 sed */;
        const int ALIGN_STEP = std::max(
            1, static_cast<int>(DESIRED_TIME_INTERVAL / Configor::Prior::DecayTimeOfActiveEvents));

        for (const auto &[topic, poseVec] : _camPoses) {
            if (topic == _refEvTopic) {
                continue;
            }
            spdlog::info(
                "initialize event extrinsics and time offsets between '{}' and '{}' based on "
                "continuous-time hand-eye alignment...",
                topic, _refEvTopic);

            const double TO_CjToBr = _parMgr->TEMPORAL.TO_CjToBr.at(topic);
            auto estimator = Estimator::Create(_parMgr);
            const double weight = Configor::DataStream::EventTopics.at(topic).Weight;

            for (int i = 0; i < static_cast<int>(poseVec.size()) - ALIGN_STEP; i++) {
                const auto &sPose = poseVec.at(i);
                const auto &ePose = poseVec.at(i + ALIGN_STEP);

                auto sIdx = IsTimeInValidSegment(sPose.timeStamp + TO_CjToBr);
                auto eIdx = IsTimeInValidSegment(ePose.timeStamp + TO_CjToBr);
                if (sIdx < 0 || sIdx >= static_cast<int>(_splineSegments.size()) || eIdx < 0 ||
                    eIdx >= static_cast<int>(_splineSegments.size()) || sIdx != eIdx) {
                    continue;
                }

                estimator->AddHandEyeTransformAlignment(
                    _splineSegments.at(sIdx).first, _splineSegments.at(sIdx).second,
                    topic,            // the ros topic
                    sPose.timeStamp,  // the time of start rotation stamped by the camera
                    ePose.timeStamp,  // the time of end rotation stamped by the camera
                    sPose.se3(),      // the start rotation
                    ePose.se3(),      // the end rotation
                    opt,              // the optimization option
                    weight            // the weight
                );
            }
            estimator->SetEvCamParamsConstant(_refEvTopic);
            auto sum = estimator->Solve(_ceresOption, nullptr);
            spdlog::info("here is the summary:\n{}\n", sum.BriefReport());
            _parMgr->ShowParamStatus();
            std::cin.get();
        }
    }

    /**
     * extend the spline segments from all cameras
     */
    this->InitSplineSegmentsOfRefCamUsingCamPose(false, SEG_NEIGHBOR, SEG_LENGTH);
    std::cin.get();
}

}  // namespace ns_ekalibr