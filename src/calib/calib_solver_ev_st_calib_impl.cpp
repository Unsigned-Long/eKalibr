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
#include <magic_enum_flags.hpp>
#include "calib/calib_solver_io.h"

namespace ns_ekalibr {
void CalibSolver::InitSplineSegmentsOfRefCamUsingCamPose(bool onlyRefCam,
                                                         double SEG_NEIGHBOR,
                                                         double SEG_LENGTH) {
    if (onlyRefCam) {
        this->BreakTimelineToSegments(SEG_NEIGHBOR, SEG_LENGTH, _refEvTopic, false);
    } else {
        this->BreakTimelineToSegments(SEG_NEIGHBOR, SEG_LENGTH, {}, false);
    }
    // SEG_NEIGHBOR[Configor::Prior::DecayTimeOfActiveEvents * 5.0] * 2.0
    const double dtRoughSpline = SEG_NEIGHBOR * 2.0;
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
            if (idx == std::nullopt) {
                continue;
            }
            // from {Cr} to {W}
            auto SE3_CrToW = pose.se3() * SE3_CrToCj;
            estimator->AddSo3Constraint(_splineSegments.at(*idx).first, time, SE3_CrToW.so3(), opt,
                                        10.0);
            estimator->AddPositionConstraint(_splineSegments.at(*idx).second, time,
                                             SE3_CrToW.translation(), opt, 10.0);
        }
    }
    for (auto &[so3Spline, posSpline] : _splineSegments) {
        estimator->AddRegularizationL2Constraint(so3Spline, opt, 1E-3);
        estimator->AddRegularizationL2Constraint(posSpline, opt, 1E-3);
    }
    // we don't want to output the solving information
    auto sum = estimator->Solve(Estimator::DefaultSolverOptions(-1, false, false), nullptr);
    spdlog::info("here is the summary:\n{}\n", sum.BriefReport());

    // fitting small-knot-distance segments
    auto roughSplineSegments = _splineSegments;
    if (onlyRefCam) {
        this->BreakTimelineToSegments(SEG_NEIGHBOR, SEG_LENGTH, _refEvTopic, false);
    } else {
        this->BreakTimelineToSegments(SEG_NEIGHBOR, SEG_LENGTH, {}, false);
    }
    // Configor::Prior::DecayTimeOfActiveEvents * 2.5
    const double dtDelicateSpline = Configor::Prior::DecayTimeOfActiveEvents * 2.5;
    this->CreateSplineSegments(dtDelicateSpline, dtDelicateSpline);

    spdlog::info(
        "fitting small-knot-distance spline segments using initialized rough spline "
        "segments...");
    estimator = Estimator::Create(_parMgr);
    for (const auto &[so3Spline, posSpline] : roughSplineSegments) {
        auto st = std::min(so3Spline.MinTime(), posSpline.MinTime());
        auto et = std::max(so3Spline.MaxTime(), posSpline.MaxTime());
        for (double t = st; t < et; t += Configor::Prior::DecayTimeOfActiveEvents * 0.5) {
            if (!so3Spline.TimeStampInRange(t) || !posSpline.TimeStampInRange(t)) {
                continue;
            }
            auto idx = this->IsTimeInValidSegment(t);
            if (idx == std::nullopt) {
                continue;
            }
            const auto so3 = so3Spline.Evaluate(t);
            const Eigen::Vector3d pos = posSpline.Evaluate(t);
            estimator->AddSo3Constraint(_splineSegments.at(*idx).first, t, so3, opt, 1.0);
            estimator->AddPositionConstraint(_splineSegments.at(*idx).second, t, pos, opt, 1.0);
        }
    }
    for (auto &[so3Spline, posSpline] : _splineSegments) {
        estimator->AddRegularizationL2Constraint(so3Spline, opt, 1E-3);
        estimator->AddRegularizationL2Constraint(posSpline, opt, 1E-3);
    }
    sum = estimator->Solve(Estimator::DefaultSolverOptions(-1, false, false), nullptr);
    spdlog::info("here is the summary:\n{}\n", sum.BriefReport());
}

void CalibSolver::EvCamSpatialTemporalCalib() {
    if (Configor::DataStream::EventTopics.size() == 1) {
        return;
    }
    if (CirclePatternType::SYMMETRIC_GRID ==
        CirclePattern::FromString(Configor::Prior::CirclePattern.Type)) {
        spdlog::warn(
            "symmetric circle grid pattern cannot be used to perform motion-based visual intrinsic "
            "refinement due to 180-degree ambiguity!");
        return;
    }

    const double SEG_NEIGHBOR = Configor::Prior::DecayTimeOfActiveEvents * 5; /*neighbor*/
    const double SEG_LENGTH = Configor::Prior::DecayTimeOfActiveEvents * 50;  /*length*/

    /**
     * Here, we choose the event camera with the longest total duration as the reference camera
     * Modify: '_refEvTopic', '_validTimeSegments'
     */
    {
        _refEvTopic = Configor::DataStream::EventTopics.cbegin()->first;
        double timeSum =
            this->BreakTimelineToSegments(SEG_NEIGHBOR, SEG_LENGTH, _refEvTopic, false);
        for (const auto &[topic, _] : Configor::DataStream::EventTopics) {
            if (topic == _refEvTopic) {
                continue;
            }
            double ts = this->BreakTimelineToSegments(SEG_NEIGHBOR, SEG_LENGTH, topic, false);
            if (ts > timeSum) {
                _refEvTopic = topic;
                timeSum = ts;
            }
        }
        // update the '_validTimeSegments' as those of the '_refEvTopic'
        timeSum = this->BreakTimelineToSegments(SEG_NEIGHBOR, SEG_LENGTH, _refEvTopic, true);
        spdlog::info("choose camera '{}' as the reference camera, tracked age: {:.3f}", _refEvTopic,
                     timeSum);
    }

    // initialize the spline segments using poses from the reference camera
    this->InitSplineSegmentsOfRefCamUsingCamPose(true, SEG_NEIGHBOR, SEG_LENGTH);
    CalibSolverIO::SaveStageCalibParam(_parMgr, "multi_camera_calib_0_spline_init");

    /**
     * initialize extrinsics and time offsets of other event cameras
     */
    {
        auto opt = OptOption::OPT_SO3_CjToBr | OptOption::OPT_POS_BiInBr;
        if (Configor::Prior::OptTemporalParams) {
            opt |= OptOption::OPT_TO_CjToBr;
        }
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
                if (sIdx == std::nullopt || eIdx == std::nullopt || *sIdx != *eIdx) {
                    continue;
                }

                estimator->AddHandEyeTransformAlignment(
                    _splineSegments.at(*sIdx).first, _splineSegments.at(*sIdx).second,
                    topic,            // the ros topic
                    sPose.timeStamp,  // the time of start transformation stamped by the camera
                    ePose.timeStamp,  // the time of end transformation stamped by the camera
                    sPose.se3(),      // the start transformation
                    ePose.se3(),      // the end transformation
                    opt,              // the optimization option
                    weight            // the weight
                );
            }
            estimator->SetEvCamParamsConstant(_refEvTopic);
            auto sum = estimator->Solve(_ceresOption, nullptr);
            spdlog::info("here is the summary:\n{}\n", sum.BriefReport());
        }
    }
    CalibSolverIO::SaveStageCalibParam(_parMgr, "multi_camera_calib_1_st_init");

    /**
     * extend the spline segments from all cameras
     */
    // initialize the spline segments using poses from all cameras
    this->InitSplineSegmentsOfRefCamUsingCamPose(false, SEG_NEIGHBOR, SEG_LENGTH);

    // create visual projection pairs
    this->CreateVisualProjPairsAsyncPointBased();

    std::array<OptOption, 1> optionAry = {OptOption::OPT_SO3_CjToBr | OptOption::OPT_POS_CjInBr |
                                          OptOption::OPT_TO_CjToBr | OptOption::OPT_SO3_SPLINE |
                                          OptOption::OPT_SCALE_SPLINE};
    /**
     *std::array<OptOption, 2> optionAry = {
     *   // the first one
     *   OptOption::OPT_SO3_CjToBr | OptOption::OPT_POS_CjInBr | OptOption::OPT_TO_CjToBr,
     *   // the second one (append to last)
     *   OptOption::OPT_SO3_SPLINE | OptOption::OPT_SCALE_SPLINE};
     */

    std::vector options(optionAry.size(), OptOption::NONE);
    for (int i = 0; i < static_cast<int>(optionAry.size()); ++i) {
        options.at(i) = optionAry.at(i);
        // append
        if (i != 0) {
            options.at(i) |= options.at(i - 1);
        }

        if (!Configor::Prior::OptTemporalParams &&
            IsOptionWith(OptOption::OPT_TO_CjToBr, options.at(i))) {
            options.at(i) ^= OptOption::OPT_TO_CjToBr;
        }
    }
    for (int i = 0; i < static_cast<int>(options.size()); ++i) {
        const auto &option = options.at(i);
        std::stringstream stringStream;
        stringStream << magic_enum::enum_flags_name(option);
        spdlog::info("performing the '{}'-th batch optimization, option:\n{}", i,
                     stringStream.str());

        auto estimator = Estimator::Create(_parMgr);
        for (const auto &[topic, _] : Configor::DataStream::EventTopics) {
            auto s = this->AddVisualProjPairsAsyncPointBasedToSplineSegments(estimator, topic,
                                                                             option, {});
            spdlog::info("add '{}' 'VisualProjectionFactor' for camera '{}'...", s, topic);
        }
        for (auto &[so3Spline, posSpline] : _splineSegments) {
            estimator->AddRegularizationL2Constraint(so3Spline, option, 1E-3);
            estimator->AddRegularizationL2Constraint(posSpline, option, 1E-3);
        }
        // make this problem full rank
        estimator->SetEvCamParamsConstant(_refEvTopic);
        auto sum = estimator->Solve(_ceresOption, nullptr);
        spdlog::info("here is the summary:\n{}\n", sum.BriefReport());
    }

    CalibSolverIO::SaveStageCalibParam(_parMgr, "multi_camera_calib_2_ba");
}

}  // namespace ns_ekalibr