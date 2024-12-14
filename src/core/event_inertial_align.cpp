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

#include "core/calib_solver.h"
#include <core/calib_param_mgr.h>
#include "core/extr_rot_estimator.h"
#include <util/tqdm.h>
#include "util//status.hpp"
#include <core/estimator.h>

namespace ns_ekalibr {
void CalibSolver::EventInertialAlignment() const {
    const auto& so3Spline = _splines->GetSo3Spline(Configor::Preference::SO3_SPLINE);
    const auto& scaleSpline = _splines->GetRdSpline(Configor::Preference::SCALE_SPLINE);
    /**
     * we throw the head and tail data as the rotations from the fitted SO3 Spline in that range are
     * poor
     */
    const double st = std::max(so3Spline.MinTime(), scaleSpline.MinTime()) +  // the max as start
                      Configor::Prior::TimeOffsetPadding;
    const double et = std::min(so3Spline.MaxTime(), scaleSpline.MaxTime()) -  // the min as end
                      Configor::Prior::TimeOffsetPadding;

    static constexpr double DESIRED_TIME_INTERVAL = 0.1 /* 0.1 sed */;
    const int ALIGN_STEP = std::max(
        1, static_cast<int>(DESIRED_TIME_INTERVAL / Configor::Prior::DecayTimeOfActiveEvents));

    /**
     * using the estimated rotations of cameras from the 'cv::solvePnP', we perform extrinsic
     * rotation recovery based on discrete-time rotation-only hand-eye alignment
     */
    for (const auto& [topic, poseVec] : _camPoses) {
        spdlog::info(
            "recover event-inertial extrinsic rotation between '{}' and '{}' based on "
            "discrete-time rotation-only hand-eye alignment...",
            topic, Configor::DataStream::RefIMUTopic);

        const double TO_CjToBr = _parMgr->TEMPORAL.TO_CjToBr.at(topic);

        // sensor-inertial rotation estimator (linear least-squares problem)
        const auto rotEstimator = ExtrRotEstimator::Create();

        ExtrRotEstimator::RelRotationSequence relRotSequence;
        relRotSequence.reserve(poseVec.size());

        auto bar = std::make_shared<tqdm>();
        for (int i = 0; i < static_cast<int>(poseVec.size()) - ALIGN_STEP; i++) {
            bar->progress(i, static_cast<int>(poseVec.size()));

            const auto& sPose = poseVec.at(i);
            const auto& ePose = poseVec.at(i + ALIGN_STEP);

            if (sPose.timeStamp + TO_CjToBr < st || ePose.timeStamp + TO_CjToBr > et) {
                continue;
            }

            auto Rot_EndToStart = sPose.so3.inverse() /*from w to s*/ * ePose.so3 /*from e to w*/;
            relRotSequence.emplace_back(sPose.timeStamp, ePose.timeStamp, Rot_EndToStart);

            // estimate the extrinsic rotation
            rotEstimator->Estimate(so3Spline, relRotSequence);

            // check solver status
            if (rotEstimator->SolveStatus()) {
                // assign the estimated extrinsic rotation
                _parMgr->EXTRI.SO3_CjToBr.at(topic) = rotEstimator->GetSO3SensorToSpline();
                // once we solve the rotation successfully, break this for loop
                bar->finish();
                break;
            }
        }
        if (!rotEstimator->SolveStatus()) {
            throw Status(Status::ERROR,
                         "initialize rotation 'SO3_CjToBr' failed, this may be related to "
                         "insufficiently excited motion or bad images.");
        } else {
            spdlog::info("extrinsic rotation of '{}' is recovered using '{:06}' frames", topic,
                         relRotSequence.size());
        }
    }

    /**
     * in last step, we use a discrete-time rotation-only hand-eye alignment, where only the
     * extrinsic rotations are considered, but here we use a continuous-time rotation-only
     * hand-eye alignment where both extrinsic rotations and temporal parameters are considered.
     */
    if (Configor::Prior::OptTemporalParams) {
        for (const auto& [topic, poseVec] : _camPoses) {
            spdlog::info(
                "refine event-inertial extrinsic rotation and time offsets between '{}' and '{}' "
                "based on continuous-time rotation-only hand-eye alignment...",
                topic, Configor::DataStream::RefIMUTopic);

            const double TO_CjToBr = _parMgr->TEMPORAL.TO_CjToBr.at(topic);

            auto estimator = Estimator::Create(_splines, _parMgr);
            const auto optOption = OptOption::OPT_SO3_CjToBr | OptOption::OPT_TO_CjToBr;
            const double weight = Configor::DataStream::EventTopics.at(topic).Weight;

            for (int i = 0; i < static_cast<int>(poseVec.size()) - ALIGN_STEP; i++) {
                const auto& sPose = poseVec.at(i);
                const auto& ePose = poseVec.at(i + ALIGN_STEP);

                if (sPose.timeStamp + TO_CjToBr < st || ePose.timeStamp + TO_CjToBr > et) {
                    continue;
                }

                estimator->AddHandEyeRotAlignment(
                    topic,            // the ros topic
                    sPose.timeStamp,  // the time of start rotation stamped by the camera
                    ePose.timeStamp,  // the time of end rotation stamped by the camera
                    sPose.so3,        // the start rotation
                    ePose.so3,        // the end rotation
                    optOption,        // the optimization option
                    weight            // the weight
                );
            }
            auto sum = estimator->Solve(_ceresOption);
            spdlog::info("here is the summary:\n{}\n", sum.BriefReport());
        }
    }
}

}  // namespace ns_ekalibr