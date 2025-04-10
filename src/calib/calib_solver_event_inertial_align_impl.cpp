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
#include <calib/calib_param_mgr.h>
#include "core/extr_rot_estimator.h"
#include <util/tqdm.h>
#include "util//status.hpp"
#include <core/circle_grid.h>
#include <calib/estimator.h>
#include <tiny-viewer/entity/coordinate.h>
#include <viewer/viewer.h>

namespace ns_ekalibr {
void CalibSolver::EventInertialAlignment() {
    /**
     * we throw the head and tail data as the rotations from the fitted SO3 Spline in that range are
     * poor
     */
    const double st = _fullSo3Spline.MinTime() + Configor::Prior::TimeOffsetPadding;
    const double et = _fullSo3Spline.MaxTime() - Configor::Prior::TimeOffsetPadding;

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
            rotEstimator->Estimate(_fullSo3Spline, relRotSequence);

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
    for (const auto& [topic, poseVec] : _camPoses) {
        spdlog::info(
            "refine event-inertial extrinsic rotation and time offsets between '{}' and '{}' "
            "based on continuous-time rotation-only hand-eye alignment...",
            topic, Configor::DataStream::RefIMUTopic);

        const double TO_CjToBr = _parMgr->TEMPORAL.TO_CjToBr.at(topic);

        auto estimator = Estimator::Create(_parMgr);
        auto optOption = OptOption::OPT_SO3_CjToBr;
        if (Configor::Prior::OptTemporalParams) {
            optOption |= OptOption::OPT_TO_CjToBr;
        }
        const double weight = Configor::DataStream::EventTopics.at(topic).Weight;

        for (int i = 0; i < static_cast<int>(poseVec.size()) - ALIGN_STEP; i++) {
            const auto& sPose = poseVec.at(i);
            const auto& ePose = poseVec.at(i + ALIGN_STEP);

            if (sPose.timeStamp + TO_CjToBr < st || ePose.timeStamp + TO_CjToBr > et) {
                continue;
            }

            estimator->AddHandEyeRotAlignment(
                _fullSo3Spline,
                topic,            // the ros topic
                sPose.timeStamp,  // the time of start rotation stamped by the camera
                ePose.timeStamp,  // the time of end rotation stamped by the camera
                sPose.so3,        // the start rotation
                ePose.so3,        // the end rotation
                optOption,        // the optimization option
                weight            // the weight
            );
        }
        auto sum = estimator->Solve(_ceresOption, _priori);
        spdlog::info("here is the summary:\n{}\n", sum.BriefReport());
    }

    /**
     * We then transform the initialized SO3 spline to the world coordinate system (grid pattern
     * coordinate system), which can be achieved by using the least-squares method to solve a pose
     * from the origin of the SO3 spline to the world coordinate system
     */
    spdlog::info(
        "obtain rotation bias between the grid coordinate system and the so3 spline coordinate "
        "system to transform the initialized SO3 spline to the world frame...");
#if 0
    auto estimator = Estimator::Create(_parMgr);
    Sophus::SO3d SO3_Br0ToW;
    for (const auto& [topic, poseVec] : _camPoses) {
        const double TO_CjToBr = _parMgr->TEMPORAL.TO_CjToBr.at(topic);
        const double weight = Configor::DataStream::EventTopics.at(topic).Weight;

        for (const auto& pose : poseVec) {
            if (pose.timeStamp + TO_CjToBr < st || pose.timeStamp + TO_CjToBr > et) {
                continue;
            }
            estimator->AddSo3SplineAlignToWorldConstraint(_fullSo3Spline, &SO3_Br0ToW, topic,
                                                          pose.timeStamp, pose.so3, OptOption::NONE,
                                                          weight);
        }
    }
    auto sum = estimator->Solve(_ceresOption);
    estimator = nullptr;
    spdlog::info("here is the summary:\n{}\n", sum.BriefReport());

    // transform the initialized SO3 spline to the world coordinate system
    for (int i = 0; i < static_cast<int>(_fullSo3Spline.GetKnots().size()); ++i) {
        _fullSo3Spline.GetKnot(i) =
            SO3_Br0ToW * _fullSo3Spline.GetKnot(i) /*from {Br(t)} to {Br0}*/;
    }
    {
        _viewer->ClearViewer();
        std::vector<ns_viewer::Entity::Ptr> entities;
        for (double t = st; t < et;) {
            if (!_fullSo3Spline.TimeStampInRange(t)) {
                t += 0.005;
                continue;
            }

            Sophus::SO3d so3 = _fullSo3Spline.Evaluate(t);
            // coordinate
            entities.push_back(ns_viewer::Coordinate::Create(
                ns_viewer::Posed(so3.matrix(), Eigen::Vector3d::Zero()).cast<float>(), 0.5f));
            t += 0.005;
        }
        _viewer->AddEntityLocal(entities);
        std::cin.get();
    }
#else
    auto estimator = Estimator::Create(_parMgr);
    for (const auto& [topic, poseVec] : _camPoses) {
        const double TO_CjToBr = _parMgr->TEMPORAL.TO_CjToBr.at(topic);
        const auto& SO3_CjToBr = _parMgr->EXTRI.SO3_CjToBr.at(topic);

        for (const auto& pose : poseVec) {
            if (!_fullSo3Spline.TimeStampInRange(pose.timeStamp + TO_CjToBr)) {
                continue;
            }
            Sophus::SO3d SO3_BrToW = pose.so3 /*from camera to world*/ * SO3_CjToBr.inverse();
            estimator->AddSo3Constraint(_fullSo3Spline, pose.timeStamp + TO_CjToBr, SO3_BrToW,
                                        OptOption::OPT_SO3_SPLINE, 10.0);
        }
    }
    AddGyroFactorToFullSo3Spline(estimator, Configor::DataStream::RefIMUTopic,
                                 OptOption::OPT_SO3_SPLINE, 0.1 /*weight*/, 100 /*down sampling*/);

    auto sum = estimator->Solve(_ceresOption, _priori);
    spdlog::info("here is the summary:\n{}\n", sum.BriefReport());
#endif

    /**
     * the gravity vector would be recovered in this stage, for better converage performance, we
     * assign the gravity roughly, f = a - g, g = a - f
     */
    Eigen::Vector3d firRefAcce = _imuMes.at(Configor::DataStream::RefIMUTopic).front()->GetAcce();
    // g = gDir * gNorm, where gDir = normalize(a - f), by assume the acceleration is zero
    _parMgr->GRAVITY = -firRefAcce.normalized() * Configor::Prior::GravityNorm;
    spdlog::info("rough assigned gravity in world frame: ['{:.3f}', '{:.3f}', '{:.3f}']",
                 _parMgr->GRAVITY(0), _parMgr->GRAVITY(1), _parMgr->GRAVITY(2));

    spdlog::info(
        "perform event-inertial alignment to recover event-inertial extrinsic translations and "
        "refine the world-frame gravity...");
    estimator = Estimator::Create(_parMgr);
    /**
     * we do not optimization the already initialized extrinsic rotations here
     */
    auto optOption = OptOption::OPT_POS_CjInBr |  // camera extrinsic translations
                     OptOption::OPT_POS_BiInBr |  // imu extrinsic translations
                     OptOption::OPT_GRAVITY;      // gravity

    static constexpr double MIN_ALIGN_TIME = 1E-3 /* 0.001 sed */;
    static constexpr double MAX_ALIGN_TIME = 0.5 /* 1.0 sed */;

    // event-inertial alignment
    std::map<std::string, std::vector<Eigen::Vector3d>> linVelSeqCm;
    for (const auto& [topic, poseVec] : _camPoses) {
        linVelSeqCm[topic] = std::vector<Eigen::Vector3d>(poseVec.size(), Eigen::Vector3d::Zero());
        auto& curCamLinVelSeq = linVelSeqCm.at(topic);
        const double weight = Configor::DataStream::EventTopics.at(topic).Weight;
        const double TO_CjToBr = _parMgr->TEMPORAL.TO_CjToBr.at(topic);
        const auto& imuFrames = _imuMes.at(Configor::DataStream::RefIMUTopic);

        spdlog::info("add visual-inertial alignment factors for '{}' and '{}', align step: {}",
                     topic, Configor::DataStream::RefIMUTopic, ALIGN_STEP);
        int count = 0;
        for (int i = 0; i < static_cast<int>(poseVec.size()) - ALIGN_STEP; ++i) {
            const auto& sPose = poseVec.at(i);
            const auto& ePose = poseVec.at(i + ALIGN_STEP);

            if (sPose.timeStamp + TO_CjToBr < st || ePose.timeStamp + TO_CjToBr > et) {
                continue;
            }

            if (ePose.timeStamp - sPose.timeStamp < MIN_ALIGN_TIME ||
                ePose.timeStamp - sPose.timeStamp > MAX_ALIGN_TIME) {
                continue;
            }

            estimator->AddEventInertialAlignment(
                _fullSo3Spline,
                imuFrames,                            // the imu frames
                topic,                                // the ros topic of the camera
                Configor::DataStream::RefIMUTopic,    // the ros topic of the imu
                sPose,                                // the start pose
                ePose,                                // the end pose
                &curCamLinVelSeq.at(i),               // the start velocity (to be estimated)
                &curCamLinVelSeq.at(i + ALIGN_STEP),  // the end velocity (to be estimated)
                optOption,                            // the optimize option
                weight);                              // the weigh
            ++count;
        }
        spdlog::info("constraint count of event-inertial alignment for '{}' and '{}': {}", topic,
                     Configor::DataStream::RefIMUTopic, count);
    }

    // inertial alignment (only when more than or equal to 2 num IMUs are involved)
    constexpr double dt = DESIRED_TIME_INTERVAL;
    std::vector<Eigen::Vector3d> linVelSeqBr(std::floor((et - st) / dt), Eigen::Vector3d::Zero());
    if (Configor::DataStream::IMUTopics.size() >= 2) {
        for (const auto& [topic, frames] : _imuMes) {
            spdlog::info("add inertial alignment factors for '{}'...", topic);
            int count = 0;
            for (int i = 0; i < static_cast<int>(linVelSeqBr.size()) - 1; ++i) {
                int sIdx = i, eIdx = i + 1;
                double sTimeByBr = sIdx * dt + st, eTimeByBr = eIdx * dt + st;
                Eigen::Vector3d *sVel = &linVelSeqBr.at(sIdx), *eVel = &linVelSeqBr.at(eIdx);

                estimator->AddInertialAlignment(
                    _fullSo3Spline,  // so3 spline
                    frames,          // imu frames
                    topic,           // the ros topic of this imu
                    sTimeByBr,       // the start time stamped by the reference imu
                    eTimeByBr,       // the end time stamped by the reference imu
                    sVel,            // the start velocity (to be estimated)
                    eVel,            // the end velocity (to be estimated)
                    optOption,       // the optimize option
                    Configor::DataStream::IMUTopics.at(topic).AcceWeight);
                ++count;
            }
            spdlog::info("constraint count of inertial alignment for '{}' and '{}': {}", topic,
                         Configor::DataStream::RefIMUTopic, count);
        }
    }

    // fix spatiotemporal parameters of reference sensor
    // make this problem full rank
    estimator->SetIMUParamsConstant(Configor::DataStream::RefIMUTopic);

    sum = estimator->Solve(_ceresOption, _priori);
    estimator = nullptr;
    spdlog::info("here is the summary:\n{}\n", sum.BriefReport());
}

}  // namespace ns_ekalibr