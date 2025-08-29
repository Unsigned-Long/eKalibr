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
#include "calib/estimator.h"
#include "spdlog/spdlog.h"
#include "calib/calib_param_mgr.h"
#include <calib/cross_correlation.h>

namespace ns_ekalibr {

void CalibSolver::InitSo3Spline() const {
    /**
     * this function would initialize the rotation spline, as well as the extrinsic rotations and
     * time offsets between multiple imus, if they are integrated
     */
    spdlog::info("fitting rotation (so3) B-spline...");

    /**
     * here we recover the so3 spline first using only the angular velocities from the reference
     * IMU, then estimates other quantities by involving angular velocity measurements from other
     * IMUs. For better readability, we could also optimize them together as follows.
     * ----------------------------------------------------------------
     * auto estimator = Estimator::Create(_splines, _parMagr);
     * auto optOption = OptOption::OPT_SO3_BiToBr;
     * if (Configor::Prior::OptTemporalParams) {
     *     optOption |= OptOption::OPT_TO_BiToBr;
     * }
     * for (const auto &[topic, _] : Configor::DataStream::IMUTopics) {
     *     this->AddGyroFactor(estimator, topic, optOption);
     * }
     * estimator->SetRefIMUParamsConstant();
     * auto sum = estimator->Solve(_ceresOption, this->_priori);
     * spdlog::info("here is the summary:\n{}\n", sum.BriefReport());
     * ----------------------------------------------------------------
     */

    auto estimator = Estimator::Create(_parMgr);
    // we initialize the rotation spline first use only the measurements from the reference imu
    this->AddGyroFactorToFullSo3Spline(estimator, Configor::DataStream::RefIMUTopic,
                                       OptOption::OPT_SO3_SPLINE, 0.1 /*weight*/,
                                       100 /*down sampling*/);
    auto sum = estimator->Solve(_ceresOption, _priori);
    spdlog::info("here is the summary:\n{}\n", sum.BriefReport());

    if (Configor::DataStream::IMUTopics.size() > 1) {
        // recover the time offsets using cross correlation max
        if (Configor::Prior::OptTemporalParams) {
            for (const auto &[topic, _] : Configor::DataStream::IMUTopics) {
                if (topic == Configor::DataStream::RefIMUTopic) {
                    continue;
                }
                spdlog::info(
                    "estimating time offset between '{}' and '{}' using cross correlation...",
                    Configor::DataStream::RefIMUTopic, topic);

                double dt = TemporalCrossCorrelation::AngularVelocityAlignment(
                    _imuMes.at(Configor::DataStream::RefIMUTopic), _imuMes.at(topic));
                spdlog::info("estimated time offset is dt = {:.3f} (sec)", dt);

                // // todo: slow!!! need to refine!!!
                // double dt = TemporalCrossCorrelation::AngularVelocityAlignmentV2(
                //     _imuMes.at(Configor::DataStream::RefIMUTopic), _imuMes.at(topic));

                spdlog::info("estimated time offset is dt = {:.3f} (sec)", dt);
                _parMgr->TEMPORAL.TO_BiToBr[topic] = dt;
            }
        }

        // if multiple imus involved, we continue to recover extrinsic rotations and time offsets
        spdlog::info("recovering extrinsic rotations and time offsets between multiple imus...");
        estimator = Estimator::Create(_parMgr);
        auto optOption = OptOption::OPT_SO3_BiToBr;
        if (Configor::Prior::OptTemporalParams) {
            optOption |= OptOption::OPT_TO_BiToBr;
        }
        for (const auto &[topic, _] : Configor::DataStream::IMUTopics) {
            if (topic == Configor::DataStream::RefIMUTopic) {
                continue;
            }
            this->AddGyroFactorToFullSo3Spline(estimator, topic, optOption, 0.1 /*weight*/,
                                               100 /*down sampling*/);
        }
        // make this problem full rank
        estimator->SetIMUParamsConstant(Configor::DataStream::RefIMUTopic);

        sum = estimator->Solve(_ceresOption, _priori);
        spdlog::info("here is the summary:\n{}\n", sum.BriefReport());
    }
}

void CalibSolver::InitSo3SplineSegments() {
    // fitting so3 segments
    spdlog::info("fitting so3 part of spline segments...");
    auto estimator = Estimator::Create(_parMgr);
#if 0
    for (const auto &[topic, poseVec] : _camPoses) {
        const double TO_CjToBr = _parMgr->TEMPORAL.TO_CjToBr.at(topic);
        const auto &SO3_CjToBr = _parMgr->EXTRI.SO3_CjToBr.at(topic);

        for (const auto &pose : poseVec) {
            auto idx = this->IsTimeInValidSegment(pose.timeStamp + TO_CjToBr);
            if (idx == std::nullopt) {
                continue;
            }
            Sophus::SO3d SO3_BrToW = pose.so3 /*from camera to world*/ * SO3_CjToBr.inverse();
            estimator->AddSo3Constraint(_splineSegments.at(*idx).first, pose.timeStamp + TO_CjToBr,
                                        SO3_BrToW, OptOption::OPT_SO3_SPLINE, 10.0);
        }
    }
    AddGyroFactorToSplineSegments(estimator, Configor::DataStream::RefIMUTopic,
                                  OptOption::OPT_SO3_SPLINE, 0.1, /*weight*/
                                  100 /*down sampling rate*/);
#else
    double st = _fullSo3Spline.MinTime(), et = _fullSo3Spline.MaxTime(),
           dt = _splineSegments.front().first.GetTimeInterval() * 0.1;
    for (double t = st; t < et; t += dt) {
        if (!_fullSo3Spline.TimeStampInRange(t)) {
            continue;
        }
        auto idx = this->IsTimeInValidSegment(t);
        if (idx == std::nullopt) {
            continue;
        }
        estimator->AddSo3Constraint(_splineSegments.at(*idx).first, t, _fullSo3Spline.Evaluate(t),
                                    OptOption::OPT_SO3_SPLINE, 10.0);
    }
    for (auto &[so3Spline, posSpline] : _splineSegments) {
        estimator->AddRegularizationL2Constraint(so3Spline, OptOption::OPT_SO3_SPLINE, 1E-3);
    }
#endif
    auto sum = estimator->Solve(_ceresOption, _priori);
    spdlog::info("here is the summary:\n{}\n", sum.BriefReport());
}

}  // namespace ns_ekalibr