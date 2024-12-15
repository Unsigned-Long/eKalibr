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
#include "core/estimator.h"
#include "core/calib_param_mgr.h"

namespace ns_ekalibr {
void CalibSolver::AddGyroFactorToFullSo3Spline(const Estimator::Ptr &estimator,
                                               const std::string &imuTopic,
                                               Estimator::Opt option,
                                               const std::optional<double> &w) const {
    auto weight = w == std::nullopt ? Configor::DataStream::IMUTopics.at(imuTopic).AcceWeight : *w;

    for (const auto &frame : _imuMes.at(imuTopic)) {
        estimator->AddIMUGyroMeasurement(_fullSo3Spline, frame, imuTopic, option, weight);
    }
}

void CalibSolver::AddGyroFactorToSplineSegments(const EstimatorPtr &estimator,
                                                const std::string &imuTopic,
                                                OptOption option,
                                                const std::optional<double> &w) const {
    auto weight = w == std::nullopt ? Configor::DataStream::IMUTopics.at(imuTopic).AcceWeight : *w;
    const auto &To_BiToBr = _parMgr->TEMPORAL.TO_BiToBr.at(imuTopic);
    for (const auto &frame : _imuMes.at(imuTopic)) {
        auto idx = this->IsTimeInValidSegment(frame->GetTimestamp() + To_BiToBr);
        if (idx < 0 || idx >= static_cast<int>(_splineSegments.size())) {
            continue;
        }
        estimator->AddIMUGyroMeasurement(_splineSegments.at(idx).first, frame, imuTopic, option,
                                         weight);
    }
}

void CalibSolver::AddAcceFactorToSplineSegments(const EstimatorPtr &estimator,
                                                const std::string &imuTopic,
                                                OptOption option,
                                                const std::optional<double> &w) const {
    auto weight = w == std::nullopt ? Configor::DataStream::IMUTopics.at(imuTopic).AcceWeight : *w;
    const auto &To_BiToBr = _parMgr->TEMPORAL.TO_BiToBr.at(imuTopic);
    for (const auto &frame : _imuMes.at(imuTopic)) {
        auto idx = this->IsTimeInValidSegment(frame->GetTimestamp() + To_BiToBr);
        if (idx < 0 || idx >= static_cast<int>(_splineSegments.size())) {
            continue;
        }
        estimator->AddIMUAcceMeasurement(_splineSegments.at(idx).first,
                                         _splineSegments.at(idx).second, frame, imuTopic, option,
                                         weight);
    }
}
}  // namespace ns_ekalibr