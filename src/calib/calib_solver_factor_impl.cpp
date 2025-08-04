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
#include "calib/calib_param_mgr.h"
#include <factor/visual_projection_factor.hpp>

namespace ns_ekalibr {
std::size_t CalibSolver::AddGyroFactorToFullSo3Spline(const Estimator::Ptr &estimator,
                                                      const std::string &imuTopic,
                                                      Estimator::Opt option,
                                                      const std::optional<double> &w,
                                                      const std::optional<double> &dsRate) const {
    auto weight =
        w == std::nullopt
            ? Configor::DataStream::IMUTopics.at(imuTopic).GyroWeight(_imuFrequency.at(imuTopic))
            : *w;
    std::size_t index = 0, count = 0;
    std::size_t pick = 1UL;
    if (dsRate == std::nullopt) {
        const double dt = _dataAlignedTimestamp.second - _dataAlignedTimestamp.first;
        const double freq = static_cast<double>(_imuMes.at(imuTopic).size()) / dt;
        pick = std::max(pick, static_cast<std::size_t>(freq / *dsRate));
    }
    for (const auto &frame : _imuMes.at(imuTopic)) {
        if (++index % pick != 0) {
            continue;
        }
        estimator->AddIMUGyroMeasurement(_fullSo3Spline, frame, imuTopic, option, weight);
        ++count;
    }
    return count;
}

std::size_t CalibSolver::AddGyroFactorToSplineSegments(const EstimatorPtr &estimator,
                                                       const std::string &imuTopic,
                                                       OptOption option,
                                                       const std::optional<double> &w,
                                                       const std::optional<double> &dsRate) const {
    auto weight =
        w == std::nullopt
            ? Configor::DataStream::IMUTopics.at(imuTopic).GyroWeight(_imuFrequency.at(imuTopic))
            : *w;
    const auto &To_BiToBr = _parMgr->TEMPORAL.TO_BiToBr.at(imuTopic);
    std::size_t index = 0, count = 0;
    std::size_t pick = 1UL;
    if (dsRate != std::nullopt) {
        const double dt = _dataAlignedTimestamp.second - _dataAlignedTimestamp.first;
        const double freq = static_cast<double>(_imuMes.at(imuTopic).size()) / dt;
        pick = std::max(pick, static_cast<std::size_t>(freq / *dsRate));
    }
    for (const auto &frame : _imuMes.at(imuTopic)) {
        if (++index % pick != 0) {
            continue;
        }

        auto idx = this->IsTimeInValidSegment(frame->GetTimestamp() + To_BiToBr);
        if (idx == std::nullopt) {
            continue;
        }
        estimator->AddIMUGyroMeasurement(_splineSegments.at(*idx).first, frame, imuTopic, option,
                                         weight);
        ++count;
    }
    return count;
}

std::size_t CalibSolver::AddAcceFactorToSplineSegments(const EstimatorPtr &estimator,
                                                       const std::string &imuTopic,
                                                       OptOption option,
                                                       const std::optional<double> &w,
                                                       const std::optional<double> &dsRate) const {
    auto weight =
        w == std::nullopt
            ? Configor::DataStream::IMUTopics.at(imuTopic).AcceWeight(_imuFrequency.at(imuTopic))
            : *w;
    const auto &To_BiToBr = _parMgr->TEMPORAL.TO_BiToBr.at(imuTopic);
    std::size_t index = 0, count = 0;
    std::size_t pick = 1UL;
    if (dsRate != std::nullopt) {
        const double dt = _dataAlignedTimestamp.second - _dataAlignedTimestamp.first;
        const double freq = static_cast<double>(_imuMes.at(imuTopic).size()) / dt;
        pick = std::max(pick, static_cast<std::size_t>(freq / *dsRate));
    }
    for (const auto &frame : _imuMes.at(imuTopic)) {
        if (++index % pick != 0) {
            continue;
        }

        auto idx = this->IsTimeInValidSegment(frame->GetTimestamp() + To_BiToBr);
        if (idx == std::nullopt) {
            continue;
        }
        estimator->AddIMUAcceMeasurement(_splineSegments.at(*idx).first,
                                         _splineSegments.at(*idx).second, frame, imuTopic, option,
                                         weight);
        ++count;
    }
    return count;
}

std::size_t CalibSolver::AddVisualProjPairsSyncPointBasedToSplineSegments(
    const EstimatorPtr &estimator,
    const std::string &camTopic,
    OptOption option,
    const std::optional<double> &w) const {
    auto weight = w == std::nullopt ? Configor::Prior::EvCameraWeight() : *w;
    const auto &TO_CjToBr = _parMgr->TEMPORAL.TO_CjToBr.at(camTopic);
    std::size_t count = 0;

    for (const auto &pair : _evSyncPointProjPairs.at(camTopic)) {
        auto idx = this->IsTimeInValidSegment(pair->timestamp + TO_CjToBr);
        if (idx == std::nullopt) {
            continue;
        }
        estimator->AddVisualProjectionFactor(_splineSegments.at(*idx).first,
                                             _splineSegments.at(*idx).second, camTopic, pair,
                                             option, weight);
        ++count;
    }
    return count;
}

std::size_t CalibSolver::AddVisualProjPairsAsyncPointBasedToSplineSegments(
    const EstimatorPtr &estimator,
    const std::string &camTopic,
    OptOption option,
    const std::optional<double> &w) const {
    auto weight = w == std::nullopt ? Configor::Prior::EvCameraWeight() : *w;
    const auto &TO_CjToBr = _parMgr->TEMPORAL.TO_CjToBr.at(camTopic);
    std::size_t count = 0;

    for (const auto &pair : _evAsyncPointProjPairs.at(camTopic)) {
        auto idx = this->IsTimeInValidSegment(pair->timestamp + TO_CjToBr);
        if (idx == std::nullopt) {
            continue;
        }
        estimator->AddVisualProjectionFactor(_splineSegments.at(*idx).first,
                                             _splineSegments.at(*idx).second, camTopic, pair,
                                             option, weight);
        ++count;
    }
    return count;
}
}  // namespace ns_ekalibr