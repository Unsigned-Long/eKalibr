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

#include "calib/cross_correlation.h"
#include "util/status.hpp"

namespace ns_ekalibr {

double TemporalCrossCorrelation::AngularVelocityAlignment(const std::vector<IMUFramePtr>& imu1,
                                                          const std::vector<IMUFramePtr>& imu2) {
    std::vector<IMUFramePtr> imu1Aligned, imu2Aligned;
    FrequencyAlign<IMUFramePtr>(imu1, imu1Aligned, imu2, imu2Aligned,
                                [](const IMUFramePtr& imu) { return imu->GetTimestamp(); });

    int N = static_cast<int>(imu1Aligned.size());
    if (N == 0 || static_cast<int>(imu2Aligned.size()) != N) {
        throw EKalibrStatus(Status::ERROR,
                            "IMU data size mismatch or empty in "
                            "[TemporalCrossCorrelation::AngularVelocityAlignment]");
    }

    double mean_imu1 = 0.0, mean_imu2 = 0.0;
    for (int i = 0; i < N; i++) {
        mean_imu1 += (imu1Aligned[i]->GetGyro().norm() - mean_imu1) / (i + 1);
        mean_imu2 += (imu2Aligned[i]->GetGyro().norm() - mean_imu2) / (i + 1);
    }

    double max_corr = -DBL_MAX;
    int best_lag = 0;
    for (int lag = -N + 1; lag < N; lag++) {
        double corr = 0.0;
        int cnt = 0;
        for (int i = 0; i < N; i++) {
            int j = i + lag;
            if (j < 0 || j >= N) continue;
            corr += (imu1Aligned[i]->GetGyro().norm() - mean_imu1) *
                    (imu2Aligned[j]->GetGyro().norm() - mean_imu2);
            cnt++;
        }
        if (cnt > 0 && corr > max_corr) {
            max_corr = corr;
            best_lag = -lag;
        }
    }
    double freqAvg =
        (static_cast<double>(N - 1) /
             (imu1Aligned.back()->GetTimestamp() - imu1Aligned.front()->GetTimestamp()) +
         static_cast<double>(N - 1) /
             (imu2Aligned.back()->GetTimestamp() - imu2Aligned.front()->GetTimestamp())) /
        2.0;
    double time_lag = static_cast<double>(best_lag) / freqAvg;
    return time_lag;
}

double TemporalCrossCorrelation::AngularVelocityAlignment(
    const std::vector<std::pair<double, Eigen::Vector3d>>& angVel1,
    const std::vector<std::pair<double, Eigen::Vector3d>>& angVel2) {
    bool isStrict =
        std::adjacent_find(angVel1.begin(), angVel1.end(),
                           [](const auto& a, const auto& b) { return a.first >= b.first; }) ==
            angVel1.end() &&
        std::adjacent_find(angVel2.begin(), angVel2.end(), [](const auto& a, const auto& b) {
            return a.first >= b.first;
        }) == angVel2.end();
    if (!isStrict) {
        throw EKalibrStatus(Status::ERROR,
                            "times must be strictly ordered in "
                            "[TemporalCrossCorrelation::AngularVelocityAlignment]");
    }
    const std::vector<std::pair<double, Eigen::Vector3d>>* angVelHigh = nullptr;
    const std::vector<std::pair<double, Eigen::Vector3d>>* angVelLow = nullptr;
    const double freq1 =
        static_cast<double>(angVel1.size() - 1) / (angVel1.back().first - angVel1.front().first);
    const double freq2 =
        static_cast<double>(angVel2.size() - 1) / (angVel2.back().first - angVel2.front().first);
    double dtHigh;
    if (freq1 < freq2) {
        angVelHigh = &angVel2;
        dtHigh = 1.0 / freq2;

        angVelLow = &angVel1;
    } else {
        angVelHigh = &angVel1;
        dtHigh = 1.0 / freq1;

        angVelLow = &angVel2;
    }

    double meanHigh = 0.0, meanLow = 0.0;
    for (int i = 0; i < static_cast<int>(angVelHigh->size()); i++) {
        meanHigh += ((*angVelHigh)[i].second.norm() - meanHigh) / (i + 1);
    }
    for (int i = 0; i < static_cast<int>(angVelLow->size()); i++) {
        meanLow += ((*angVelLow)[i].second.norm() - meanLow) / (i + 1);
    }

    int N = static_cast<int>(angVelHigh->size());
    int M = static_cast<int>(angVelLow->size());
    double max_corr = -DBL_MAX;
    int best_lag = 0;
    for (int lag = -N + 1; lag < N; lag++) {
        double corr = 0.0;
        int cnt = 0;
        for (int j = 0; j < M; j++) {
            double ti = (*angVelLow)[j].first + lag * dtHigh;
            if (ti < angVelHigh->front().first || ti > angVelHigh->back().first) continue;

            int idx = std::lower_bound(angVelHigh->begin(), angVelHigh->end(), ti,
                                       [](const auto& a, double t) { return a.first < t; }) -
                      angVelHigh->begin();
            Eigen::Vector3d interp;
            if (idx == 0)
                interp = (*angVelHigh)[0].second;
            else if (idx >= static_cast<int>(angVelHigh->size()))
                interp = angVelHigh->back().second;
            else {
                double t0 = (*angVelHigh)[idx - 1].first;
                double t1 = (*angVelHigh)[idx].first;
                double w = (ti - t0) / (t1 - t0);
                interp = (1 - w) * (*angVelHigh)[idx - 1].second + w * (*angVelHigh)[idx].second;
            }
            corr += (interp.norm() - meanHigh) * ((*angVelLow)[j].second.norm() - meanLow);
            cnt++;
        }
        if (cnt > 0 && corr > max_corr) {
            max_corr = corr;
            best_lag = lag;
        }
    }
    // from low to high
    double time_lag = static_cast<double>(best_lag) * dtHigh;
    if (freq1 < freq2) {
        // from freq1 to freq2
        time_lag *= -1.0;
    } else {
        // from freq2 to freq1
    }
    return time_lag;
}

double TemporalCrossCorrelation::AngularVelocityAlignmentV2(const std::vector<IMUFramePtr>& imu1,
                                                            const std::vector<IMUFramePtr>& imu2) {
    throw EKalibrStatus(Status::WARNING, "This method is deprecated and may be removed in future.");
    std::vector<std::pair<double, Eigen::Vector3d>> angVel1(imu1.size()), angVel2(imu2.size());
    for (size_t i = 0; i < imu1.size(); i++) {
        angVel1[i] = {imu1[i]->GetTimestamp(), imu1[i]->GetGyro()};
    }
    for (size_t i = 0; i < imu2.size(); i++) {
        angVel2[i] = {imu2[i]->GetTimestamp(), imu2[i]->GetGyro()};
    }
    return AngularVelocityAlignment(angVel1, angVel2);
}

void TemporalCrossCorrelation::FrequencyAlign(const std::vector<double>& highFreqTimes,
                                              std::vector<size_t>& highFreqIdxVec,
                                              const std::vector<double>& lowFreqTimes,
                                              std::vector<size_t>& lowFreqIdxVec) {
    bool isStrict =
        std::adjacent_find(highFreqTimes.begin(), highFreqTimes.end(),
                           [](double a, double b) { return a >= b; }) == highFreqTimes.end() &&
        std::adjacent_find(lowFreqTimes.begin(), lowFreqTimes.end(),
                           [](double a, double b) { return a >= b; }) == lowFreqTimes.end();
    if (!isStrict) {
        throw EKalibrStatus(Status::ERROR,
                            "times must be strictly ordered in "
                            "[TemporalCrossCorrelation::FrequencyAlign]");
    }

    highFreqIdxVec.clear();
    lowFreqIdxVec.clear();

    if (highFreqTimes.empty() || lowFreqTimes.empty()) return;

    size_t i = 0;
    for (size_t j = 0; j < lowFreqTimes.size(); j++) {
        double t2 = lowFreqTimes[j];

        while (i + 1 < highFreqTimes.size() &&
               std::abs(highFreqTimes[i + 1] - t2) < std::abs(highFreqTimes[i] - t2)) {
            i++;
        }

        highFreqIdxVec.push_back(i);
        lowFreqIdxVec.push_back(j);
    }
}
}  // namespace ns_ekalibr