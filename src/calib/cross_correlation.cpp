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

double TemporalCrossCorrelation::AngularVelAlignStableToDense(
    const std::vector<std::pair<double, Eigen::Vector3d>>& angVel1,
    const std::vector<std::pair<double, Eigen::Vector3d>>& angVel2) {
    std::vector<std::pair<double, Eigen::Vector3d>> angVel1Aligned, angVel2Aligned;
    FrequencyAlign<std::pair<double, Eigen::Vector3d>>(
        angVel1, angVel1Aligned, angVel2, angVel2Aligned,
        [](const std::pair<double, Eigen::Vector3d>& p) { return p.first; });

    int N = static_cast<int>(angVel1Aligned.size());
    if (N == 0 || static_cast<int>(angVel2Aligned.size()) != N) {
        throw EKalibrStatus(Status::ERROR,
                            "Angular velocity data size mismatch or empty in "
                            "[TemporalCrossCorrelation::AngularVelAlignStableFreq]");
    }

    double imu1Means = 0.0, imu2Mean = 0.0;
    std::vector<double> imu1Norms(N), imu2Norms(N);
    for (int i = 0; i < N; i++) {
        imu1Norms[i] = angVel1Aligned[i].second.norm();
        imu1Means += (imu1Norms[i] - imu1Means) / (i + 1);

        imu2Norms[i] = angVel2Aligned[i].second.norm();
        imu2Mean += (imu2Norms[i] - imu2Mean) / (i + 1);
    }

    double max_corr = -DBL_MAX;
    int best_lag = 0;
    for (int lag = -N + 1; lag < N; lag++) {
        double corr = 0.0;
        int cnt = 0;
        for (int i = 0; i < N; i++) {
            int j = i + lag;
            if (j < 0 || j >= N) continue;
            corr += (imu1Norms[i] - imu1Means) * (imu2Norms[j] - imu2Mean);
            cnt++;
        }
        if (cnt > 0 && corr > max_corr) {
            max_corr = corr;
            best_lag = -lag;
        }
    }
    double freq1 =
        static_cast<double>(N - 1) / (angVel1Aligned.back().first - angVel1Aligned.front().first);
    double freq2 =
        static_cast<double>(N - 1) / (angVel2Aligned.back().first - angVel2Aligned.front().first);

    double freqAvg = (freq1 + freq2) / 2.0;
    double time_lag = static_cast<double>(best_lag) / freqAvg;
    return time_lag;
}

double TemporalCrossCorrelation::AngularVelAlignSparseToDense(
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
    std::vector<double> normsHigh(angVelHigh->size()), normsLow(angVelLow->size());
    for (int i = 0; i < static_cast<int>(angVelHigh->size()); i++) {
        normsHigh[i] = (*angVelHigh)[i].second.norm();
        meanHigh += (normsHigh[i] - meanHigh) / (i + 1);
    }
    for (int i = 0; i < static_cast<int>(angVelLow->size()); i++) {
        normsLow[i] = (*angVelLow)[i].second.norm();
        meanLow += (normsLow[i] - meanLow) / (i + 1);
    }

    int N = static_cast<int>(angVelHigh->size());
    int M = static_cast<int>(angVelLow->size());
    double max_corr = -DBL_MAX;
    int best_lag = 0;
    for (int lag = -N + 1; lag < N; lag++) {
        double corr = 0.0;
        int cnt = 0;
        int idx = 0;
        for (int j = 0; j < M; j++) {
            double ti = (*angVelLow)[j].first + lag * dtHigh;
            if (ti < angVelHigh->front().first || ti > angVelHigh->back().first) continue;

            while (idx < N && (*angVelHigh)[idx].first < ti) {
                idx++;
            }

            double interp;
            if (idx == 0)
                interp = normsHigh[0];
            else if (idx >= static_cast<int>(angVelHigh->size()))
                interp = normsHigh.back();
            else {
                double t0 = (*angVelHigh)[idx - 1].first;
                double t1 = (*angVelHigh)[idx].first;
                double w = (ti - t0) / (t1 - t0);
                interp = (1 - w) * normsHigh[idx - 1] + w * normsHigh[idx];
            }
            corr += (interp - meanHigh) * (normsLow[j] - meanLow);
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