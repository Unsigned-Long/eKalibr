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
                            "Frequency times must be strictly ordered in "
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