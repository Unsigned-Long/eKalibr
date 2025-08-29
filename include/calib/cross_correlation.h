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

#ifndef EKALIBR_CROSS_CORRELATION_H
#define EKALIBR_CROSS_CORRELATION_H

#include <utility>
#include <vector>
#include <memory>
#include <sensor/imu.hpp>

namespace ns_ekalibr {

class TemporalCrossCorrelation {
public:
    /**
     * @brief Estimate the time offset between two IMUs using cross-correlation of their angular
     * velocities.
     * @param imu1 the inertial data of the first IMU
     * @param imu2 the inertial data of the second IMU
     * @return the time offset from the second IMU to the first IMU, i.e., t_imu1 = t_imu2 + dt
     */
    static double AngularVelocityAlignment(const std::vector<IMUFramePtr>& imu1,
                                           const std::vector<IMUFramePtr>& imu2);

protected:
    template <class Type>
    static void FrequencyAlign(const std::vector<Type>& data1,
                               std::vector<Type>& data1Aligned,
                               const std::vector<Type>& data2,
                               std::vector<Type>& data2Aligned,
                               const std::function<double(const Type&)>& timeAccessor) {
        std::vector<double> times1(data1.size()), times2(data2.size());
        for (size_t i = 0; i < data1.size(); i++) {
            times1[i] = timeAccessor(data1[i]);
        }
        for (size_t i = 0; i < data2.size(); i++) {
            times2[i] = timeAccessor(data2[i]);
        }
        const double freq1 =
            static_cast<double>(times1.size() - 1) / (times1.back() - times1.front());
        const double freq2 =
            static_cast<double>(times2.size() - 1) / (times2.back() - times2.front());

        std::vector<size_t> data1AlignedIdxVec, data2AlignedIdxVec;

        if (freq1 > freq2) {
            // data1 has higher frequency
            FrequencyAlign(times1, data1AlignedIdxVec, times2, data2AlignedIdxVec);
        } else {
            // data2 has higher frequency
            FrequencyAlign(times2, data2AlignedIdxVec, times1, data1AlignedIdxVec);
        }
        assert(data1AlignedIdxVec.size() == data2AlignedIdxVec.size());

        data1Aligned.resize(data1AlignedIdxVec.size());
        for (size_t i = 0; i < data1AlignedIdxVec.size(); i++) {
            data1Aligned[i] = data1[data1AlignedIdxVec[i]];
        }
        data2Aligned.resize(data2AlignedIdxVec.size());
        for (size_t i = 0; i < data2AlignedIdxVec.size(); i++) {
            data2Aligned[i] = data2[data2AlignedIdxVec[i]];
        }
    }

    static void FrequencyAlign(const std::vector<double>& highFreqTimes,
                               std::vector<size_t>& highFreqIdxVec,
                               const std::vector<double>& lowFreqTimes,
                               std::vector<size_t>& lowFreqIdxVec);
};
}  // namespace ns_ekalibr

#endif  // EKALIBR_CROSS_CORRELATION_H
