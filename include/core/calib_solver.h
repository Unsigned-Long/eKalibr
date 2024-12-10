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

#ifndef CALIB_SOLVER_H
#define CALIB_SOLVER_H

#include "memory"
#include "ceres/ceres.h"
#include "tiny-viewer/core/pose.hpp"

namespace ns_ekalibr {
class Viewer;
using ViewerPtr = std::shared_ptr<Viewer>;
struct EventArray;
using EventArrayPtr = std::shared_ptr<EventArray>;

class CalibSolver {
public:
    using Ptr = std::shared_ptr<CalibSolver>;

protected:
    // options used for ceres-related optimization
    ceres::Solver::Options _ceresOption;
    // viewer used to visualize entities in calibration
    ViewerPtr _viewer;
    // indicates whether the solving is finished
    bool _solveFinished;

    std::map<std::string, std::vector<EventArrayPtr>> _evMes;
    // start time, end time
    std::pair<double, double> _evDataRawTimestamp;
    std::pair<double, double> _evDataAlignedTimestamp;

    ns_viewer::Posef _viewCamPose;

public:
    CalibSolver();

    static Ptr Create();

    virtual ~CalibSolver();

    void Process();

protected:
    void LoadEventData();

    void OutputDataStatus() const;

private:
    // remove the head data according to the pred
    void EraseSeqHeadData(std::vector<EventArrayPtr> &seq,
                          std::function<bool(const EventArrayPtr &)> pred,
                          const std::string &errorMsg) const;

    // remove the tail data according to the pred
    void EraseSeqTailData(std::vector<EventArrayPtr> &seq,
                          std::function<bool(const EventArrayPtr &)> pred,
                          const std::string &errorMsg) const;
};
}  // namespace ns_ekalibr

#endif  // CALIB_SOLVER_H
