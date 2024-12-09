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
#include "sensor/event_rosbag_loader.h"
#include "config/configor.h"
#include "pangolin/display/display.h"
#include "viewer/viewer.h"

namespace ns_ekalibr {
CalibSolver::CalibSolver()
    : _viewer(Viewer::Create()),
      _solveFinished(false) {
    // organize the default solver option
    _ceresOption.minimizer_type = ceres::TRUST_REGION;
    _ceresOption.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    _ceresOption.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    _ceresOption.minimizer_progress_to_stdout = false;
}

CalibSolver::Ptr CalibSolver::Create() { return std::make_shared<CalibSolver>(); }

CalibSolver::~CalibSolver() {
    // solving is not performed or not finished as an exception is thrown
    if (!_solveFinished) {
        pangolin::QuitAll();
    }
    // solving is finished (when use 'pangolin::QuitAll()', the window not quit immediately)
    while (_viewer->IsActive()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void CalibSolver::Process() { _solveFinished = true; }
}  // namespace ns_ekalibr
