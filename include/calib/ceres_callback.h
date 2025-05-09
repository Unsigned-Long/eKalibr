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

#ifndef CERES_CALLBACK_H
#define CERES_CALLBACK_H

#include "ceres/iteration_callback.h"
#include "memory"
#include "fstream"

namespace ns_ekalibr {
class CalibParamManager;
using CalibParamManagerPtr = std::shared_ptr<CalibParamManager>;
class Viewer;
using ViewerPtr = std::shared_ptr<Viewer>;
class CircleGrid3D;
using CircleGrid3DPtr = std::shared_ptr<CircleGrid3D>;

struct CeresDebugCallBack : public ceres::IterationCallback {
private:
    CalibParamManagerPtr _parMagr;
    const std::string _outputDir;
    std::ofstream _iterInfoFile;
    int _idx;

public:
    explicit CeresDebugCallBack(CalibParamManagerPtr calibParamManager);

    ~CeresDebugCallBack() override;

    ceres::CallbackReturnType operator()(const ceres::IterationSummary &summary) override;
};

struct CeresViewerCallBack : public ceres::IterationCallback {
private:
    ViewerPtr _viewer;

public:
    explicit CeresViewerCallBack(ViewerPtr viewer);

    ceres::CallbackReturnType operator()(const ceres::IterationSummary &summary) override;
};
}  // namespace ns_ekalibr

#endif  // CERES_CALLBACK_H
