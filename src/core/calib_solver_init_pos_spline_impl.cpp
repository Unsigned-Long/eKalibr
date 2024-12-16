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
#include "spdlog/spdlog.h"
#include <core/calib_param_mgr.h>

namespace ns_ekalibr {

void CalibSolver::InitPosSpline() const {
    spdlog::info("performing position spline recovery...");

    /**
     * we throw the head and tail data as the rotations from the fitted SO3 Spline in that range are
     * poor
     */
    auto estimator = Estimator::Create(_parMgr);
    auto optOption = OptOption::OPT_SCALE_SPLINE;

    // add camera position constraints
    for (const auto& [topic, poseVec] : _camPoses) {
        const double TO_CjToBr = _parMgr->TEMPORAL.TO_CjToBr.at(topic);
        const auto& SE3_BrToCj = _parMgr->EXTRI.SE3_CjToBr(topic).inverse();

        for (const auto& pose : poseVec) {
            const double timeByBr = pose.timeStamp + TO_CjToBr;

            auto idx = IsTimeInValidSegment(timeByBr);
            if (idx < 0 || idx >= static_cast<int>(_validTimeSegments.size())) {
                continue;
            }

            const Sophus::SE3d SE3_BrToW = pose.se3() * SE3_BrToCj;
            estimator->AddPositionConstraint(_splineSegments.at(idx).second, timeByBr,
                                             SE3_BrToW.translation(), optOption, 10.0);
        }
    }

    AddAcceFactorToSplineSegments(estimator, Configor::DataStream::RefIMUTopic, optOption,
                                  0.1, /*weight*/
                                  100 /*down sampling rate*/);

    auto sum = estimator->Solve(_ceresOption);
    spdlog::info("here is the summary:\n{}\n", sum.BriefReport());
}
}  // namespace ns_ekalibr