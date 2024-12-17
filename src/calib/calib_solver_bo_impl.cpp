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
#include <magic_enum_flags.hpp>
#include <core/circle_grid.h>
#include <calib/estimator.h>
#include <factor/visual_projection_factor.hpp>
#include <spdlog/spdlog.h>

namespace ns_ekalibr {
void CalibSolver::BatchOptimizations() {
    std::array<OptOption, 3> optionAry = {
        // the first one
        OptOption::OPT_SO3_SPLINE | OptOption::OPT_SCALE_SPLINE | OptOption::OPT_GRAVITY |
            OptOption::OPT_SO3_CjToBr | OptOption::OPT_POS_CjInBr | OptOption::OPT_TO_CjToBr,
        // the second one (append to last)
        OptOption::OPT_SO3_BiToBr | OptOption::OPT_POS_BiInBr | OptOption::OPT_TO_BiToBr |
            OptOption::OPT_ACCE_BIAS | OptOption::OPT_GYRO_BIAS,
        // the third one (append to last)
        OptOption::OPT_ACCE_MAP_COEFF | OptOption::OPT_GYRO_MAP_COEFF | OptOption::OPT_SO3_AtoG};

    std::vector options(optionAry.size(), OptOption::NONE);
    for (int i = 0; i < static_cast<int>(optionAry.size()); ++i) {
        options.at(i) = optionAry.at(i);
        // append
        if (i != 0) {
            options.at(i) |= options.at(i - 1);
        }
    }

    // create visual projection pairs
    for (const auto& [topic, patterns] : _extractedPatterns) {
        const auto& grid3d = patterns->GetGrid3d();
        const auto& grid2dVec = patterns->GetGrid2d();

        auto& pairs = _evProjPairs[topic];
        pairs.reserve(grid2dVec.size() * grid3d->points.size());

        for (const auto& grid2d : grid2dVec) {
            for (int i = 0; i < static_cast<int>(grid2d->centers.size()); ++i) {
                const auto& center = grid2d->centers.at(i);
                const Eigen::Vector2d pixel(center.x, center.y);

                const auto& point3d = grid3d->points.at(i);
                const Eigen::Vector3d point(point3d.x, point3d.y, point3d.z);

                pairs.push_back(VisualProjectionPair::Create(grid2d->timestamp, point, pixel));
            }
        }
        spdlog::info("create {} (={}x{}) visual projection pairs for camera '{}'", pairs.size(),
                     grid2dVec.size(), grid3d->points.size(), topic);
    }

    for (int i = 0; i < static_cast<int>(options.size()); ++i) {
        const auto& option = options.at(i);
        std::stringstream stringStream;
        stringStream << magic_enum::enum_flags_name(option);
        spdlog::info("performing the '{}'-th batch optimization, option:\n{}", i,
                     stringStream.str());

        auto estimator = Estimator::Create(_parMgr);

        for (const auto& [topic, _] : Configor::DataStream::IMUTopics) {
            auto s = this->AddAcceFactorToSplineSegments(estimator, topic, option, {}, 100);
            spdlog::info("add '{}' 'IMUAcceFactor' for imu '{}'...", s, topic);

            s = this->AddGyroFactorToSplineSegments(estimator, topic, option, {}, 100);
            spdlog::info("add '{}' 'IMUGyroFactor' for imu '{}'...", s, topic);
        }

        for (const auto& [topic, _] : Configor::DataStream::EventTopics) {
            auto s = this->AddVisualProjPairsToSplineSegments(estimator, topic, option, {});
            spdlog::info("add '{}' 'VisualProjectionFactor' for camera '{}'...", s, topic);
        }
        // make this problem full rank
        estimator->SetRefIMUParamsConstant();
        auto sum = estimator->Solve(_ceresOption);
        spdlog::info("here is the summary:\n{}\n", sum.BriefReport());
    }
}

}  // namespace ns_ekalibr