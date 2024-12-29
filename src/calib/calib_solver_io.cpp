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

#include "calib/calib_solver_io.h"
#include "calib/calib_solver.h"
#include "spdlog/spdlog.h"
#include "config/configor.h"
#include "util/utils.h"
#include "util/status.hpp"
#include "core/time_varying_ellipse.h"
#include "sensor/event.h"
#include <cereal/types/map.hpp>
#include <cereal/types/vector.hpp>
#include "cereal/types/list.hpp"
#include "cereal/types/polymorphic.hpp"
#include "cereal/types/utility.hpp"

namespace ns_ekalibr {

CalibSolverIO::CalibSolverIO(CalibSolver::Ptr solver)
    : _solver(std::move(solver)) {
    if (!_solver->_solveFinished) {
        spdlog::warn("calibration has not been performed!!! Do not try to save anything now!!!");
    }
}

CalibSolverIO::Ptr CalibSolverIO::Create(const CalibSolver::Ptr &solver) {
    return std::make_shared<CalibSolverIO>(solver);
}

void CalibSolverIO::SaveByProductsToDisk() const {
    // if (IsOptionWith(OutputOption::LiDARMaps, Configor::Preference::Outputs)) {
    //     this->SaveLiDARMaps();
    // }
}

bool CalibSolverIO::SaveRawEventsOfExtractedPatterns(const std::map<int, ExtractedCirclesVec> &data,
                                                     const std::string &filename,
                                                     double timeBias,
                                                     CerealArchiveType::Enum archiveType) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        return false;
    }
    auto archive = GetOutputArchiveVariant(file, archiveType);
    SerializeByOutputArchiveVariant(archive, archiveType, cereal::make_nvp("time_bias", timeBias),
                                    cereal::make_nvp("raw_ev_arys_of_circles_of_grid2d", data));
    return true;
}

std::map<int, CalibSolverIO::ExtractedCirclesVec> CalibSolverIO::LoadRawEventsOfExtractedPatterns(
    const std::string &filename, double newTimeBias, CerealArchiveType::Enum archiveType) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        return {};
    }
    std::map<int, ExtractedCirclesVec> rawEvsOfPattern;
    double time_bias;
    try {
        auto archive = GetInputArchiveVariant(file, archiveType);
        SerializeByInputArchiveVariant(
            archive, archiveType, cereal::make_nvp("time_bias", time_bias),
            cereal::make_nvp("raw_ev_arys_of_circles_of_grid2d", rawEvsOfPattern));
    } catch (const cereal::Exception &exception) {
        throw Status(
            Status::CRITICAL,
            "Can not load 'map<int, ExtractedCirclesVec>' file '{}' into eKalibr using cereal!!! "
            "Detailed cereal exception information: \n'{}'",
            filename, exception.what());
    }
    for (auto &[id, circles] : rawEvsOfPattern) {
        for (auto &[tvCircles, rawEvs] : circles) {
            tvCircles->st = tvCircles->st + time_bias - newTimeBias;
            tvCircles->et = tvCircles->et + time_bias - newTimeBias;
            /**
             * time-varying function should be modified!!!
             * v(t) = a * t + b
             * v(t - dt) = a * t + (-a * dt + b)
             * attention: not v(t + dt) = a * t + (a * dt + b)
             */
            tvCircles->cx(1) = -tvCircles->cx(0) * (time_bias - newTimeBias) + tvCircles->cx(1);
            tvCircles->cy(1) = -tvCircles->cy(0) * (time_bias - newTimeBias) + tvCircles->cy(1);
            tvCircles->mx(1) = -tvCircles->mx(0) * (time_bias - newTimeBias) + tvCircles->mx(1);
            tvCircles->my(1) = -tvCircles->my(0) * (time_bias - newTimeBias) + tvCircles->my(1);

            for (const auto &ev : rawEvs->GetEvents()) {
                ev->SetTimestamp(ev->GetTimestamp() + time_bias - newTimeBias);
            }
        }
    }
    return rawEvsOfPattern;
}

std::pair<std::string, std::string> CalibSolverIO::GetDiskPathOfExtractedGridPatterns(
    const std::string &topic) {
    const std::string dir = Configor::DataStream::OutputPath + "/" + topic;
    if (!TryCreatePath(dir)) {
        spdlog::info(
            "find directory '{}' to save/load extracted circle grid patterns of '{}' failed!!!",
            dir, topic);
        return {};
    }
    const auto &e = Configor::Preference::FileExtension.at(Configor::Preference::OutputDataFormat);
    return {dir + "/patterns" + e, dir + "/patterns_raw_evs.bin"};
}

std::string CalibSolverIO::GetDiskPathOfOpenCVIntrinsicCalibRes(const std::string &topic) {
    const std::string dir = Configor::DataStream::OutputPath + "/" + topic;
    if (!TryCreatePath(dir)) {
        spdlog::info(
            "find directory '{}' to save/load extracted circle grid patterns of '{}' failed!!!",
            dir, topic);
        return {};
    }
    const auto &e = Configor::Preference::FileExtension.at(Configor::Preference::OutputDataFormat);
    return dir + "/intrinsics" + e;
}
}  // namespace ns_ekalibr