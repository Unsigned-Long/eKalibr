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
#include "core/circle_grid.h"
#include "calib/calib_param_mgr.h"
#include "veta/camera/pinhole.h"
#include "util/utils_tpl.hpp"
#include "core/circle_extractor.h"
#include <opencv2/imgcodecs.hpp>
#include <pangolin/display/display.h>

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
    if (IsOptionWith(OutputOption::VisualReprojError, Configor::Preference::Outputs)) {
        this->SaveVisualReprojError();
    }
}

void CalibSolverIO::SaveVisualIntrinsics() const {
    for (const auto &[topic, intri] : _solver->_parMgr->INTRI.Camera) {
        const std::string filename = Configor::DataStream::OutputPath + '/' +
                                     TopicConvertToFilename(topic) + ".intri" +
                                     Configor::GetFormatExtension();
        spdlog::info("saving intrinsics for '{}' to '{}'", topic, filename);
        CalibParamManager::ParIntri::SaveCameraIntri(intri, filename,
                                                     Configor::Preference::OutputDataFormat);
    }
}

void CalibSolverIO::SaveVisualReprojError() const {
    std::string saveDir = Configor::DataStream::OutputPath + "/residual/reproj";
    if (TryCreatePath(saveDir)) {
        spdlog::info("saving visual reprojection error to dir: '{}'...", saveDir);
    } else {
        return;
    }

    for (const auto &[topic, _] : Configor::DataStream::EventTopics) {
        auto subSaveDir = saveDir + "/" + topic;
        if (!TryCreatePath(subSaveDir)) {
            spdlog::warn("create sub directory for '{}' failed: '{}'", topic, subSaveDir);
            continue;
        }

        auto &curCamPoses = _solver->_camPoses.at(topic);
        const auto &patterns = _solver->_extractedPatterns.at(topic);
        const auto &curGridIdToPoseIdxMap = _solver->_gridIdToPoseIdxMap.at(topic);
        const auto &intri = _solver->_parMgr->INTRI.Camera.at(topic);

        std::map<int, std::vector<Eigen::Vector2d>> residuals;

        for (const auto &grid2d : patterns->GetGrid2d()) {
            auto SE3_WtoCam = curCamPoses.at(curGridIdToPoseIdxMap.at(grid2d->id)).se3().inverse();
            auto &errorVec = residuals[grid2d->id];
            errorVec.resize(grid2d->centers.size());

            for (int i = 0; i < static_cast<int>(grid2d->centers.size()); ++i) {
                const auto &center = grid2d->centers.at(i);
                const Eigen::Vector2d pixel(center.x, center.y);

                const auto &point3d = patterns->GetGrid3d()->points.at(i);
                const Eigen::Vector3d point(point3d.x, point3d.y, point3d.z);

                Eigen::Vector3d pInCam = SE3_WtoCam * point;
                Eigen::Vector2d pInCamPlane(pInCam(0) / pInCam(2), pInCam(1) / pInCam(2));
                Eigen::Vector2d pixelPred = intri->CamToImg(intri->AddDisto(pInCamPlane));

                Eigen::Vector2d error = pixel - pixelPred;
                errorVec.at(i) = error;
            }
        }

        std::ofstream file(subSaveDir + "/residuals" + Configor::GetFormatExtension(),
                           std::ios::out);
        auto ar = GetOutputArchiveVariant(file, Configor::Preference::OutputDataFormat);
        SerializeByOutputArchiveVariant(ar, Configor::Preference::OutputDataFormat,
                                        cereal::make_nvp("reproj_residuals", residuals));
    }
    spdlog::info("saving visual reprojection errors finished!");
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
            if (rawEvs == nullptr) {
                continue;
            }
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

void CalibSolverIO::SaveSAEMaps(const std::string &topic,
                                const EventCircleExtractorPtr &extractor,
                                const cv::Mat &sae) {
    static std::map<std::string, int> idxMap;
    const auto count = idxMap[topic]++;

    if (IsOptionWith(OutputOption::SAEMapClusterNormFlowEvents, Configor::Preference::Outputs)) {
        std::string saveDir =
            Configor::DataStream::OutputPath + "/sae/cluster_norm_flow_events" + topic;
        if (!TryCreatePath(saveDir)) {
            return;
        }
        cv::imwrite(saveDir + "/SAEMapClusterNormFlowEvents-" + std::to_string(count) + ".png",
                    extractor->SAEMapClusterNormFlowEvents());
    }
    if (IsOptionWith(OutputOption::SAEMapExtractCircles, Configor::Preference::Outputs)) {
        std::string saveDir = Configor::DataStream::OutputPath + "/sae/extract_circles" + topic;
        if (!TryCreatePath(saveDir)) {
            return;
        }
        cv::imwrite(saveDir + "/SAEMapExtractCircles-" + std::to_string(count) + ".png",
                    extractor->SAEMapExtractCircles());
    }
    if (IsOptionWith(OutputOption::SAEMapExtractCirclesGrid, Configor::Preference::Outputs)) {
        std::string saveDir =
            Configor::DataStream::OutputPath + "/sae/extract_circles_grid" + topic;
        if (!TryCreatePath(saveDir)) {
            return;
        }
        cv::imwrite(saveDir + "/SAEMapExtractCirclesGrid-" + std::to_string(count) + ".png",
                    extractor->SAEMapExtractCirclesGrid());
    }
    if (IsOptionWith(OutputOption::SAEMapIdentifyCategory, Configor::Preference::Outputs)) {
        std::string saveDir = Configor::DataStream::OutputPath + "/sae/identify_category" + topic;
        if (!TryCreatePath(saveDir)) {
            return;
        }
        cv::imwrite(saveDir + "/SAEMapIdentifyCategory-" + std::to_string(count) + ".png",
                    extractor->SAEMapIdentifyCategory());
    }
    if (IsOptionWith(OutputOption::SAEMapSearchMatches, Configor::Preference::Outputs)) {
        std::string saveDir = Configor::DataStream::OutputPath + "/sae/search_matches" + topic;
        if (!TryCreatePath(saveDir)) {
            return;
        }
        cv::imwrite(saveDir + "/SAEMapSearchMatches-" + std::to_string(count) + ".png",
                    extractor->SAEMapSearchMatches3());
    }
    if (!sae.empty() && IsOptionWith(OutputOption::SAEMap, Configor::Preference::Outputs)) {
        std::string saveDir = Configor::DataStream::OutputPath + "/sae/sae" + topic;
        if (!TryCreatePath(saveDir)) {
            return;
        }
        cv::imwrite(saveDir + "/sae-" + std::to_string(count) + ".png", sae);
    }
}

void CalibSolverIO::SaveTinyViewerOnRender(const std::string &topic) {
    static std::map<std::string, int> idxMap;
    const auto count = idxMap[topic]++;

    std::string saveDir = Configor::DataStream::OutputPath + "/render/" + topic;
    if (!TryCreatePath(saveDir)) {
        return;
    }
    auto filename = saveDir + "/tv-render-" + std::to_string(count) + ".png";
    pangolin::SaveWindowOnRender(filename);
}

std::string CalibSolverIO::TopicConvertToFilename(const std::string &topic) {
    std::string result = topic;

    std::replace(result.begin(), result.end(), '/', '_');

    if (result[0] == '_') {
        result.erase(0, 1);
    }

    return result;
}
}  // namespace ns_ekalibr