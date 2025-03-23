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

#include "calib/calib_param_mgr.h"
#include "util/utils.h"
#include "util/utils_tpl.hpp"
#include "spdlog/spdlog.h"
#include "filesystem"
#include "util/status.hpp"
#include "ros/ros.h"
#include "regex"
#include <util/tqdm.h>
#include "veta/camera/pinhole.h"
#include "sensor/imu_intrinsic.h"

#include <core/visual_distortion.h>
#include <opencv2/imgcodecs.hpp>

int main(int argc, char **argv) {
    ros::init(argc, argv, "ekalibr_undistortion");

    try {
        ns_ekalibr::ConfigSpdlog();

        ns_ekalibr::PrintEKalibrLibInfo();

        // load settings
        auto valid_name_regex =
            ns_ekalibr::GetParamFromROS<std::string>("/ekalibr_undistortion/valid_name_regex");
        spdlog::info("valid name regex: '{}'", valid_name_regex);

        auto ekalibr_param_file =
            ns_ekalibr::GetParamFromROS<std::string>("/ekalibr_undistortion/ekalibr_param_file");
        if (!std::filesystem::exists(ekalibr_param_file)) {
            throw ns_ekalibr::Status(ns_ekalibr::Status::CRITICAL,
                                     "ekalibr param file dose not exist: '{}'", ekalibr_param_file);
        }
        spdlog::info("ekalibr param file: '{}'", ekalibr_param_file);
        auto par = ns_ekalibr::CalibParamManager::Load(ekalibr_param_file);

        auto topic_in_param_file =
            ns_ekalibr::GetParamFromROS<std::string>("/ekalibr_undistortion/topic_in_param_file");
        spdlog::info("camera topic in param file: '{}'", topic_in_param_file);

        auto intriIter = par->INTRI.Camera.find(topic_in_param_file);
        if (intriIter == par->INTRI.Camera.end()) {
            throw ns_ekalibr::Status(
                ns_ekalibr::Status::CRITICAL,
                "camera topic '{}' dose not exist in eKalibr parameter file: '{}'",
                topic_in_param_file, ekalibr_param_file);
        }
        const auto &intri = intriIter->second;

        auto imgs_dir = ns_ekalibr::GetParamFromROS<std::string>("/ekalibr_undistortion/imgs_dir");
        if (!std::filesystem::exists(imgs_dir)) {
            throw ns_ekalibr::Status(ns_ekalibr::Status::CRITICAL,
                                     "direction of images dose not exist: '{}'", imgs_dir);
        }
        spdlog::info("loading images from direction '{}'...", imgs_dir);
        auto filenames = ns_ekalibr::FilesInDirRecursive(imgs_dir);

        // erase filenames that are not images
        if (!valid_name_regex.empty()) {
            std::regex fileNameRegex(valid_name_regex);
            auto iter = std::remove_if(
                filenames.begin(), filenames.end(), [&fileNameRegex](const std::string &filename) {
                    std::string sName = std::filesystem::path(filename).filename().string();
                    return !std::regex_match(sName, fileNameRegex);
                });
            filenames.erase(iter, filenames.cend());
        }
        if (filenames.empty()) {
            throw ns_ekalibr::Status(ns_ekalibr::Status::CRITICAL,
                                     "there is no any matched image in this directory '{}'!!!",
                                     imgs_dir);
        }

        spdlog::info("find '{}' files that match the regex expression, start undistort images...",
                     filenames.size());

        auto undistortionMap = ns_ekalibr::VisualUndistortionMap::Create(intri);
        auto bar = std::make_shared<tqdm>();
        for (int i = 0; i < static_cast<int>(filenames.size()); i++) {
            bar->progress(i, static_cast<int>(filenames.size()));
            const auto &filename = filenames.at(i);
            auto imgUndistorted =
                undistortionMap->RemoveDistortion(cv::imread(filename, cv::IMREAD_UNCHANGED));

            auto path = std::filesystem::path(filename);
            auto newFilename = path.parent_path() /
                               (path.stem().string() + "-undistortion" + path.extension().string());
            cv::imwrite(newFilename, imgUndistorted);
        }
        bar->finish();

        static constexpr auto FStyle = fmt::emphasis::italic | fmt::fg(fmt::color::green);
        spdlog::info(format(FStyle, "solving and outputting finished!!! Everything is fine!!!"));

    } catch (const ns_ekalibr::EKalibrStatus &status) {
        // if error happened, print it
        static constexpr auto FStyle = fmt::emphasis::italic | fmt::fg(fmt::color::green);
        static constexpr auto WECStyle = fmt::emphasis::italic | fmt::fg(fmt::color::red);
        switch (status.flag) {
            case ns_ekalibr::Status::FINE:
                // this case usually won't happen
                spdlog::info(fmt::format(FStyle, "{}", status.what));
                break;
            case ns_ekalibr::Status::WARNING:
                spdlog::warn(fmt::format(WECStyle, "{}", status.what));
                break;
            case ns_ekalibr::Status::ERROR:
                spdlog::error(fmt::format(WECStyle, "{}", status.what));
                break;
            case ns_ekalibr::Status::CRITICAL:
                spdlog::critical(fmt::format(WECStyle, "{}", status.what));
                break;
        }
    } catch (const std::exception &e) {
        // an unknown exception not thrown by this program
        static constexpr auto WECStyle = fmt::emphasis::italic | fmt::fg(fmt::color::red);
        spdlog::critical(fmt::format(WECStyle, "unknown error happened: '{}'", e.what()));
    }
    return 0;
}
