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

#include "ros/ros.h"
#include "util/status.hpp"
#include "spdlog/fmt/bundled/color.h"
#include "spdlog/spdlog.h"
#include "config/configor.h"
#include "util/utils.h"
#include "util/utils_tpl.hpp"
#include "filesystem"
#include "core/calib_solver.h"
#include "core/calib_param_mgr.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "ekalibr_prog");

    try {
        ns_ekalibr::ConfigSpdlog();

        ns_ekalibr::PrintIKalibrLibInfo();

        // load settings
        auto configPath = ns_ekalibr::GetParamFromROS<std::string>("/ekalibr_prog/config_path");
        spdlog::info("loading configure from yaml file '{}'...", configPath);
        if (!std::filesystem::exists(configPath)) {
            throw ns_ekalibr::Status(ns_ekalibr::Status::CRITICAL,
                                     "configure file dose not exist: '{}'", configPath);
        }

        if (!ns_ekalibr::Configor::LoadConfigure(configPath)) {
            /**
             * Attention: once the configure information is loaded in to this program, anywhere this
             * configure infomoration can be used, as the 'Configor' is a large state mechine (a
             * struct that contains all static configure fields), just like the one in OpenGL, such
             * a design may lead to confuse to the new, and not easy to understand, however it can
             * make code cleaner.
             *
             * Suggestion from authors: such a design is not recommanded in other programs,
             * especially those multi-thread programs!!!
             */
            throw ns_ekalibr::Status(ns_ekalibr::Status::CRITICAL,
                                     "load configure file from '{}' failed!", configPath);
        } else {
            ns_ekalibr::Configor::PrintMainFields();
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        // create parameter manager based on loaded configure information
        auto parMgr = ns_ekalibr::CalibParamManager::InitParamsFromConfigor();
        parMgr->ShowParamStatus();

        // pass parameter manager to solver for solving
        auto solver = ns_ekalibr::CalibSolver::Create(parMgr);
        // the calibration results are stored in 'parMgr'
        solver->Process();

        // solve finished, save calibration results (file type: JSON | YAML | XML | BINARY)
        const auto filename = ns_ekalibr::Configor::DataStream::OutputPath + "/ikalibr_param" +
                              ns_ekalibr::Configor::GetFormatExtension();
        parMgr->Save(filename, ns_ekalibr::Configor::Preference::OutputDataFormat);

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