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

#include "util/utils.h"
#include "util/utils_tpl.hpp"
#include "spdlog/spdlog.h"
#include "filesystem"
#include "util/status.hpp"
#include "ros/ros.h"
#include <sensor/event_rosbag_loader.h>
#include <fstream>
#include <util/tqdm.h>

void RosbagToE2VID(const std::string &outputPath,
                   const std::vector<ns_ekalibr::EventArray::Ptr> &data,
                   int width,
                   int height) {
    std::ofstream ofs(outputPath, std::ios::out);
    if (!ofs.is_open()) {
        throw ns_ekalibr::Status(ns_ekalibr::Status::CRITICAL,
                                 "failed to open the output file: '{}'", outputPath);
    }
    ofs << std::fixed << std::setprecision(12);
    ofs << width << ' ' << height << '\n';
    auto bar = std::make_shared<tqdm>();
    for (int i = 0; i < static_cast<int>(data.size()); i++) {
        const auto &d = data.at(i);
        bar->progress(i, static_cast<int>(data.size()));
        for (const auto &e : d->GetEvents()) {
            ofs << e->GetTimestamp() << " " << e->GetPos()(0) << " " << e->GetPos()(1) << " "
                << (e->GetPolarity() ? 1 : 0) << "\n";
        }
    }
    bar->finish();
    ofs.close();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ekalibr_converter");

    try {
        ns_ekalibr::ConfigSpdlog();

        ns_ekalibr::PrintEKalibrLibInfo();

        // load settings
        auto rosbag_file =
            ns_ekalibr::GetParamFromROS<std::string>("/ekalibr_converter/rosbag_file");
        spdlog::info("the input ros bag: '{}'", rosbag_file);
        if (!std::filesystem::exists(rosbag_file)) {
            throw ns_ekalibr::Status(ns_ekalibr::Status::CRITICAL,
                                     "the input ros bag dose not exist: '{}'", rosbag_file);
        }

        auto event_topics =
            ns_ekalibr::GetParamFromROS<std::string>("/ekalibr_converter/event_topics");
        spdlog::info("the event topics: '{}'", event_topics);

        auto topics = ns_ekalibr::SplitString(event_topics, ';');
        if (topics.empty()) {
            throw ns_ekalibr::Status(
                ns_ekalibr::Status::CRITICAL,
                "there is no any event topic in the parameter 'event_topics'!!!");
        }

        auto desired_type =
            ns_ekalibr::GetParamFromROS<std::string>("/ekalibr_converter/desired_type");
        spdlog::info("the desired output type: '{}'", desired_type);

        std::map<std::string, std::string> topicMap;
        std::map<std::string, std::pair<int, int>> whMap;
        for (const auto &topic : topics) {
            auto items = ns_ekalibr::SplitString(topic, ':');
            if (items.size() != 3) {
                throw ns_ekalibr::Status(ns_ekalibr::Status::CRITICAL,
                                         "the topic '{}' is invalid, it should be in the "
                                         "format of '<topic>:<event_type>:<width>x<height>'",
                                         topic);
            }
            topicMap.insert({items[0], items[1]});
            auto wh = ns_ekalibr::SplitString(items[2], 'x');
            whMap.insert({items[0], std::pair{std::stoi(wh[0]), std::stoi(wh[1])}});
        }

        auto dataMap = ns_ekalibr::LoadEventsFromROSBag(rosbag_file, topicMap);

        for (const auto &[topic, data] : dataMap) {
            // replace '/' in topic with '_'
            auto newTopic = topic;
            std::replace(newTopic.begin(), newTopic.end(), '/', '_');
            if (newTopic.front() == '_') {
                newTopic = newTopic.substr(1);
            }
            std::string outputDir = std::filesystem::path(rosbag_file).parent_path().string();
            std::string outputName =
                std::filesystem::path(rosbag_file).stem().string() + "_" + newTopic;

            if (desired_type == "e2vid") {
                std::string oFilePath = outputDir + "/e2vid_" + outputName + ".txt";
                spdlog::info("saving events of topic '{}' to file: '{}'", topic, oFilePath);
                RosbagToE2VID(oFilePath, data, whMap.at(topic).first, whMap.at(topic).second);
            } else {
                throw ns_ekalibr::Status(ns_ekalibr::Status::CRITICAL,
                                         "the desired type '{}' is not supported yet!!!",
                                         desired_type);
            }
        }

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
