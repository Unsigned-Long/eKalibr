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

#include "config/configor.h"
#include "util/status.hpp"
#include "ros/package.h"
#include "spdlog/spdlog.h"
#include "filesystem"

namespace ns_ekalibr {
std::map<std::string, Configor::DataStream::EventConfig> Configor::DataStream::EventTopics = {};
std::string Configor::DataStream::BagPath = {};
double Configor::DataStream::BeginTime = {};
double Configor::DataStream::Duration = {};
std::string Configor::DataStream::OutputPath = {};
const std::string Configor::DataStream::PkgPath = ros::package::getPath("ekalibr");
const std::string Configor::DataStream::DebugPath = PkgPath + "/debug/";

std::string Configor::Preference::OutputDataFormatStr = {};
CerealArchiveType::Enum Configor::Preference::OutputDataFormat = CerealArchiveType::Enum::YAML;
const std::map<CerealArchiveType::Enum, std::string> Configor::Preference::FileExtension = {
    {CerealArchiveType::Enum::YAML, ".yaml"},
    {CerealArchiveType::Enum::JSON, ".json"},
    {CerealArchiveType::Enum::XML, ".xml"},
    {CerealArchiveType::Enum::BINARY, ".bin"}};

Configor::Configor() = default;

void Configor::PrintMainFields() {
    std::stringstream ssEventTopics;
    for (const auto &[topic, info] : DataStream::EventTopics) {
        ssEventTopics << topic << '[' << info.Type << '|' << info.Width << 'x' << info.Height
                      << "] ";
    }
    std::string EventTopics = ssEventTopics.str();

#define DESC_FIELD(field) #field, field
#define DESC_FORMAT "\n{:>30}: {}"
    spdlog::info("main fields of configor:" DESC_FORMAT DESC_FORMAT DESC_FORMAT DESC_FORMAT
                     DESC_FORMAT DESC_FORMAT,
                 DESC_FIELD(EventTopics), DESC_FIELD(DataStream::BagPath),
                 DESC_FIELD(DataStream::BeginTime), DESC_FIELD(DataStream::Duration),
                 DESC_FIELD(DataStream::OutputPath), "Preference::OutputDataFormat",
                 Preference::OutputDataFormatStr);

#undef DESC_FIELD
#undef DESC_FORMAT
}

void Configor::CheckConfigure() {
    if (DataStream::EventTopics.empty()) {
        throw Status(Status::ERROR,
                     "the topic count of event cameras (i.e., DataStream::EventTopics) should be "
                     "larger equal than 1!");
    }

    if (!std::filesystem::exists(DataStream::BagPath)) {
        throw Status(Status::ERROR, "can not find the ros bag (i.e., DataStream::BagPath)!");
    }

    if (DataStream::OutputPath.empty()) {
        throw Status(Status::ERROR, "the output path (i.e., DataStream::OutputPath) is empty!");
    }
    if (!std::filesystem::exists(DataStream::OutputPath) &&
        !std::filesystem::create_directories(DataStream::OutputPath)) {
        // if the output path doesn't exist and create it failed
        throw Status(Status::ERROR,
                     "the output path (i.e., DataStream::OutputPath) can not be created!");
    }
}

Configor::Ptr Configor::Create() { return std::make_shared<Configor>(); }

std::string Configor::GetFormatExtension() {
    return Preference::FileExtension.at(Preference::OutputDataFormat);
}

bool Configor::LoadConfigure(const std::string &filename, CerealArchiveType::Enum archiveType) {
    // load configure info
    std::ifstream file(filename);
    if (!file.is_open()) {
        return false;
    }
    auto archive = GetInputArchiveVariant(file, archiveType);
    auto configor = Configor::Create();
    try {
        SerializeByInputArchiveVariant(archive, archiveType,
                                       cereal::make_nvp("Configor", *configor));
    } catch (const cereal::Exception &exception) {
        throw Status(
            Status::CRITICAL,
            "The configuration file '{}' is outdated or broken, and can not be loaded in eKalibr "
            "using cereal!!! To make it right, please refer to our latest configuration file "
            "template released at "
            "https://github.com/Unsigned-Long/eKalibr.git, and then fix your custom configuration "
            "file. Detailed cereal exception information: \n'{}'",
            filename, exception.what());
    }

    // perform internal data transformation
    try {
        Preference::OutputDataFormat =
            EnumCast::stringToEnum<CerealArchiveType::Enum>(Preference::OutputDataFormatStr);
    } catch (...) {
        throw Status(Status::CRITICAL, "unsupported data format '{}' for io!!!",
                     Preference::OutputDataFormatStr);
    }

    // perform checking
    CheckConfigure();
    return true;
}

bool Configor::SaveConfigure(const std::string &filename, CerealArchiveType::Enum archiveType) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        return false;
    }
    auto archive = GetOutputArchiveVariant(file, archiveType);
    SerializeByOutputArchiveVariant(archive, archiveType, cereal::make_nvp("Configor", *this));
    return true;
}

}  // namespace ns_ekalibr