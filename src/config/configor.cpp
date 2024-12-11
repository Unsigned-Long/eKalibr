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
#include <sensor/sensor_model.h>

namespace ns_ekalibr {
Configor::DataStream Configor::dataStream = {};
std::map<std::string, Configor::DataStream::EventConfig> Configor::DataStream::EventTopics = {};
std::string Configor::DataStream::BagPath = {};
double Configor::DataStream::BeginTime = {};
double Configor::DataStream::Duration = {};
std::string Configor::DataStream::OutputPath = {};
const std::string Configor::DataStream::PkgPath = ros::package::getPath("ekalibr");
const std::string Configor::DataStream::DebugPath = PkgPath + "/debug/";

Configor::Prior Configor::prior = {};
Configor::Prior::CirclePatternConfig Configor::Prior::CirclePattern = {};
double Configor::Prior::DecayTimeOfActiveEvents = 0.0;
Configor::Prior::CircleExtractorConfig Configor::Prior::CircleExtractor = {};
Configor::Prior::NormFlowEstimatorConfig Configor::Prior::NormFlowEstimator = {};

Configor::Preference Configor::preference = {};
std::string Configor::Preference::OutputDataFormatStr = {};
CerealArchiveType::Enum Configor::Preference::OutputDataFormat = CerealArchiveType::Enum::YAML;
const std::map<CerealArchiveType::Enum, std::string> Configor::Preference::FileExtension = {
    {CerealArchiveType::Enum::YAML, ".yaml"},
    {CerealArchiveType::Enum::JSON, ".json"},
    {CerealArchiveType::Enum::XML, ".xml"},
    {CerealArchiveType::Enum::BINARY, ".bin"}};
std::pair<double, double> Configor::Preference::EventViewerSpatialTemporalScale = {0.01, 50.0};
bool Configor::Preference::Visualization = {};
int Configor::Preference::MaxEntityCountInViewer = {};

Configor::Configor() = default;

void Configor::PrintMainFields() {
    std::stringstream ssEventTopics;
    for (const auto &[topic, info] : DataStream::EventTopics) {
        ssEventTopics << topic << '[' << info.Type << '|' << info.Width << 'x' << info.Height
                      << "] ";
    }
    std::string EventTopics = ssEventTopics.str();

#define DESC_FIELD(field) #field, field
#define DESC_FORMAT "\n{:>40}: {}"
    spdlog::info(
        "main fields of configor:" DESC_FORMAT DESC_FORMAT DESC_FORMAT DESC_FORMAT DESC_FORMAT
            DESC_FORMAT DESC_FORMAT DESC_FORMAT DESC_FORMAT DESC_FORMAT DESC_FORMAT DESC_FORMAT
                DESC_FORMAT DESC_FORMAT DESC_FORMAT DESC_FORMAT DESC_FORMAT DESC_FORMAT DESC_FORMAT
                    DESC_FORMAT,
        DESC_FIELD(EventTopics), DESC_FIELD(DataStream::BagPath), DESC_FIELD(DataStream::BeginTime),
        DESC_FIELD(DataStream::Duration), DESC_FIELD(DataStream::OutputPath),
        DESC_FIELD(Prior::DecayTimeOfActiveEvents),
        // fields for CirclePattern
        "CirclePattern::Type", Prior::CirclePattern.Type,  // pattern type
        "CirclePattern::Cols", Prior::CirclePattern.Cols,  // number of circles (cols)
        "CirclePattern::Rows", Prior::CirclePattern.Rows,  // number of circles (rows)
        "CirclePattern::SpacingMeters",
        Prior::CirclePattern.SpacingMeters,  // distance between circles
        // fields for CircleExtractor
        "CircleExtractor::ValidClusterAreaThd", Prior::CircleExtractor.ValidClusterAreaThd,
        "CircleExtractor::CircleClusterPairDirThd", Prior::CircleExtractor.CircleClusterPairDirThd,
        "CircleExtractor::PointToCircleDistThd", Prior::CircleExtractor.PointToCircleDistThd,
        // fields for NormFlowEstimator
        "NormFlowEstimator::WinSizeInPlaneFit", Prior::NormFlowEstimator.WinSizeInPlaneFit,
        "NormFlowEstimator::RansacMaxIterations", Prior::NormFlowEstimator.RansacMaxIterations,
        "NormFlowEstimator::RansacInlierRatioThd", Prior::NormFlowEstimator.RansacInlierRatioThd,
        "NormFlowEstimator::EventToPlaneTimeDistThd",
        Prior::NormFlowEstimator.EventToPlaneTimeDistThd,
        // Preference
        "Preference::OutputDataFormat", Preference::OutputDataFormatStr,
        DESC_FIELD(Preference::Visualization), DESC_FIELD(Preference::MaxEntityCountInViewer));

#undef DESC_FIELD
#undef DESC_FORMAT
}

void Configor::CheckConfigure() {
    if (DataStream::EventTopics.empty()) {
        throw Status(Status::ERROR,
                     "the topic count of event cameras (i.e., DataStream::EventTopics) should be "
                     "larger equal than 1!");
    }

    for (const auto &[topic, config] : DataStream::EventTopics) {
        if (topic.empty()) {
            throw Status(Status::ERROR, "the topic of event camera should not be empty string!");
        }
        // verify event camera type
        EventModel::FromString(config.Type);
    }
    // verify circle pattern type
    CirclePattern::FromString(Prior::CirclePattern.Type);

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

    if (Prior::DecayTimeOfActiveEvents < 1E-6) {
        throw Status(Status::ERROR,
                     "the decay time of the surface of active events (i.e., "
                     "Prior::DecayTimeOfActiveEvents) should be positive!");
    }

    if (Prior::CirclePattern.Cols == 0) {
        throw Status(Status::ERROR,
                     "the columns of circle grid pattern (i.e., "
                     "CirclePattern::Cols) should be positive!");
    }

    if (Prior::CirclePattern.Rows == 0) {
        throw Status(Status::ERROR,
                     "the rows of circle grid pattern (i.e., "
                     "CirclePattern::Rows) should be positive!");
    }

    if (Prior::CirclePattern.SpacingMeters < 1E-6 /*m*/) {
        throw Status(Status::ERROR,
                     "the distance between circles in the circle pattern (i.e., "
                     "CirclePattern::SpacingMeters) should be positive!");
    }

    if (Prior::CircleExtractor.ValidClusterAreaThd < 1 /*pixels*/) {
        throw Status(Status::ERROR,
                     "the valid cluster area threshold (i.e., "
                     "CircleExtractor::ValidClusterAreaThd) should be positive!");
    }

    if (Prior::CircleExtractor.CircleClusterPairDirThd < 1E-6 /*degrees*/) {
        throw Status(Status::ERROR,
                     "the circle cluster pair threshold (i.e., "
                     "CircleExtractor::CircleClusterPairDirThd) should be positive!");
    }

    if (Prior::CircleExtractor.PointToCircleDistThd < 1E-6 /*pixels*/) {
        throw Status(Status::ERROR,
                     "the point-to-circle threshold (i.e., "
                     "CircleExtractor::PointToCircleDistThd) should be positive!");
    }

    if (Prior::NormFlowEstimator.RansacMaxIterations < 1) {
        throw Status(Status::ERROR,
                     "the ransac max iterations (i.e., NormFlowEstimator::RansacMaxIterations) "
                     "should be larger than zero!");
    }

    if (Prior::NormFlowEstimator.RansacInlierRatioThd <= 0.0 ||
        Prior::NormFlowEstimator.RansacInlierRatioThd >= 1.0) {
        throw Status(Status::ERROR,
                     "the ransac inlier ratio threshold (i.e., "
                     "NormFlowEstimator::RansacInlierRatioThd) should be in range of (0.0, 1.0)!");
    }

    if (Prior::NormFlowEstimator.WinSizeInPlaneFit < 1) {
        throw Status(
            Status::ERROR,
            "the (half) window size in plane fitting (i.e., NormFlowEstimator::WinSizeInPlaneFit) "
            "should be larger than zero!");
    }

    if (Prior::NormFlowEstimator.EventToPlaneTimeDistThd < 1E-6) {
        throw Status(Status::ERROR,
                     "the event-to-plane time dist threshold (i.e., "
                     "NormFlowEstimator::EventToPlaneTimeDistThd) should be larger than zero!");
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