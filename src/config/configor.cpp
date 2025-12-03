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
#include "magic_enum_flags.hpp"
#include "cereal/types/set.hpp"

namespace ns_ekalibr {
const static std::map<std::string, OutputOption> OutputOptionMap = {
    {"NONE", OutputOption::NONE},
    {"ParamInEachIter", OutputOption::ParamInEachIter},
    {"BSplines", OutputOption::BSplines},
    {"HessianMat", OutputOption::HessianMat},
    {"VisualReprojError", OutputOption::VisualReprojError},
    {"InertialAcceError", OutputOption::InertialAcceError},
    {"InertialGyroError", OutputOption::InertialGyroError},
    {"SAEMapClusterNormFlowEvents", OutputOption::SAEMapClusterNormFlowEvents},
    {"SAEMapIdentifyCategory", OutputOption::SAEMapIdentifyCategory},
    {"SAEMapSearchMatches", OutputOption::SAEMapSearchMatches},
    {"SAEMapExtractCircles", OutputOption::SAEMapExtractCircles},
    {"SAEMapExtractCirclesGrid", OutputOption::SAEMapExtractCirclesGrid},
    {"SAEMapTrackedCirclesGrid", OutputOption::SAEMapTrackedCirclesGrid},
    {"SAEMap", OutputOption::SAEMap},
    {"SAEMapAccumulatedEvents", OutputOption::SAEMapAccumulatedEvents},
    {"ALL", OutputOption::ALL},
};

using namespace magic_enum::bitwise_operators;

Configor::DataStream Configor::dataStream = {};
std::map<std::string, Configor::DataStream::IMUConfig> Configor::DataStream::IMUTopics = {};
std::map<std::string, Configor::DataStream::EventConfig> Configor::DataStream::EventTopics = {};
std::string Configor::DataStream::RefIMUTopic = {};
std::string Configor::DataStream::BagPath = {};
double Configor::DataStream::BeginTime = {};
double Configor::DataStream::Duration = {};
std::string Configor::DataStream::OutputPath = {};
const std::string Configor::DataStream::PkgPath = ros::package::getPath("ekalibr");
const std::string Configor::DataStream::DebugPath = PkgPath + "/debug/";

Configor::Prior Configor::prior = {};
double Configor::Prior::GravityNorm = {};
double Configor::Prior::TimeOffsetPadding = {};
bool Configor::Prior::OptTemporalParams = {};
Configor::Prior::CirclePatternConfig Configor::Prior::CirclePattern = {};
double Configor::Prior::DecayTimeOfActiveEvents = 0.0;
Configor::Prior::CircleExtractorConfig Configor::Prior::CircleExtractor = {};
Configor::Prior::NormFlowEstimatorConfig Configor::Prior::NormFlowEstimator = {};
std::string Configor::Prior::SpatTempPrioriPath = {};

OutputOption Configor::Preference::Outputs = OutputOption::NONE;
std::set<std::string> Configor::Preference::OutputsStr = {};
Configor::Preference Configor::preference = {};
std::string Configor::Preference::OutputDataFormatStr = {};
CerealArchiveType::Enum Configor::Preference::OutputDataFormat = CerealArchiveType::Enum::YAML;
const std::map<CerealArchiveType::Enum, std::string> Configor::Preference::FileExtension = {
    {CerealArchiveType::Enum::YAML, ".yaml"},
    {CerealArchiveType::Enum::JSON, ".json"},
    {CerealArchiveType::Enum::XML, ".xml"},
    {CerealArchiveType::Enum::BINARY, ".bin"}};
std::pair<double, double> Configor::Preference::EventViewerSpatialTemporalScale = {0.01, 50.0};
double Configor::Preference::SplineViewerSpatialScale = 15.0;
bool Configor::Preference::Visualization = {};
int Configor::Preference::MaxEntityCountInViewer = {};
const std::string Configor::Preference::SO3_SPLINE = "SO3_SPLINE";
const std::string Configor::Preference::SCALE_SPLINE = "SCALE_SPLINE";

bool Configor::DataStream::EventConfig::NeedEstIntrinsics() const { return Intrinsics.empty(); }

Configor::Configor() = default;

void Configor::PrintMainFields() {
    std::stringstream ssEventTopics, ssIMUTopics;
    for (const auto &[topic, info] : DataStream::EventTopics) {
        ssEventTopics << topic << '[' << info.Type << '|' << info.Width << 'x' << info.Height
                      << "] ";
    }
    for (const auto &[topic, info] : DataStream::IMUTopics) {
        ssIMUTopics << topic << '[' << info.Type << "] ";
    }
    std::string EventTopics = ssEventTopics.str();
    std::string IMUTopics = ssIMUTopics.str();

    auto GetOptString = [](OutputOption opt) -> std::string {
        std::stringstream stringStream;
        stringStream << magic_enum::enum_flags_name(opt);
        return stringStream.str();
    };

#define DESC_FIELD(field) #field, field
#define DESC_FORMAT "\n{:>42}: {}"
    spdlog::info(
        "main fields of configor:" DESC_FORMAT DESC_FORMAT DESC_FORMAT DESC_FORMAT DESC_FORMAT
            DESC_FORMAT DESC_FORMAT DESC_FORMAT DESC_FORMAT DESC_FORMAT DESC_FORMAT DESC_FORMAT
                DESC_FORMAT DESC_FORMAT DESC_FORMAT DESC_FORMAT DESC_FORMAT DESC_FORMAT DESC_FORMAT
                    DESC_FORMAT DESC_FORMAT DESC_FORMAT DESC_FORMAT DESC_FORMAT DESC_FORMAT
                        DESC_FORMAT DESC_FORMAT,
        DESC_FIELD(EventTopics), DESC_FIELD(IMUTopics), DESC_FIELD(DataStream::RefIMUTopic),
        DESC_FIELD(DataStream::BagPath), DESC_FIELD(DataStream::BeginTime),
        DESC_FIELD(DataStream::Duration), DESC_FIELD(DataStream::OutputPath),
        DESC_FIELD(Prior::SpatTempPrioriPath), DESC_FIELD(Prior::GravityNorm),
        DESC_FIELD(Prior::TimeOffsetPadding), DESC_FIELD(Prior::OptTemporalParams),
        DESC_FIELD(Prior::DecayTimeOfActiveEvents),
        // fields for CirclePattern
        "CirclePattern::Type", Prior::CirclePattern.Type,  // pattern type
        "CirclePattern::Cols", Prior::CirclePattern.Cols,  // number of circles (cols)
        "CirclePattern::Rows", Prior::CirclePattern.Rows,  // number of circles (rows)
        "CirclePattern::SpacingMeters",
        Prior::CirclePattern.SpacingMeters,  // distance between circles
        "CirclePattern::RadiusRate", Prior::CirclePattern.RadiusRate,
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
        "Preference::Outputs", GetOptString(Preference::Outputs), "Preference::OutputDataFormat",
        Preference::OutputDataFormatStr, DESC_FIELD(Preference::Visualization),
        DESC_FIELD(Preference::MaxEntityCountInViewer));

#undef DESC_FIELD
#undef DESC_FORMAT
}

void Configor::CheckConfigure() {
    if (DataStream::EventTopics.empty()) {
        throw Status(Status::ERROR,
                     "the topic count of event cameras (i.e., DataStream::EventTopics) should be "
                     "larger equal than 1!");
    }
    std::multiset<std::string> topics;
    for (const auto &[topic, config] : DataStream::EventTopics) {
        if (topic.empty()) {
            throw Status(Status::ERROR, "the topic of event camera should not be empty string!");
        }
        // verify event camera type.
        if (config.TemporarilyForFrame) {
            FrameModel::FromString(config.Type);
        } else {
            EventModel::FromString(config.Type);
        }
        topics.insert(topic);
    }
    for (const auto &[topic, config] : DataStream::IMUTopics) {
        if (topic.empty()) {
            throw Status(Status::ERROR, "the topic of imu should not be empty string!");
        }
        // verify imu type
        IMUModel::FromString(config.Type);

        if (config.AcceWhiteNoise <= 0.0) {
            throw Status(Status::ERROR, "accelerator white noise of IMU '{}' should be positive!",
                         topic);
        }
        if (config.GyroWhiteNoise <= 0.0) {
            throw Status(Status::ERROR, "gyroscope white noise of IMU '{}' should be positive!",
                         topic);
        }
        topics.insert(topic);
    }
    for (const auto &topic : topics) {
        if (topics.count(topic) != 1) {
            throw Status(Status::ERROR,
                         "the topic of '{}' is ambiguous, associated to not unique sensors!",
                         topic);
        }
    }

    // the reference imu should be one of multiple imus
    if (!DataStream::IMUTopics.empty() &&
        DataStream::IMUTopics.find(DataStream::RefIMUTopic) == DataStream::IMUTopics.cend()) {
        auto oldRefIMUTopics = DataStream::RefIMUTopic;
        DataStream::RefIMUTopic = DataStream::IMUTopics.cbegin()->first;
        spdlog::warn(
            "the reference IMU, i.e., '{}', is not one of the IMUs! set '{}' as the reference IMU!",
            oldRefIMUTopics, DataStream::RefIMUTopic);
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
    if (Prior::TimeOffsetPadding <= 0.0) {
        throw Status(
            Status::ERROR,
            "the time offset padding (i.e., Prior::TimeOffsetPadding) should be positive!");
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

    if (Prior::CirclePattern.RadiusRate < 1E-6 /*m*/) {
        throw Status(Status::ERROR,
                     "the radius rate in the circle pattern (i.e., "
                     "CirclePattern::RadiusRate) should be positive!");
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
            Status::WARNING,
            "The configuration file '{}' is outdated or broken, and can not be loaded in eKalibr "
            "using cereal!!! To make it right, please refer to our latest configuration file "
            "template released at "
            "https://github.com/Unsigned-Long/eKalibr/blob/master/config/ekalibr-config.yaml, and "
            "then fix your custom configuration file. Detailed cereal exception information: "
            "\n'{}'",
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
    for (const auto &output : Preference::OutputsStr) {
        // when the enum is out of range of [MAGIC_ENUM_RANGE_MIN, MAGIC_ENUM_RANGE_MAX],
        // magic_enum would not work
        // try {
        //     Configor::Preference::Outputs |= EnumCast::stringToEnum<OutputOption>(output);
        // } catch (...) {
        //     throw Status(Status::CRITICAL, "unsupported output context: '{}'!!!", output);
        // }
        if (auto iter = OutputOptionMap.find(output); iter == OutputOptionMap.cend()) {
            throw Status(Status::CRITICAL, "unsupported output context: '{}'!!!", output);
        } else {
            Configor::Preference::Outputs |= iter->second;
        }
    }
    if (std::filesystem::exists(DataStream::BagPath)) {
        auto bagPath = std::filesystem::path(DataStream::BagPath);
        auto outputDir = (bagPath.parent_path() / bagPath.stem()).string();
        if (!std::filesystem::exists(outputDir) &&
            !std::filesystem::create_directories(outputDir)) {
            throw Status(Status::CRITICAL, "create directory '{}' to save outputs failed!!!",
                         outputDir);
        } else {
            DataStream::OutputPath = outputDir;
        }
    }
    for (auto &[topic, config] : DataStream::EventTopics) {
        for (const auto &name : magic_enum::enum_names<FrameModelType>()) {
            if (config.Type == name) {
                config.TemporarilyForFrame = true;
                spdlog::warn("the event topic '{}' is configured as frame model '{}'!!!", topic,
                             config.Type);
                break;
            }
        }
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