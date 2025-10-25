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

#ifndef CONFIGOR_H
#define CONFIGOR_H

#include "util/enum_cast.hpp"
#include "util/cereal_archive_helper.hpp"
#include "cereal/types/map.hpp"
#include "cereal/types/string.hpp"

namespace ns_ekalibr {
enum class OutputOption : std::uint32_t {
    /**
     * @brief options
     */
    NONE = 1 << 0,
    ParamInEachIter = 1 << 1,
    BSplines = 1 << 2,
    HessianMat = 1 << 3,
    VisualReprojError = 1 << 4,
    SAEMapClusterNormFlowEvents = 1 << 5,
    SAEMapIdentifyCategory = 1 << 6,
    SAEMapSearchMatches = 1 << 7,
    SAEMapExtractCircles = 1 << 8,
    SAEMapExtractCirclesGrid = 1 << 9,
    SAEMap = 1 << 10,
    SAEMapTrackedCirclesGrid = 1 << 11,
    SAEMapAccumulatedEvents = 1 << 12,

    ALL = ParamInEachIter | BSplines | HessianMat | VisualReprojError |
          SAEMapClusterNormFlowEvents | SAEMapIdentifyCategory | SAEMapSearchMatches |
          SAEMapExtractCircles | SAEMapExtractCirclesGrid | SAEMap | SAEMapTrackedCirclesGrid |
          SAEMapAccumulatedEvents
};

struct Configor {
public:
    using Ptr = std::shared_ptr<Configor>;

public:
    static struct DataStream {
        struct IMUConfig {
        public:
            std::string Type;
            double AcceWhiteNoise;
            double GyroWhiteNoise;

            double AcceWeight(int hz) const {
                double cov = hz * AcceWhiteNoise * AcceWhiteNoise;
                return 1.0 / std::sqrt(cov);
            }
            double GyroWeight(int hz) const {
                double cov = hz * GyroWhiteNoise * GyroWhiteNoise;
                return 1.0 / std::sqrt(cov);
            }

            IMUConfig() = default;

        public:
            template <class Archive>
            void serialize(Archive &ar) {
                ar(CEREAL_NVP(Type), CEREAL_NVP(AcceWhiteNoise), CEREAL_NVP(GyroWhiteNoise));
            }
        };

        struct EventConfig {
        public:
            std::string Type;
            std::uint16_t Width;
            std::uint16_t Height;
            std::string Intrinsics;

            EventConfig() = default;

            bool NeedEstIntrinsics() const;

        public:
            template <class Archive>
            void serialize(Archive &ar) {
                ar(CEREAL_NVP(Type), CEREAL_NVP(Width), CEREAL_NVP(Height), CEREAL_NVP(Intrinsics));
            }
        };

        static std::map<std::string, IMUConfig> IMUTopics;

        static std::map<std::string, EventConfig> EventTopics;

        static std::string RefIMUTopic;

        static std::string BagPath;
        static double BeginTime;
        static double Duration;

        static std::string OutputPath;
        const static std::string PkgPath;
        const static std::string DebugPath;

    public:
        template <class Archive>
        void serialize(Archive &ar) {
            ar(CEREAL_NVP(IMUTopics), CEREAL_NVP(EventTopics), CEREAL_NVP(RefIMUTopic),
               CEREAL_NVP(BagPath), CEREAL_NVP(BeginTime), CEREAL_NVP(Duration));
        }
    } dataStream;

    static struct Prior {
        static constexpr int SplineOrder = 4;
        static std::string SpatTempPrioriPath;

        // sigma for the event circle extraction, in pixel
        static constexpr double evCircleExtractionSigma = 0.8;
        static constexpr double EvCameraWeight() { return 1.0 / evCircleExtractionSigma; }

        static double GravityNorm;
        static double TimeOffsetPadding;
        static bool OptTemporalParams;

        struct CirclePatternConfig {
        public:
            std::string Type;
            std::uint16_t Cols;
            std::uint16_t Rows;
            double SpacingMeters;
            double RadiusRate;  // radius = SpacingMeters / RadiusRate

            CirclePatternConfig() = default;

            double Radius() const { return SpacingMeters / RadiusRate; }

        public:
            template <class Archive>
            void serialize(Archive &ar) {
                ar(CEREAL_NVP(Type), CEREAL_NVP(Cols), CEREAL_NVP(Rows), CEREAL_NVP(SpacingMeters),
                   CEREAL_NVP(RadiusRate));
            }
        };

        static CirclePatternConfig CirclePattern;

        static double DecayTimeOfActiveEvents;

        struct CircleExtractorConfig {
            double ValidClusterAreaThd;
            double CircleClusterPairDirThd;
            double PointToCircleDistThd;

            CircleExtractorConfig() = default;

            template <class Archive>
            void serialize(Archive &ar) {
                ar(CEREAL_NVP(ValidClusterAreaThd), CEREAL_NVP(CircleClusterPairDirThd),
                   CEREAL_NVP(PointToCircleDistThd));
            }
        };

        static CircleExtractorConfig CircleExtractor;

        struct NormFlowEstimatorConfig {
            std::uint16_t WinSizeInPlaneFit;
            std::uint16_t RansacMaxIterations;
            double RansacInlierRatioThd;
            double EventToPlaneTimeDistThd;

            NormFlowEstimatorConfig() = default;

            template <class Archive>
            void serialize(Archive &ar) {
                ar(CEREAL_NVP(WinSizeInPlaneFit), CEREAL_NVP(RansacMaxIterations),
                   CEREAL_NVP(RansacInlierRatioThd), CEREAL_NVP(EventToPlaneTimeDistThd));
            }
        };

        static NormFlowEstimatorConfig NormFlowEstimator;

        struct KnotTimeDistConfig {
            double So3Spline;
            double ScaleSpline;

            KnotTimeDistConfig() = default;

        public:
            template <class Archive>
            void serialize(Archive &ar) {
                ar(CEREAL_NVP(So3Spline), CEREAL_NVP(ScaleSpline));
            }
        };

        // static KnotTimeDistConfig KnotTimeDist;

    public:
        template <class Archive>
        void serialize(Archive &ar) {
            ar(CEREAL_NVP(SpatTempPrioriPath), CEREAL_NVP(GravityNorm),
               CEREAL_NVP(TimeOffsetPadding), CEREAL_NVP(OptTemporalParams),
               CEREAL_NVP(CirclePattern), CEREAL_NVP(DecayTimeOfActiveEvents),
               CEREAL_NVP(CircleExtractor), CEREAL_NVP(NormFlowEstimator));
        }
    } prior;

    static struct Preference {
        static OutputOption Outputs;
        static std::set<std::string> OutputsStr;
        // str for file configuration, and enum for internal use
        static std::string OutputDataFormatStr;
        static CerealArchiveType::Enum OutputDataFormat;
        const static std::map<CerealArchiveType::Enum, std::string> FileExtension;

        static std::pair<double, double> EventViewerSpatialTemporalScale;
        static double SplineViewerSpatialScale;

        static bool Visualization;

        static int MaxEntityCountInViewer;

        const static std::string SO3_SPLINE, SCALE_SPLINE;

    public:
        template <class Archive>
        void serialize(Archive &ar) {
            ar(cereal::make_nvp("Outputs", OutputsStr),
               cereal::make_nvp("OutputDataFormat", OutputDataFormatStr), CEREAL_NVP(Visualization),
               CEREAL_NVP(MaxEntityCountInViewer));
        }
    } preference;

public:
    Configor();

    static Ptr Create();

    // load configure information from file
    static bool LoadConfigure(const std::string &filename,
                              CerealArchiveType::Enum archiveType = CerealArchiveType::Enum::YAML);

    // save configure information to file
    bool SaveConfigure(const std::string &filename,
                       CerealArchiveType::Enum archiveType = CerealArchiveType::Enum::YAML);

    // print the main fields
    static void PrintMainFields();

    static std::string GetFormatExtension();

protected:
    // check the input configure
    static void CheckConfigure();

public:
    template <class Archive>
    void serialize(Archive &ar) {
        ar(cereal::make_nvp("DataStream", dataStream), cereal::make_nvp("Prior", prior),
           cereal::make_nvp("Preference", preference));
    }
};
}  // namespace ns_ekalibr

#endif  // CONFIGOR_H
