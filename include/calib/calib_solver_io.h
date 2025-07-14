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

#ifndef CALIB_SOLVER_IO_H
#define CALIB_SOLVER_IO_H

#include "memory"
#include "util/cereal_archive_helper.hpp"
#include "opencv4/opencv2/core.hpp"

namespace ns_ekalibr {
class CalibSolver;
using CalibSolverPtr = std::shared_ptr<CalibSolver>;
class TimeVaryingEllipse;
using TimeVaryingEllipsePtr = std::shared_ptr<TimeVaryingEllipse>;
struct EventArray;
using EventArrayPtr = std::shared_ptr<EventArray>;
struct EventCircleExtractor;
using EventCircleExtractorPtr = std::shared_ptr<EventCircleExtractor>;
struct CalibParamManager;
using CalibParamManagerPtr = std::shared_ptr<CalibParamManager>;

class CalibSolverIO {
public:
    using Ptr = std::shared_ptr<CalibSolverIO>;
    // for a tracked 2d grid pattern
    using ExtractedCirclesVec = std::vector<std::pair<TimeVaryingEllipsePtr, EventArrayPtr>>;

private:
    CalibSolverPtr _solver;

public:
    explicit CalibSolverIO(CalibSolverPtr solver);

    static Ptr Create(const CalibSolverPtr &solver);

    void SaveByProductsToDisk() const;

    void SaveVisualIntrinsics() const;

protected:
    void SaveVisualReprojError() const;

public:
    static bool SaveRawEventsOfExtractedPatterns(const std::map<int, ExtractedCirclesVec> &data,
                                                 const std::string &filename,
                                                 double timeBias,
                                                 CerealArchiveType::Enum archiveType);

    static std::map<int, ExtractedCirclesVec> LoadRawEventsOfExtractedPatterns(
        const std::string &filename, double newTimeBias, CerealArchiveType::Enum archiveType);

    static std::pair<std::string, std::string> GetDiskPathOfExtractedGridPatterns(
        const std::string &topic);

    static std::string GetDiskPathOfOpenCVIntrinsicCalibRes(const std::string &topic);

    static void SaveSAEMaps(const std::string &topic,
                            const EventCircleExtractorPtr &extractor,
                            int grid2dId,
                            const cv::Mat &sae = cv::Mat());

    static void SaveSAEMaps(const std::string &topic,
                            const std::unordered_map<int, cv::Mat> &SAEMapTrackedCirclesGrid);

    static void SaveTinyViewerOnRender(const std::string &topic, int grid2dId);

    static void SaveStageCalibParam(const CalibParamManagerPtr &par, const std::string &desc);

protected:
    static std::string TopicConvertToFilename(const std::string &topic);
};
}  // namespace ns_ekalibr

#endif  // CALIB_SOLVER_IO_H
