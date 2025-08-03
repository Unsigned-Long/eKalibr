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

#ifndef CALIB_SOLVER_H
#define CALIB_SOLVER_H

#include "memory"
#include "ceres/ceres.h"
#include "tiny-viewer/core/pose.hpp"
#include <sensor/sensor_model.h>
#include "opencv4/opencv2/core/types.hpp"
#include "ctraj/core/pose.hpp"
#include "sensor/imu.hpp"
#include "ctraj/core/spline_bundle.h"
#include "config/configor.h"

namespace ns_veta {
struct PinholeIntrinsic;
using PinholeIntrinsicPtr = std::shared_ptr<PinholeIntrinsic>;

struct PinholeIntrinsicBrownT2;
using PinholeIntrinsicBrownT2Ptr = std::shared_ptr<PinholeIntrinsicBrownT2>;
}  // namespace ns_veta

namespace ns_ekalibr {
class Viewer;
using ViewerPtr = std::shared_ptr<Viewer>;
struct EventArray;
using EventArrayPtr = std::shared_ptr<EventArray>;
struct CircleGrid2D;
using CircleGrid2DPtr = std::shared_ptr<CircleGrid2D>;
struct CircleGrid3D;
using CircleGrid3DPtr = std::shared_ptr<CircleGrid3D>;
struct CircleGridPattern;
using CircleGridPatternPtr = std::shared_ptr<CircleGridPattern>;
class CalibParamManager;
using CalibParamManagerPtr = std::shared_ptr<CalibParamManager>;
class Estimator;
using EstimatorPtr = std::shared_ptr<Estimator>;
enum class OptOption : std::uint64_t;
struct VisualProjectionPair;
using VisualProjectionPairPtr = std::shared_ptr<VisualProjectionPair>;
struct VisualProjectionCircleBasedPair;
using VisualProjectionCircleBasedPairPtr = std::shared_ptr<VisualProjectionCircleBasedPair>;
class SpatialTemporalPriori;
using SpatialTemporalPrioriPtr = std::shared_ptr<SpatialTemporalPriori>;
struct TimeVaryingEllipse;
using TimeVaryingEllipsePtr = std::shared_ptr<TimeVaryingEllipse>;
class CalibSolverIO;

class CalibSolver {
public:
    friend class CalibSolverIO;

public:
    using Ptr = std::shared_ptr<CalibSolver>;
    using So3SplineType = ns_ctraj::So3Spline<Configor::Prior::SplineOrder>;
    using PosSplineType = ns_ctraj::RdSpline<3, Configor::Prior::SplineOrder>;
    using SplineBundleType = ns_ctraj::SplineBundle<Configor::Prior::SplineOrder>;

protected:
    CalibParamManagerPtr _parMgr;
    // viewer used to visualize entities in calibration
    ViewerPtr _viewer;
    // indicates whether the solving is finished
    bool _solveFinished;
    // prior knowledge about spatiotemporal parameters used in optimization (if provided)
    SpatialTemporalPrioriPtr _priori;

    /**
     * data
     */
    std::map<std::string, std::vector<EventArrayPtr>> _evMes;
    std::map<std::string, std::vector<IMUFramePtr>> _imuMes;
    // start time, end time
    std::pair<double, double> _dataRawTimestamp;
    std::pair<double, double> _dataAlignedTimestamp;

    /**
     * utilized in event intrinsic calibration
     */
    CircleGrid3DPtr _grid3d;
    // topic, [3d grid, tracked 2d grids]
    std::map<std::string, CircleGridPatternPtr> _extractedPatterns;
    // for a tracked 2d grid pattern
    using ExtractedCirclesVec = std::vector<std::pair<TimeVaryingEllipsePtr, EventArrayPtr>>;
    /**
     * topic, [raw events vector of circles of each tracked 2d grid (id)], example:
     * _extractedPatterns[topic].Grid2D[i][j] -> _rawEventsOfExtractedPatterns[topic].Grid2D[i][j]
     * the raw events of the {j}-th circle of the {i}-th tracked grid pattern of camera {topic}
     */
    std::map<std::string, std::map<int, ExtractedCirclesVec>> _rawEventsOfExtractedPatterns;
    // the topic of the reference event camera;
    std::string _refEvTopic;

    /**
     * utilized in event-inertial calibration
     */
    // from camera frame to world frame
    std::map<std::string, std::vector<ns_ctraj::Posed>> _camPoses;
    // topic, gridId, pose index in '_camPoses'
    std::map<std::string, std::map<int, int>> _gridIdToPoseIdxMap;
    // start time, end  time
    std::vector<std::pair<double, double>> _validTimeSegments;
    std::vector<std::pair<So3SplineType, PosSplineType>> _splineSegments;
    // the full so3 spline
    So3SplineType _fullSo3Spline;
    // options used for ceres-related optimization
    ceres::Solver::Options _ceresOption;
    // visual reprojection pairs
    std::map<std::string, std::vector<VisualProjectionPairPtr>> _evSyncPointProjPairs;
    // visual reprojection pairs (asynchronous, from time-varying circle, circle-center-based)
    std::map<std::string, std::list<VisualProjectionPairPtr>> _evAsyncPointProjPairs;

public:
    CalibSolver(CalibParamManagerPtr parMgr);

    static Ptr Create(const CalibParamManagerPtr &parMgr);

    virtual ~CalibSolver();

    void Process();

protected:
    void LoadDataFromRosBag();

    void OutputDataStatus() const;

    static So3SplineType CreateSo3Spline(double st, double et, double so3Dt, bool printInfo);

    static PosSplineType CreatePosSpline(double st, double et, double posDt, bool printInfo);

    void CreateSplineSegments(double dtSo3, double dtPos, bool printInfo = false);

    void EstimateCameraIntrinsics();

    std::pair<cv::Mat, cv::Mat> EstimateCameraIntrinsicsInitials(
        const std::string &topic,
        int ATTEMPT_COUNT_PER_CAMERA = 50,
        int FRAME_COUNT_PER_ATTEMPT = 20) const;

    static ns_veta::PinholeIntrinsicPtr OrganizeCamParamsOpenCVToVeta(const cv::Mat &cameraMatrix,
                                                                      const cv::Mat &distCoeffs,
                                                                      std::size_t imgWidth,
                                                                      std::size_t imgHeight);

    static std::tuple<cv::Mat, cv::Mat, std::size_t, std::size_t> OrganizeCamParamsVetaToOpenCV(
        const ns_veta::PinholeIntrinsicBrownT2Ptr &intri);

    std::pair<std::vector<ns_ctraj::Posed>, std::map<int, int>> EstimateCameraPoses(
        const std::string &topic) const;

    void RefineCameraIntrinsicsInitials(const std::string &topic, bool fixIntrinsics);

    void InitSplineSegmentsOfRefCamUsingCamPose(bool onlyRefCam,
                                                double SEG_NEIGHBOR,
                                                double SEG_LENGTH);

    void EvCamSpatialTemporalCalib();

    void GridPatternTracking(bool tryLoadAndSaveRes);

    /**
     * initialize (recover) the rotation spline using raw angular velocity measurements from
     * the gyroscope. If multiple gyroscopes (IMUs) are involved, the extrinsic rotations and
     * time offsets would be also recovered
     */
    void InitSo3Spline() const;

    void EventInertialAlignment();

    double BreakTimelineToSegments(double maxNeighborThd,
                                   double maxLengthThd,
                                   const std::optional<std::string> &evTopic = {},
                                   bool optInfo = true);

    void InitSo3SplineSegments();

    void InitPosSpline() const;

    std::optional<int> IsTimeInValidSegment(double timeByBr) const;

    void CreateVisualProjPairsSyncPointBased();

    void CreateVisualProjPairsAsyncPointBased(double dt = 0.005);

    void BatchOptimizations();

protected:
    std::size_t AddGyroFactorToFullSo3Spline(const EstimatorPtr &estimator,
                                             const std::string &imuTopic,
                                             OptOption option,
                                             const std::optional<double> &weight,
                                             const std::optional<double> &dsRate = {}) const;

    std::size_t AddGyroFactorToSplineSegments(const EstimatorPtr &estimator,
                                              const std::string &imuTopic,
                                              OptOption option,
                                              const std::optional<double> &weight,
                                              const std::optional<double> &dsRate = {}) const;

    std::size_t AddAcceFactorToSplineSegments(const EstimatorPtr &estimator,
                                              const std::string &imuTopic,
                                              OptOption option,
                                              const std::optional<double> &weight,
                                              const std::optional<double> &dsRate = {}) const;

    std::size_t AddVisualProjPairsSyncPointBasedToSplineSegments(
        const EstimatorPtr &estimator,
        const std::string &camTopic,
        OptOption option,
        const std::optional<double> &weight) const;

    std::size_t AddVisualProjPairsAsyncPointBasedToSplineSegments(
        const EstimatorPtr &estimator,
        const std::string &camTopic,
        OptOption option,
        const std::optional<double> &weight) const;

private:
    // remove the head data according to the pred
    void EraseSeqHeadData(std::vector<EventArrayPtr> &seq,
                          std::function<bool(const EventArrayPtr &)> pred,
                          const std::string &errorMsg) const;

    // remove the tail data according to the pred
    void EraseSeqTailData(std::vector<EventArrayPtr> &seq,
                          std::function<bool(const EventArrayPtr &)> pred,
                          const std::string &errorMsg) const;

    // remove the head data according to the pred
    void EraseSeqHeadData(std::vector<IMUFramePtr> &seq,
                          std::function<bool(const IMUFramePtr &)> pred,
                          const std::string &errorMsg) const;

    // remove the tail data according to the pred
    void EraseSeqTailData(std::vector<IMUFramePtr> &seq,
                          std::function<bool(const IMUFramePtr &)> pred,
                          const std::string &errorMsg) const;

    std::list<std::list<double>> ContinuousGridTrackingSegments(const std::string &camTopic,
                                                                const double neighborTimeDistThd,
                                                                const double minSegmentThd) const;

    static std::vector<std::pair<double, double>> ContinuousSegments(
        const std::list<std::pair<double, double>> &segBoundary, const double neighborTimeDistThd);

    static int IsTimeInSegment(
        double t, const std::vector<std::pair<double, double>> &mergedSegmentsBoundary);
};
}  // namespace ns_ekalibr

#endif  // CALIB_SOLVER_H
