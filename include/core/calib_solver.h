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

class CalibSolver {
public:
    using Ptr = std::shared_ptr<CalibSolver>;
    using SplineBundleType = ns_ctraj::SplineBundle<Configor::Prior::SplineOrder>;

protected:
    CalibParamManagerPtr _parMgr;
    SplineBundleType::Ptr _splines;
    // options used for ceres-related optimization
    ceres::Solver::Options _ceresOption;
    // viewer used to visualize entities in calibration
    ViewerPtr _viewer;
    // indicates whether the solving is finished
    bool _solveFinished;

    std::map<std::string, std::vector<EventArrayPtr>> _evMes;
    std::map<std::string, std::vector<IMUFramePtr>> _imuMes;
    // start time, end time
    std::pair<double, double> _dataRawTimestamp;
    std::pair<double, double> _dataAlignedTimestamp;
    ns_viewer::Posef _viewCamPose;

    std::map<std::string, CircleGridPatternPtr> _extractedPatterns;
    // from camera frame to world frame
    std::map<std::string, std::vector<ns_ctraj::Posed>> _camPoses;

public:
    CalibSolver(CalibParamManagerPtr parMgr);

    static Ptr Create(const CalibParamManagerPtr &parMgr);

    virtual ~CalibSolver();

    void Process();

protected:
    void LoadDataFromRosBag();

    void OutputDataStatus() const;

    static SplineBundleType::Ptr CreateSplineBundle(double st,
                                                    double et,
                                                    double so3Dt,
                                                    double scaleDt);

    static std::string GetDiskPathOfExtractedGridPatterns(const std::string &topic);

    static std::string GetDiskPathOfOpenCVIntrinsicCalibRes(const std::string &topic);

    void EstimateCameraIntrinsics();

    /**
     * initialize (recover) the rotation spline using raw angular velocity measurements from
     * the gyroscope. If multiple gyroscopes (IMUs) are involved, the extrinsic rotations and
     * time offsets would be also recovered
     */
    void InitSO3Spline() const;

    void EventInertialAlignment() const;

protected:
    /**
     * add gyroscope factors for the IMU to the estimator
     * @param estimator the estimator
     * @param imuTopic the ros topic of this IMU
     * @param option the option for the optimization
     */
    void AddGyroFactor(EstimatorPtr &estimator,
                       const std::string &imuTopic,
                       OptOption option) const;

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
};
}  // namespace ns_ekalibr

#endif  // CALIB_SOLVER_H
