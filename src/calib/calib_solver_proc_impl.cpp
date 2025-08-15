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

#include "calib/calib_solver.h"
#include "spdlog/spdlog.h"
#include "util/tqdm.h"
#include "config/configor.h"
#include "viewer/viewer.h"
#include "core/norm_flow.h"
#include "calib/calib_param_mgr.h"
#include "calib/calib_solver_io.h"

namespace ns_ekalibr {

void CalibSolver::Process() {
    /**
     * load event data from the rosbag and align timestamps (temporal normalization)
     */
    spdlog::info("load data from the rosbag and align timestamps...");
    this->LoadDataFromRosBag();

    /**
     * perform circle grid pattern extraction from raw event data stream:
     * (1) perform norm flow estimation
     * (2) perform clustering
     * (3) identity cirlce clusters
     * (4) fit time-varying ciecles using least-squares estimation
     */
    this->GridPatternTracking(true);
    _evMes.clear();  // "we don't talk anymore...", I mean the '_evMes'.

    /**
     * we want to keep al added entities in the viewer, and do not just keep a const count of them
     */
    if (Configor::Preference::Visualization) {
        _viewer->ClearViewer();
        _viewer->ResetViewerCamera();
        _viewer->SetKeptEntityCount(-1);
    }

    /**
     * perform intrinsic calibration using opencv
     */
    this->EstimateCameraIntrinsics();
    _parMgr->ShowParamStatus();
    CalibSolverIO::SaveStageCalibParam(_parMgr, "camera_intrinsics_calib");

    /**
     * Currently, we only support intrinsic calibration for event cameras. For other types of
     * calibration, such as spatiotemporal calibration for multi-camera systems and event-inertial
     * systems, please stay tuned.
     */

    /**
     * calibrate spatiotemporal parameters of events camera
     */
    if (Configor::DataStream::IMUTopics.empty()) {
        if (Configor::DataStream::EventTopics.size() > 1) {
            if (Configor::Preference::Visualization) {
                _viewer->SetStates(&_splineSegments, _parMgr, _grid3d);
            }
            this->EvCamSpatialTemporalCalib();
            _parMgr->ShowParamStatus();
            CalibSolverIO::SaveStageCalibParam(_parMgr, "multi_camera_calib");

            _solveFinished = true;
            return;
        } else {
            // Configor::DataStream::EventTopics.size() <= 1, only perform intrinsic calibration
        }
    } else {
        if (Configor::DataStream::EventTopics.empty()) {
            // exit
            _solveFinished = true;
            return;
        }
        // perform visual-inertial calibration
    }

    // this->GridPatternTracking(false, true);

    /**
     * we want to keep al added entities in the viewer, and do not just keep a const count of them
     */
    if (Configor::Preference::Visualization) {
        _viewer->ClearViewer();
        _viewer->ResetViewerCamera();
    }

    // create so3 spline given start and end times, knot distances
    _fullSo3Spline = CreateSo3Spline(_dataAlignedTimestamp.first, _dataAlignedTimestamp.second,
                                     Configor::Prior::DecayTimeOfActiveEvents * 2.5, true);

    if (Configor::Preference::Visualization) {
        _viewer->SetStates(nullptr, _parMgr, nullptr);
    }

    /* initialize (recover) the rotation spline using raw angular velocity measurements from the
     * gyroscope. If multiple gyroscopes (IMUs) are involved, the extrinsic rotations and time
     * offsets would be also recovered
     */
    this->InitSo3Spline();
    // _parMgr->ShowParamStatus();
    CalibSolverIO::SaveStageCalibParam(_parMgr, "visual_inertial_calib_0_so3_spline_init");

    /**
     * perform sensor-inertial alignment to recover the gravity vector and extrinsic translations.
     */
    this->EventInertialAlignment();
    // _parMgr->ShowParamStatus();
    CalibSolverIO::SaveStageCalibParam(_parMgr, "visual_inertial_calib_1_visual_inertial_align");

    if (Configor::Preference::Visualization) {
        _viewer->SetStates(&_splineSegments, _parMgr, _grid3d);
    }

    /**
     * Due to the possibility that the checkerboard may be intermittently tracked (potentially due
     * to insufficient stimulation leading to an inadequate number of events, or the checkerboard
     * moving out of the field of view), it is necessary to identify the continuous segments for
     * subsequent calibration.
     */
    // const double SEG_NEIGHBOR = Configor::Prior::DecayTimeOfActiveEvents * 5; /*neighbor*/
    // const double SEG_LENGTH = Configor::Prior::DecayTimeOfActiveEvents * 50;  /*length*/
    // this->BreakTimelineToSegments(SEG_NEIGHBOR /*neighbor*/, SEG_LENGTH /*len*/);
    this->BreakTimelineToSegments(0.5 /*neighbor*/, 1.0 /*len*/);
    this->CreateSplineSegments(Configor::Prior::DecayTimeOfActiveEvents * 2.5,
                               Configor::Prior::DecayTimeOfActiveEvents * 2.5);
    this->InitSo3SplineSegments();

    /**
     * recover the linear scale spline using quantities from the one-shot sensor-inertial alignment
     */
    this->InitPosSpline();
    _parMgr->ShowParamStatus();
    CalibSolverIO::SaveStageCalibParam(_parMgr, "visual_inertial_calib_2_pos_spline_init");

    /**
     * perform several batch optimizaitons to refine all initialized states to global optimal ones
     */
    this->BatchOptimizations();
    _parMgr->ShowParamStatus();
    CalibSolverIO::SaveStageCalibParam(_parMgr, "visual_inertial_calib");

    _solveFinished = true;
}
}  // namespace ns_ekalibr