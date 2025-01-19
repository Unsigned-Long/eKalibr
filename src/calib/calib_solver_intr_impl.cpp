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
#include "opencv4/opencv2/calib3d.hpp"
#include "spdlog/spdlog.h"
#include "config/configor.h"
#include "core/circle_grid.h"
#include "util/utils.h"
#include "util/utils_tpl.hpp"
#include "filesystem"
#include "util/tqdm.h"
#include "calib/calib_param_mgr.h"
#include <tiny-viewer/object/camera.h>
#include <veta/camera/pinhole_brown.h>
#include <viewer/viewer.h>
#include "calib/estimator.h"
#include "factor/visual_projection_factor.hpp"
#include <util/status.hpp>
#include "magic_enum_flags.hpp"

namespace ns_ekalibr {

void CalibSolver::EstimateCameraIntrinsics() {
    for (const auto &[topic, config] : Configor::DataStream::EventTopics) {
        if (config.NeedEstIntrinsics()) {
            // compute intrinsics initials for each uncalibrated event camera using OpenCV
            const auto &[cameraMatrix, distCoeffs] =
                this->EstimateCameraIntrinsicsInitials(topic, 50, 20);

            // organize eKalibr-format intrinsics using intrinsic matrix from OpenCV
            _parMgr->INTRI.Camera.at(topic) = OrganizeCamParamsOpenCVToVeta(
                cameraMatrix, distCoeffs, _parMgr->INTRI.Camera.at(topic)->imgWidth,
                _parMgr->INTRI.Camera.at(topic)->imgHeight);
        } else {
            spdlog::warn("intrinsics of camera '{}' are not estimated as have been loaded!", topic);
        }

        // compute poses for each event camera
        const auto &poseRes = this->EstimateCameraPoses(topic);
        _camPoses[topic] = poseRes.first;
        _gridIdToPoseIdxMap[topic] = poseRes.second;

        // refine intrinsics initials using reprojection-based bundle adjustment
        this->RefineCameraIntrinsicsInitials(topic, !config.NeedEstIntrinsics());
    }

    // visualization
    if (!Configor::Preference::Visualization) {
        return;
    }
    const auto scale = static_cast<float>(Configor::Preference::SplineViewerSpatialScale);
    for (const auto &[topic, curCamPoses] : _camPoses) {
        // grid pattern
        _viewer->AddGridPattern(_grid3d->points,
                                static_cast<float>(Configor::Prior::CirclePattern.Radius()), scale,
                                ns_viewer::Colour::Black());

        // cameras
        std::vector<ns_viewer::Entity::Ptr> entities;
        entities.reserve(curCamPoses.size());
        for (const auto &pose : curCamPoses) {
            auto vPose = ns_viewer::Posed(pose.so3.matrix(), pose.t * scale).cast<float>();
            auto color = ns_viewer::Entity::GetUniqueColour();
            entities.push_back(ns_viewer::CubeCamera::Create(vPose, color, 0.05f));
        }
        _viewer->AddEntityLocal(entities);
    }
}

std::pair<cv::Mat, cv::Mat> CalibSolver::EstimateCameraIntrinsicsInitials(
    const std::string &topic, int ATTEMPT_COUNT_PER_CAMERA, int FRAME_COUNT_PER_ATTEMPT) const {
    /**
     * compute intrinsics initials for each uncalibrated event camera using OpenCV
     * output: 'cameraMatrixMap', 'distCoeffsMap'
     */
    spdlog::info("initialize intrinsic calibration for camera '{}'...", topic);
    const auto &patterns = _extractedPatterns.at(topic);

    // 2d and 3d grid points
    std::vector<std::vector<cv::Point2f>> gridPoints2DVec;
    gridPoints2DVec.reserve(patterns->GetGrid2d().size());
    for (const auto &grid2d : patterns->GetGrid2d()) {
        gridPoints2DVec.push_back(grid2d->centers);
    }
    const std::size_t GRID3D_COUNT_PER_ATTEMPT =
        std::min(FRAME_COUNT_PER_ATTEMPT, static_cast<int>(patterns->GetGrid2d().size()));

    std::vector gridPoints3DVec(GRID3D_COUNT_PER_ATTEMPT, patterns->GetGrid3d()->points);

    // image size
    const auto &config = Configor::DataStream::EventTopics.at(topic);
    auto imgSize = cv::Size(config.Width, config.Height);

    cv::Mat cameraMatrix[ATTEMPT_COUNT_PER_CAMERA];
    cv::Mat distCoeffs[ATTEMPT_COUNT_PER_CAMERA];

    std::vector<cv::Mat> rVecs[ATTEMPT_COUNT_PER_CAMERA], tVecs[ATTEMPT_COUNT_PER_CAMERA];
    cv::Mat stdDeviationsIntrinsics[ATTEMPT_COUNT_PER_CAMERA];
    cv::Mat stdDeviationsExtrinsics[ATTEMPT_COUNT_PER_CAMERA];
    cv::Mat perViewErrors[ATTEMPT_COUNT_PER_CAMERA];

    double rmse[ATTEMPT_COUNT_PER_CAMERA];

    auto bar = std::make_shared<tqdm>();
    int finishedCount = 0;
#pragma omp parallel for num_threads(3) default(none)                                         \
    shared(gridPoints2DVec, gridPoints3DVec, imgSize, cameraMatrix, distCoeffs, rVecs, tVecs, \
               stdDeviationsIntrinsics, stdDeviationsExtrinsics, perViewErrors, rmse, bar,    \
               GRID3D_COUNT_PER_ATTEMPT, ATTEMPT_COUNT_PER_CAMERA, finishedCount)
    for (int i = 0; i < ATTEMPT_COUNT_PER_CAMERA; i++) {
        std::default_random_engine generator(
            std::chrono::system_clock::now().time_since_epoch().count());
        auto grids = SamplingWoutReplace2(generator, gridPoints2DVec, GRID3D_COUNT_PER_ATTEMPT);

        rmse[i] = cv::calibrateCamera(
            // a vector of vectors of calibration pattern points in the calibration pattern
            // coordinate space  (e.g. std::vector<std::vector<cv::Vec3f>>)
            gridPoints3DVec,
            // a vector of vectors of the projections of calibration pattern points (e.g.
            // std::vector<std::vector<cv::Vec2f>>)
            grids,
            // Size of the image used only to initialize the intrinsic camera matrix
            imgSize,
            // Output 3x3 floating-point camera matrix: [fx, 0, c_x; 0, fy, cy; 0, 0, 1;]
            cameraMatrix[i],
            // Output vector of distortion coefficients (k1, k2, p1, p2[, k3[, k4, k5, k6
            // [, s1, s2, s3, s4[, tau_x, tau_y]]]]) of 4, 5, 8, 12 or 14 elements
            distCoeffs[i],
            // Output vector of rotation vectors (see Rodrigues) estimated for each pattern view
            // (e.g. std::vector<cv::Mat>>)
            rVecs[i],
            // Output vector of translation vectors estimated for each pattern view
            tVecs[i],
            // Output vector of standard deviations estimated for intrinsic parameters
            stdDeviationsIntrinsics[i],
            // Output vector of standard deviations estimated for extrinsic parameters
            stdDeviationsExtrinsics[i],
            // Output vector of the RMS re-projection error estimated for each pattern view
            perViewErrors[i],
            // calibration flag
            cv::CALIB_USE_QR,
            // Termination criteria for the iterative optimization algorithm
            cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 1E-6));
        // spdlog::info("the overall rmes (re-projection error) in {:02}-th attempt: {:.3f}", i,
        //              rmse[i]);

#pragma omp critical
        {
            bar->progress(++finishedCount, ATTEMPT_COUNT_PER_CAMERA);
        }
    }
    bar->finish();

    auto minIter = std::min_element(rmse, rmse + ATTEMPT_COUNT_PER_CAMERA);
    const std::size_t idx = std::distance(rmse, minIter);

    spdlog::info(
        "the {:02}-th attempt obtains the minimum rmse: {:.3f}, treat its results as "
        "intrinsics",
        idx, rmse[idx]);

#if 0
        // save results to disk (these intrinsics are rough ones!!!)
        auto path = GetDiskPathOfOpenCVIntrinsicCalibRes(topic);
        if (std::filesystem::path(path).extension() ==
            Configor::Preference::FileExtension.at(CerealArchiveType::Enum::BINARY)) {
            path = std::filesystem::path(path).replace_extension(
                Configor::Preference::FileExtension.at(CerealArchiveType::Enum::YAML));
            }
        spdlog::info("save intrinsic results of camera '{}' to '{}'...", topic, path);
        cv::FileStorage fs(path, cv::FileStorage::WRITE);
        fs << "cameraMatrix" << cameraMatrix[idx];
        fs << "distCoeffs" << distCoeffs[idx];
        fs << "stdDeviationsIntrinsics" << stdDeviationsIntrinsics[idx];
        fs << "perViewErrors" << perViewErrors[idx];
        fs << "rVecs" << rVecs[idx];
        fs << "tVecs" << tVecs[idx];
        fs << "stdDeviationsExtrinsics" << stdDeviationsExtrinsics[idx];

        fs.release();
#endif
    // record
    cv::Mat cameraMatrixWinner = cameraMatrix[idx];
    cv::Mat distCoeffsWinner = distCoeffs[idx];

    return {cameraMatrixWinner, distCoeffsWinner};
}

ns_veta::PinholeIntrinsicPtr CalibSolver::OrganizeCamParamsOpenCVToVeta(const cv::Mat &cameraMatrix,
                                                                        const cv::Mat &distCoeffs,
                                                                        std::size_t imgWidth,
                                                                        std::size_t imgHeight) {
    return ns_veta::PinholeIntrinsicBrownT2::Create(static_cast<int>(imgWidth),     // w
                                                    static_cast<int>(imgHeight),    // h
                                                    cameraMatrix.at<double>(0, 0),  // fx
                                                    cameraMatrix.at<double>(1, 1),  // fy
                                                    cameraMatrix.at<double>(0, 2),  // cx
                                                    cameraMatrix.at<double>(1, 2),  // cy
                                                    distCoeffs.at<double>(0),       // k1
                                                    distCoeffs.at<double>(1),       // k2
                                                    distCoeffs.at<double>(4),       // k3
                                                    distCoeffs.at<double>(2),       // p1
                                                    distCoeffs.at<double>(3)        // p2
    );
}

std::tuple<cv::Mat, cv::Mat, std::size_t, std::size_t> CalibSolver::OrganizeCamParamsVetaToOpenCV(
    const ns_veta::PinholeIntrinsicBrownT2Ptr &intri) {
    const auto &par = intri->GetParams();
    cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    cameraMatrix.at<double>(0, 0) = par.at(0);  // fx
    cameraMatrix.at<double>(1, 1) = par.at(1);  // fy
    cameraMatrix.at<double>(0, 2) = par.at(2);  // cx
    cameraMatrix.at<double>(1, 2) = par.at(3);  // cy

    cv::Mat distCoeffs = cv::Mat(1, 5, CV_64F);
    distCoeffs.at<double>(0) = par.at(4);  // k1
    distCoeffs.at<double>(1) = par.at(5);  // k2
    distCoeffs.at<double>(4) = par.at(6);  // k3
    distCoeffs.at<double>(2) = par.at(7);  // p1
    distCoeffs.at<double>(3) = par.at(8);  // p2

    return {cameraMatrix, distCoeffs, intri->imgWidth, intri->imgHeight};
}

std::pair<std::vector<ns_ctraj::Posed>, std::map<int, int>> CalibSolver::EstimateCameraPoses(
    const std::string &topic) const {
    /**
     * estimate poses for each camera using intrinsic intrinics
     *  input: 'cameraMatrixMap' and 'distCoeffsMap'
     * output: 'poseVecMap'
     */
    spdlog::info("perform PnP solving to obtain poses of all timestamps for camera '{}'...", topic);

    auto intri = std::dynamic_pointer_cast<ns_veta::PinholeIntrinsicBrownT2>(
        _parMgr->INTRI.Camera.at(topic));
    if (intri == nullptr) {
        throw Status(Status::CRITICAL,
                     "only intrinsics type 'ns_veta::PinholeIntrinsicBrownT2' is supported in "
                     "eKalibr!!!");
    }
    const auto &[cameraMatrix, distCoeffs, imgWidth, imgHeight] =
        this->OrganizeCamParamsVetaToOpenCV(intri);

    const auto &patterns = _extractedPatterns.at(topic);

    // topic, timestamp, rVec, tVec
    std::list<std::tuple<CircleGrid2D::Ptr, cv::Mat, cv::Mat>> curPoseVec;

    auto bar = std::make_shared<tqdm>();
    int idx = 0;
    for (const auto &grid2d : patterns->GetGrid2d()) {
        bar->progress(idx++, static_cast<int>(patterns->GetGrid2d().size()));
        // The estimated pose is thus the rotation (rvec) and the translation (tvec) vectors
        // that allow transforming a 3D point expressed in the world frame into the camera
        // frame.
        cv::Mat rVecs, tVecs;
        bool res = cv::solvePnP(
            // Array of object points in the object coordinate space
            patterns->GetGrid3d()->points,
            // Array of corresponding image points
            grid2d->centers,
            // camera intrinsics
            cameraMatrix,
            // Input vector of distortion coefficients
            distCoeffs,
            // Output rotation vector (see @ref Rodrigues ) that, together with tvec, brings
            // points from the model coordinate system to the camera coordinate system
            rVecs,
            // Output translation vector
            tVecs,
            // Parameter used for #SOLVEPNP_ITERATIVE. If true, the function uses the provided
            // rvec and tvec values as initial approximations of the rotation and translation
            // vectors, respectively, and further optimizes them.
            false,
            // Iterative method is based on a Levenberg-Marquardt optimization
            cv::SOLVEPNP_ITERATIVE);
        if (!res) {
            spdlog::warn("PnP solving temporally stamped at '{:.3f}' failed!!!", grid2d->timestamp);
        } else {
            curPoseVec.emplace_back(grid2d, rVecs, tVecs);
        }
    }
    bar->finish();

    std::vector<ns_ctraj::Posed> poseVecRes;
    std::map<int, int> gridIdToPoseIdxMap;
    int poseIdx = 0;
    for (const auto &[grid2d, rVec, tVec] : curPoseVec) {
        // rotation
        cv::Mat rotMatrix;
        cv::Rodrigues(rVec, rotMatrix);
        Eigen::Matrix3d Rot_WtoCj;
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                Rot_WtoCj(i, j) = rotMatrix.at<double>(i, j);
            }
        }
        Rot_WtoCj = Sophus::makeRotationMatrix(Rot_WtoCj);

        // translation
        Eigen::Vector3d Pos_WinCj;
        for (int i = 0; i < 3; i++) {
            Pos_WinCj(i) = tVec.at<double>(i);
        }

        // inverse
        Eigen::Matrix3d R = Rot_WtoCj.transpose();
        Eigen::Vector3d t = -Rot_WtoCj.transpose() * Pos_WinCj;

        // storage
        poseVecRes.emplace_back(R, t, grid2d->timestamp);
        gridIdToPoseIdxMap.insert({grid2d->id, poseIdx++});
    }
    return {poseVecRes, gridIdToPoseIdxMap};
}

void CalibSolver::RefineCameraIntrinsicsInitials(const std::string &topic, bool fixIntrinsics) {
    OptOption option = OptOption::NONE;
    if (!fixIntrinsics) {
        option = OptOption::OPT_CAM_PRINCIPAL_POINT | OptOption::OPT_CAM_FOCAL_LEN |
                 OptOption::OPT_CAM_DIST_COEFFS;
    }

    spdlog::info("refine intrinsics using non-linear least-squares optimization for camera '{}'...",
                 topic);
    auto estimator = Estimator::Create(_parMgr);

    const auto &gridIdToPoseIdxMap = _gridIdToPoseIdxMap.at(topic);
    auto &poseVec = _camPoses.at(topic);

    const auto &patterns = _extractedPatterns.at(topic);
    const auto &grid2dVec = patterns->GetGrid2d();
    const auto &grid3d = patterns->GetGrid3d();

    for (const auto &grid2d : grid2dVec) {
        for (int i = 0; i < static_cast<int>(grid2d->centers.size()); ++i) {
            const auto &center = grid2d->centers.at(i);
            const Eigen::Vector2d pixel(center.x, center.y);

            const auto &point3d = grid3d->points.at(i);
            const Eigen::Vector3d point(point3d.x, point3d.y, point3d.z);

            auto pair = VisualProjectionPair::Create(grid2d->timestamp, point, pixel);

            auto &pose = poseVec.at(gridIdToPoseIdxMap.at(grid2d->id));
            estimator->AddVisualDiscreteProjectionFactor(&pose.so3, &pose.t, topic, pair, option,
                                                         1.0);
        }
    }

    auto sum = estimator->Solve(_ceresOption, nullptr);
    spdlog::info("here is the summary:\n{}\n", sum.BriefReport());
}

}  // namespace ns_ekalibr