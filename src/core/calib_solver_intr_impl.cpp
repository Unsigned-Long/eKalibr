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

#include "core/calib_solver.h"
#include "opencv4/opencv2/calib3d.hpp"
#include "spdlog/spdlog.h"
#include "config/configor.h"
#include "core/circle_grid.h"
#include "util/utils.h"
#include "util/utils_tpl.hpp"
#include "filesystem"

namespace ns_ekalibr {

void CalibSolver::EstimateCameraIntrinsics() {
    constexpr int ATTEMPT_COUNT_PER_CAMERA = 20;

    std::map<std::string, cv::Mat> cameraMatrixMap;
    std::map<std::string, cv::Mat> distCoeffsMap;

    for (const auto &[topic, patterns] : _extractedPatterns) {
        spdlog::info("perform intrinsic calibration for camera '{}'...", topic);

        // 2d and 3d grid points
        std::vector<std::vector<cv::Point2f>> gridPoints2DVec;
        gridPoints2DVec.reserve(patterns->GetGrid2d().size());
        for (const auto &grid2d : patterns->GetGrid2d()) {
            gridPoints2DVec.push_back(grid2d->centers);
        }
        const std::size_t GRID_COUNT_PER_ATTEMPT = std::min(50UL, patterns->GetGrid2d().size());

        std::vector gridPoints3DVec(GRID_COUNT_PER_ATTEMPT, patterns->GetGrid3d()->points);

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

#pragma omp parallel for
        for (int i = 0; i < ATTEMPT_COUNT_PER_CAMERA; i++) {
            std::default_random_engine generator(
                std::chrono::system_clock::now().time_since_epoch().count());
            auto grids = SamplingWoutReplace2(generator, gridPoints2DVec, GRID_COUNT_PER_ATTEMPT);

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
                cv::CALIB_USE_LU,
                // Termination criteria for the iterative optimization algorithm
                cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 1E-6));
            spdlog::info("the overall rmes (re-projection error) in {:02}-th attempt: {:.3f}", i,
                         rmse[i]);
        }

        auto minIter = std::min_element(rmse, rmse + ATTEMPT_COUNT_PER_CAMERA);
        const std::size_t idx = std::distance(rmse, minIter);

        spdlog::info(
            "the {:02}-th attempt obtains the minimum rmse: {:.3f}, treat its results as "
            "intrinsics",
            idx, rmse[idx]);

        // save results to disk
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

        // record
        cameraMatrixMap[topic] = cameraMatrix[idx];
        distCoeffsMap[topic] = distCoeffs[idx];
    }

    for (const auto &[topic, _] : Configor::DataStream::EventTopics) {
        auto cameraMatrix = cameraMatrixMap.at(topic);
        auto distCoeffs = distCoeffsMap.at(topic);
        // todo: organize as veta intrinsics
    }
}
}  // namespace ns_ekalibr