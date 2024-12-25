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

#include "core/norm_flow.h"
#include "core/sae.h"
#include "sensor/event.h"
#include "util/utils.h"
#include "opencv4/opencv2/imgproc.hpp"
#include "opengv/sac/Ransac.hpp"
#include <config/configor.h>
#include <opencv2/imgcodecs.hpp>

namespace ns_ekalibr {
/**
 * NormFlow
 */
NormFlow::NormFlow(double timestamp, const Eigen::Vector2i &p, const Eigen::Vector2d &nf)
    : timestamp(timestamp),
      p(p),
      nf(nf),
      nfNorm(nf.norm()),
      nfDir(nf.normalized()) {}

NormFlow::Ptr NormFlow::Create(double timestamp,
                               const Eigen::Vector2i &p,
                               const Eigen::Vector2d &nf) {
    return std::make_shared<NormFlow>(timestamp, p, nf);
}

/**
 * EventNormFlow::NormFlowPack
 */
std::list<Event::Ptr> EventNormFlow::NormFlowPack::ActiveEvents(double dt) const {
    std::list<Event::Ptr> events;
    const int rows = rawTimeSurfaceMap.rows;
    const int cols = rawTimeSurfaceMap.cols;
    for (int ey = 0; ey < rows; ey++) {
        for (int ex = 0; ex < cols; ex++) {
            const auto &et = rawTimeSurfaceMap.at<double>(ey, ex);
            // whether this pixel is assigned
            if (et < 1E-3) {
                continue;
            }
            // time range
            if (dt > 0.0 && timestamp - et > dt) {
                continue;
            }
            const auto &ep = polarityMap.at<uchar>(ey, ex);
            events.push_back(Event::Create(et, {ex, ey}, ep));
        }
    }
    return events;
}

std::list<Event::Ptr> EventNormFlow::NormFlowPack::NormFlowInlierEvents() const {
    std::list<Event::Ptr> events;
    for (const auto &[nf, inliers] : this->nfs) {
        for (const auto &[ex, ey, et] : inliers) {
            const auto &ep = polarityMap.at<uchar>(ey, ex);
            events.push_back(Event::Create(et, {ex, ey}, ep));
        }
    }

    return events;
}

cv::Mat EventNormFlow::NormFlowPack::Visualization(double dt, bool save) const {
    cv::Mat m1;
    cv::hconcat(nfSeedsImg, nfsImg, m1);

    cv::Mat m2;
    cv::Mat accEvMat = AccumulativeEventMat(dt);
    cv::Mat nfInlierEvMat = NormFlowInlierEventMat();
    cv::hconcat(accEvMat, nfInlierEvMat, m2);

    cv::Mat m3;
    cv::vconcat(m1, m2, m3);

    if (save) {
        static int count = 0;
        cv::imwrite(
            Configor::DataStream::DebugPath + "/nfSeedsImg-" + std::to_string(count) + ".png",
            nfSeedsImg);
        cv::imwrite(Configor::DataStream::DebugPath + "/nfsImg-" + std::to_string(count) + ".png",
                    nfsImg);
        cv::imwrite(Configor::DataStream::DebugPath + "/accEvMat-" + std::to_string(count) + ".png",
                    accEvMat);
        cv::imwrite(
            Configor::DataStream::DebugPath + "/nfInlierEvMat-" + std::to_string(count) + ".png",
            nfInlierEvMat);
        ++count;
    }

    return m3;
}

cv::Mat EventNormFlow::NormFlowPack::InliersOccupyMat() const {
    const int rows = rawTimeSurfaceMap.rows;
    const int cols = rawTimeSurfaceMap.cols;
    cv::Mat inliersOccupy = cv::Mat::zeros(rows, cols, CV_8UC1);
    for (const auto &[nf, inliers] : this->nfs) {
        for (const auto &[ex, ey, et] : inliers) {
            inliersOccupy.at<uchar>(ey /*row*/, ex /*col*/) = 255;
        }
    }
    return inliersOccupy;
}

cv::Mat EventNormFlow::NormFlowPack::NormFlowInlierEventMat() const {
    cv::Mat nfEventMat(nfSeedsImg.size(), CV_8UC3, cv::Scalar(0, 0, 0));
    const cv::Vec3b blue(255, 0, 0), red(0, 0, 255);
    for (const auto &event : this->NormFlowInlierEvents()) {
        Event::PosType::Scalar ex = event->GetPos()(0), ey = event->GetPos()(1);
        nfEventMat.at<cv::Vec3b>(ey, ex) = event->GetPolarity() ? blue : red;
    }
    return nfEventMat;
}

cv::Mat EventNormFlow::NormFlowPack::AccumulativeEventMat(double dt) const {
    const cv::Vec3b blue(255, 0, 0), red(0, 0, 255);
    cv::Mat actEventMat(nfSeedsImg.size(), CV_8UC3, cv::Scalar(0, 0, 0));

    for (const auto &event : this->ActiveEvents(dt)) {
        Event::PosType::Scalar ex = event->GetPos()(0), ey = event->GetPos()(1);
        actEventMat.at<cv::Vec3b>(ey, ex) = event->GetPolarity() ? blue : red;
    }
    return actEventMat;
}

std::list<NormFlow::Ptr> EventNormFlow::NormFlowPack::TemporallySortedNormFlows() const {
    std::list<NormFlow::Ptr> nfsList = NormFlows();
    nfsList.sort([&](const NormFlow::Ptr &a, const NormFlow::Ptr &b) {
        return a->timestamp < b->timestamp;
    });
    return nfsList;
}

std::list<NormFlow::Ptr> EventNormFlow::NormFlowPack::NormFlows() const {
    std::list<NormFlow::Ptr> nfsList;
    for (const auto &[nf, inliers] : this->nfs) {
        nfsList.push_back(nf);
    }
    return nfsList;
}

int EventNormFlow::NormFlowPack::Rows() const { return rawTimeSurfaceMap.rows; }

int EventNormFlow::NormFlowPack::Cols() const { return rawTimeSurfaceMap.cols; }

/**
 * EventNormFlow
 */
EventNormFlow::EventNormFlow(const ActiveEventSurfacePtr &sea)
    : _sea(sea) {}

EventNormFlow::NormFlowPack::Ptr EventNormFlow::ExtractNormFlows(double decaySec,
                                                                 int winSize,
                                                                 int neighborDist,
                                                                 double goodRatioThd,
                                                                 double timeDistEventToPlaneThd,
                                                                 int ransacMaxIter) const {
    // CV_64FC1
    auto [rtsMat, pMat] = _sea->RawTimeSurface(true);
    // CV_8UC1
    auto tsImg = _sea->DecayTimeSurface(true, 0, decaySec);
    const double timeLast = _sea->GetTimeLatest();
    cv::Mat mask;
    cv::inRange(rtsMat, std::max(1E-3, timeLast - decaySec), timeLast, mask);

    cv::cvtColor(tsImg, tsImg, cv::COLOR_GRAY2BGR);
    auto nfsImg = tsImg.clone();
    auto nfSeedsImg = tsImg.clone();

    const int ws = winSize;
    const int subTravSize = std::max(winSize, neighborDist);
    const int winSampleCount = (2 * ws + 1) * (2 * ws + 1);
    const int winSampleCountThd = winSampleCount * goodRatioThd;
    const int rows = mask.rows;
    const int cols = mask.cols;
    cv::Mat occupy = cv::Mat::zeros(rows, cols, CV_8UC1);
    std::map<NormFlow::Ptr, std::vector<std::tuple<int, int, double>>> nfsInliers;

#define OUTPUT_PLANE_FIT 0
#if OUTPUT_PLANE_FIT
    std::list<std::pair<Eigen::Vector3d, std::list<std::tuple<double, double, double>>>> drawData;
#endif
    for (int y = subTravSize; y < mask.rows - subTravSize; y++) {
        for (int x = subTravSize; x < mask.cols - subTravSize; x++) {
            if (mask.at<uchar>(y /*row*/, x /*col*/) != 255) {
                continue;
            }
            // for this window, obtain the values [x, y, timestamp]
            std::vector<std::tuple<int, int, double>> inRangeData;
            double timeCen = 0.0;
            inRangeData.reserve(winSampleCount);
            bool jumpCurPixel = false;
            for (int dy = -subTravSize; dy <= subTravSize; ++dy) {
                for (int dx = -subTravSize; dx <= subTravSize; ++dx) {
                    int nx = x + dx;
                    int ny = y + dy;

                    // this pixel is in neighbor range
                    if (std::abs(dx) <= neighborDist && std::abs(dy) <= neighborDist) {
                        if (occupy.at<uchar>(ny /*row*/, nx /*col*/) == 255) {
                            // this pixl has been occupied, thus the current pixel would not be
                            // considered in norm flow estimation
                            jumpCurPixel = true;
                            break;
                        }
                    }

                    // this pixel is not considered in the window
                    if (std::abs(dx) > ws || std::abs(dy) > ws) {
                        continue;
                    }

                    // in window but not involved in norm flow estimation
                    if (mask.at<uchar>(ny /*row*/, nx /*col*/) != 255) {
                        continue;
                    }

                    double timestamp = rtsMat.at<double>(ny /*row*/, nx /*col*/);
                    inRangeData.emplace_back(nx, ny, timestamp);
                    if (nx == x && ny == y) {
                        timeCen = timestamp;
                    }
                }
                if (jumpCurPixel) {
                    break;
                }
            }
            // data in this window is sufficient
            if (jumpCurPixel || static_cast<int>(inRangeData.size()) < winSampleCountThd) {
                continue;
            }
            /**
             * drawing
             */
            nfSeedsImg.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 255);  // selected but not verified
            occupy.at<uchar>(y /*row*/, x /*col*/) = 255;

            // try fit planes using ransac
            auto centeredInRangeData = Centralization(inRangeData);
            opengv::sac::Ransac<EventLocalPlaneSacProblem> ransac;
            std::shared_ptr<EventLocalPlaneSacProblem> probPtr(
                new EventLocalPlaneSacProblem(centeredInRangeData));
            ransac.sac_model_ = probPtr;
            // the point to plane threshold in temporal domain
            ransac.threshold_ = timeDistEventToPlaneThd;
            ransac.max_iterations_ = ransacMaxIter;
            auto res = ransac.computeModel();

            if (!res || ransac.inliers_.size() / (double)inRangeData.size() < goodRatioThd) {
                continue;
            }
            // success
            Eigen::Vector3d abc;
            probPtr->optimizeModelCoefficients(ransac.inliers_, ransac.model_coefficients_, abc);

            // 'abd' is the params we are interested in
            const double dtdx = -abc(0), dtdy = -abc(1);
            Eigen::Vector2d nf = 1.0 / (dtdx * dtdx + dtdy * dtdy) * Eigen::Vector2d(dtdx, dtdy);

            if (nf.squaredNorm() > 4E3 * 4E3) {
                // the fitted plane is orthogonal to the t-axis, todo: a better way?
                continue;
            }

            // inliers of the norm flow
            std::vector<std::tuple<int, int, double>> inlierData;
            inlierData.reserve(ransac.inliers_.size());
            for (int idx : ransac.inliers_) {
                inlierData.emplace_back(inRangeData.at(idx));
            }
            auto newNormFlow = NormFlow::Create(timeCen, Eigen::Vector2i{x, y}, nf);
            nfsInliers[newNormFlow] = inlierData;

            /**
             * drawing
             */
            nfSeedsImg.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 255, 0);  // selected and verified
            DrawLineOnCVMat(nfsImg, Eigen::Vector2d{x, y} + 0.01 * nf, {x, y});

#if OUTPUT_PLANE_FIT
            std::list<std::tuple<double, double, double>> centeredInliers;
            for (int idx : ransac.inliers_) {
                centeredInliers.push_back(centeredInRangeData.at(idx));
            }
            drawData.push_back({abc, centeredInliers});
#endif
        }
    }
#if OUTPUT_PLANE_FIT
    auto path = Configor::DataStream::DebugPath;
    static int count = 0;
    if (std::filesystem::exists(path)) {
        auto filename = path + "/event_local_planes" + std::to_string(count++) + ".yaml";
        std::ofstream ofs(filename);
        cereal::YAMLOutputArchive ar(ofs);
        ar(cereal::make_nvp("event_local_planes", drawData));
    }
#endif

    auto pack = std::make_shared<NormFlowPack>();
    pack->nfs = nfsInliers;
    pack->polarityMap = pMat;
    pack->rawTimeSurfaceMap = rtsMat;
    pack->timestamp = _sea->GetTimeLatest();
    // for visualization
    pack->nfsImg = nfsImg;
    pack->nfSeedsImg = nfSeedsImg;
    pack->tsImg = tsImg.clone();
    return pack;
}

std::vector<std::tuple<double, double, double>> EventNormFlow::Centralization(
    const std::vector<std::tuple<int, int, double>> &inRangeData) {
    double mean1 = 0.0, mean2 = 0.0, mean3 = 0.0;

    for (const auto &t : inRangeData) {
        mean1 += std::get<0>(t);
        mean2 += std::get<1>(t);
        mean3 += std::get<2>(t);
    }

    size_t n = inRangeData.size();
    mean1 /= n;
    mean2 /= n;
    mean3 /= n;

    std::vector<std::tuple<double, double, double>> centeredInRangeData;
    centeredInRangeData.resize(inRangeData.size());

    for (size_t i = 0; i < inRangeData.size(); ++i) {
        centeredInRangeData[i] = std::make_tuple(std::get<0>(inRangeData[i]) - mean1,
                                                 std::get<1>(inRangeData[i]) - mean2,
                                                 std::get<2>(inRangeData[i]) - mean3);
    }

    return centeredInRangeData;
}

/**
 * EventLocalPlaneSacProblem
 */

EventLocalPlaneSacProblem::EventLocalPlaneSacProblem(
    const std::vector<std::tuple<double, double, double>> &data, bool randomSeed)
    : opengv::sac::SampleConsensusProblem<model_t>(randomSeed),
      _data(data) {
    setUniformIndices(static_cast<int>(_data.size()));
}

int EventLocalPlaneSacProblem::getSampleSize() const { return 3; }

double EventLocalPlaneSacProblem::PointToPlaneDistance(
    double x, double y, double t, double A, double B, double C) {
    // double numerator = std::abs(A * x + B * y + t + C);
    // double denominator = std::sqrt(A * A + B * B + 1);
    // return numerator / denominator;
    double tPred = -(A * x + B * y + C);
    return std::abs(t - tPred);
}

bool EventLocalPlaneSacProblem::computeModelCoefficients(const std::vector<int> &indices,
                                                         model_t &outModel) const {
    Eigen::MatrixXd M(indices.size(), 3);
    Eigen::VectorXd b(indices.size());

    for (int i = 0; i < static_cast<int>(indices.size()); ++i) {
        const auto &[x, y, t] = _data.at(indices.at(i));
        M(i, 0) = x;
        M(i, 1) = y;
        M(i, 2) = 1;
        b(i) = -t;
    }
    outModel = (M.transpose() * M).ldlt().solve(M.transpose() * b);
    return true;
}

void EventLocalPlaneSacProblem::getSelectedDistancesToModel(const model_t &model,
                                                            const std::vector<int> &indices,
                                                            std::vector<double> &scores) const {
    scores.resize(indices.size());
    for (int i = 0; i < static_cast<int>(indices.size()); ++i) {
        const auto &[x, y, t] = _data.at(indices.at(i));
        scores.at(i) = PointToPlaneDistance(x, y, t, model(0), model(1), model(2));
    }
}

void EventLocalPlaneSacProblem::optimizeModelCoefficients(const std::vector<int> &inliers,
                                                          const model_t &model,
                                                          model_t &optimized_model) {
    computeModelCoefficients(inliers, optimized_model);
}

}  // namespace ns_ekalibr
