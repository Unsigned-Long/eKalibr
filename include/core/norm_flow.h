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

#ifndef NORM_FLOW_H
#define NORM_FLOW_H

#include "opencv4/opencv2/core.hpp"
#include "Eigen/Dense"
#include "opengv/sac/SampleConsensusProblem.hpp"
#include "map"
#include "list"

namespace ns_ekalibr {
struct Event;
using EventPtr = std::shared_ptr<Event>;
class ActiveEventSurface;
using ActiveEventSurfacePtr = std::shared_ptr<ActiveEventSurface>;

struct NormFlow {
public:
    using Ptr = std::shared_ptr<NormFlow>;

    double timestamp;
    Eigen::Vector2i p;
    Eigen::Vector2d nf;
    // components of norm flow, i.e., nf = nfNorm * nfDir
    double nfNorm;
    Eigen::Vector2d nfDir;

    NormFlow(double timestamp, const Eigen::Vector2i &p, const Eigen::Vector2d &nf);

    static Ptr Create(double timestamp, const Eigen::Vector2i &p, const Eigen::Vector2d &nf);
};

class EventNormFlow {
public:
    using Ptr = std::shared_ptr<EventNormFlow>;

    struct NormFlowPack {
        using Ptr = std::shared_ptr<NormFlowPack>;
        using NormFlowContainer =
            std::map<NormFlow::Ptr, std::vector<std::tuple<int, int, double>>>;

    public:
        // norm flow, [x, y, timestamp]
        std::map<NormFlow::Ptr, std::vector<std::tuple<int, int, double>>> nfs;
        cv::Mat rawTimeSurfaceMap;  // ex, ey, et
        cv::Mat polarityMap;        // the polarity map
        double timestamp;

        // for visualization
        cv::Mat nfSeedsImg;
        cv::Mat nfsImg;
        cv::Mat tsImg;

    public:
        std::list<EventPtr> ActiveEvents(double dt = 0.02) const;

        std::list<EventPtr> NormFlowInlierEvents() const;

        cv::Mat Visualization(double dt = 0.02) const;

        cv::Mat InliersOccupyMat() const;

        cv::Mat NormFlowInlierEventMat() const;

        cv::Mat AccumulativeEventMat(double dt = 0.02) const;

        std::list<NormFlow::Ptr> TemporallySortedNormFlows() const;

        std::list<NormFlow::Ptr> NormFlows() const;

        int Rows() const;

        int Cols() const;
    };

private:
    ActiveEventSurfacePtr _sea;

public:
    explicit EventNormFlow(const ActiveEventSurfacePtr &sea);

    NormFlowPack::Ptr ExtractNormFlows(double decaySec = 0.02,
                                       int winSize = 2,
                                       int neighborDist = 2,
                                       double goodRatioThd = 0.9,
                                       double timeDistEventToPlaneThd = 2E-3,
                                       int ransacMaxIter = 3) const;

protected:
    static std::vector<std::tuple<double, double, double>> Centralization(
        const std::vector<std::tuple<int, int, double>> &inRangeData);
};

class EventLocalPlaneSacProblem : public opengv::sac::SampleConsensusProblem<Eigen::Vector3d> {
public:
    typedef Eigen::Vector3d model_t;

public:
    explicit EventLocalPlaneSacProblem(const std::vector<std::tuple<double, double, double>> &data,
                                       bool randomSeed = true);

    ~EventLocalPlaneSacProblem() override = default;

    bool computeModelCoefficients(const std::vector<int> &indices,
                                  model_t &outModel) const override;

    void getSelectedDistancesToModel(const model_t &model,
                                     const std::vector<int> &indices,
                                     std::vector<double> &scores) const override;

    void optimizeModelCoefficients(const std::vector<int> &inliers,
                                   const model_t &model,
                                   model_t &optimized_model) override;

    [[nodiscard]] int getSampleSize() const override;

    static double PointToPlaneDistance(double x, double y, double t, double A, double B, double C);

protected:
    /** The adapter holding all input data */
    const std::vector<std::tuple<double, double, double>> &_data;
};
}  // namespace ns_ekalibr

#endif  // NORM_FLOW_H
