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

#include "viewer/viewer.h"
#include "config/configor.h"
#include "sensor/event.h"
#include "tiny-viewer/entity/line.h"
#include "tiny-viewer/entity/point_cloud.hpp"
#include "tiny-viewer/object/landmark.h"
#include "pcl/point_types.h"
#include "tiny-viewer/entity/coordinate.h"
#include "tiny-viewer/core/pose.hpp"
#include "tiny-viewer/entity/arrow.h"
#include "core/calib_param_mgr.h"
#include <spdlog/spdlog.h>

namespace ns_ekalibr {
using ColorPoint = pcl::PointXYZRGBA;
using ColorPointCloud = pcl::PointCloud<ColorPoint>;

Viewer::Viewer(int keptEntityCount)
    : Parent(GenViewerConfigor()),
      keptEntityCount(keptEntityCount),
      _parMagr(nullptr),
      _splines(nullptr) {
    // run
    this->RunInMultiThread();
    std::cout << "\033[92m\033[3m[Viewer] "
                 "use 'a' and 'd' keys to spatially zoom out and in viewer in run time, and 's' "
                 "and 'w' to temporally to zoom out and in viewer in run time!\033[0m"
              << std::endl;
}

std::shared_ptr<Viewer> Viewer::Create(int keptEntityCount) {
    return std::make_shared<Viewer>(keptEntityCount);
}

ns_viewer::ViewerConfigor Viewer::GenViewerConfigor() {
    ns_viewer::ViewerConfigor viewConfig("eKalibr");
    viewConfig.grid.showGrid = false;
    viewConfig.WithScreenShotSaveDir(Configor::DataStream::OutputPath);

    viewConfig.callBacks.insert({'a', [this] { ZoomOutSpatialScaleCallBack(); }});
    viewConfig.callBacks.insert({'d', [this] { ZoomInSpatialScaleCallBack(); }});
    viewConfig.callBacks.insert({'s', [] { ZoomOutTemporalScaleCallBack(); }});
    viewConfig.callBacks.insert({'w', [] { ZoomInTemporalScaleCallBack(); }});

    return viewConfig;
}

pangolin::OpenGlRenderState Viewer::GetInitRenderState() const {
    const auto &c = _configor.camera;
    auto camView = pangolin::OpenGlRenderState(
        pangolin::ProjectionMatrix(c.width, c.height, c.fx, c.fy, c.cx, c.cy, c.near, c.far),
        pangolin::ModelViewLookAt(ExpandStdVec3(_configor.camera.initPos),
                                  ExpandStdVec3(_configor.camera.initViewPoint), pangolin::AxisZ));
    return camView;
}

Viewer &Viewer::ClearViewer() {
    this->RemoveEntity({_entities.begin(), _entities.end()});
    _entities.clear();
    return *this;
}

Viewer &Viewer::ResetViewerCamera() {
    this->SetCamView(this->GetInitRenderState());
    return *this;
}

Viewer &Viewer::PopBackEntity() {
    if (!_entities.empty()) {
        this->RemoveEntity(_entities.back());
        _entities.pop_back();
    }
    return *this;
}

Viewer &Viewer::AddEntityLocal(const std::vector<ns_viewer::Entity::Ptr> &entities) {
    auto ids = this->AddEntity(entities);
    _entities.insert(_entities.end(), ids.cbegin(), ids.cend());
    if (keptEntityCount > 0 && _entities.size() > 1.2 * keptEntityCount) {
        auto iter = _entities.cbegin();
        std::advance(iter, _entities.size() - keptEntityCount);

        this->RemoveEntity({_entities.cbegin(), iter});
        _entities.erase(_entities.cbegin(), iter);
    }
    return *this;
}

Viewer &Viewer::AddSpatioTemporalTrace(const std::vector<Eigen::Vector3d> &trace,
                                       float size,
                                       const ns_viewer::Colour &color,
                                       const std::pair<float, float> &ptScales) {
    std::vector<ns_viewer::Entity::Ptr> entities;
    for (int i = 0; i != static_cast<int>(trace.size()) - 1; ++i) {
        const Eigen::Vector3f &f1 = trace.at(i).cast<float>();
        const Eigen::Vector3f &f2 = trace.at(i + 1).cast<float>();

        const Eigen::Vector2f &p1 = f1.tail<2>() * ptScales.first;
        const auto z1 = -f1(0) * ptScales.second;

        const Eigen::Vector2f &p2 = f2.tail<2>() * ptScales.first;
        const auto z2 = -f2(0) * ptScales.second;

        auto line = ns_viewer::Line::Create({p1(0), p1(1), z1}, {p2(0), p2(1), z2}, color, size);
        entities.push_back(line);
    }
    AddEntityLocal(entities);
    return *this;
}

Viewer &Viewer::AddEventData(const std::vector<EventArray::Ptr>::const_iterator &sIter,
                             const std::vector<EventArray::Ptr>::const_iterator &eIter,
                             const std::pair<float, float> &ptScales,
                             float ptSize) {
    pcl::PointCloud<ColorPoint>::Ptr cloud(new ColorPointCloud);
    for (auto iter = sIter; iter != eIter; ++iter) {
        for (const auto &event : (*iter)->GetEvents()) {
            Eigen::Vector2f p = event->GetPos().cast<float>() * ptScales.first;
            float t = (float)event->GetTimestamp() * ptScales.second;
            ColorPoint cp;
            cp.x = p(0), cp.y = p(1), cp.z = -t;
            if (event->GetPolarity()) {
                cp.b = 255;
                cp.r = cp.g = 0;
            } else {
                cp.r = 255;
                cp.b = cp.g = 0;
            }
            cp.a = 50;
            cloud->push_back(cp);
        }
    }
    AddEntityLocal({ns_viewer::Cloud<ColorPoint>::Create(cloud, ptSize)});
    return *this;
}

Viewer &Viewer::AddEventData(const EventArray::Ptr &ary,
                             const std::pair<float, float> &ptScales,
                             const std::optional<ns_viewer::Colour> &color,
                             float ptSize) {
    if (ary == nullptr) {
        return *this;
    }
    pcl::PointCloud<ColorPoint>::Ptr cloud(new ColorPointCloud);
    const auto &events = ary->GetEvents();
    for (const auto &event : events) {
        Eigen::Vector2f p = event->GetPos().cast<float>() * ptScales.first;
        float t = (float)event->GetTimestamp() * ptScales.second;
        ColorPoint cp;
        cp.x = p(0), cp.y = p(1), cp.z = -t;
        if (color == std::nullopt) {
            if (event->GetPolarity()) {
                cp.b = 255;
                cp.r = cp.g = 0;
            } else {
                cp.r = 255;
                cp.b = cp.g = 0;
            }
        } else {
            cp.b = color->b * 255;
            cp.r = color->r * 255;
            cp.g = color->g * 255;
        }

        cp.a = 255;
        cloud->push_back(cp);
    }
    AddEntityLocal({ns_viewer::Cloud<ColorPoint>::Create(cloud, ptSize)});
    return *this;
}

Viewer &Viewer::AddEventData(const std::list<EventPtr> &ary,
                             const std::pair<float, float> &ptScales,
                             const std::optional<ns_viewer::Colour> &color,
                             float ptSize) {
    if (ary.empty()) {
        return *this;
    }
    auto eAry = EventArray::Create(ary.back()->GetTimestamp(), {ary.cbegin(), ary.cend()});
    return AddEventData(eAry, ptScales, color, ptSize);
}

Viewer &Viewer::AddGridPattern(const std::vector<Eigen::Vector2f> &centers,
                               double timestamp,
                               const std::pair<float, float> &ptScales,
                               const ns_viewer::Colour &color,
                               float ptSize) {
    const float z = -timestamp * ptScales.second;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud->resize(centers.size());
    for (const auto &center : centers) {
        pcl::PointXYZRGB p;
        p.x = center(0) * ptScales.first;
        p.y = center(1) * ptScales.first;
        p.z = z;
        p.r = color.r * 255;
        p.g = color.g * 255;
        p.b = color.b * 255;

        cloud->push_back(p);
    }
    std::vector<ns_viewer::Entity::Ptr> entities;
    entities.reserve(centers.size() * 2 - 1);
    entities.push_back(std::make_shared<ns_viewer::Cloud<ns_viewer::Landmark>>(cloud, ptSize));

    for (int i = 0; i < static_cast<int>(centers.size() - 1); i++) {
        int j = i + 1;
        Eigen::Vector3f ci = centers.at(i).homogeneous();
        ci *= ptScales.first;
        ci(2) = z;
        Eigen::Vector3f cj = centers.at(j).homogeneous();
        cj *= ptScales.first;
        cj(2) = z;
        entities.push_back(ns_viewer::Line::Create(ci, cj, 40.0f * ptSize, color));
    }

    return AddEntityLocal(entities);
}

Viewer &Viewer::AddGridPattern(const std::vector<cv::Point3f> &centers,
                               const float &pScale,
                               const ns_viewer::Colour &color,
                               float ptSize) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud->resize(centers.size());
    for (const auto &center : centers) {
        pcl::PointXYZRGB p;
        p.x = center.x * pScale;
        p.y = center.y * pScale;
        p.z = center.z;
        p.r = color.r * 255;
        p.g = color.g * 255;
        p.b = color.b * 255;

        cloud->push_back(p);
    }
    std::vector<ns_viewer::Entity::Ptr> entities;
    entities.reserve(centers.size() * 2 - 1);
    entities.push_back(std::make_shared<ns_viewer::Cloud<ns_viewer::Landmark>>(cloud, ptSize));

    for (int i = 0; i < static_cast<int>(centers.size() - 1); i++) {
        int j = i + 1;
        Eigen::Vector3f ci(centers.at(i).x, centers.at(i).y, centers.at(i).z);
        ci *= pScale;
        Eigen::Vector3f cj(centers.at(j).x, centers.at(j).y, centers.at(j).z);
        cj *= pScale;
        entities.push_back(ns_viewer::Line::Create(ci, cj, 40.0f * ptSize, color));
    }

    return AddEntityLocal(entities);
}

Viewer &Viewer::UpdateSplineViewer(const float &pScale, double dt) {
    ClearViewer();

    // spline poses
    std::vector<ns_viewer::Entity::Ptr> entities;
    const auto &so3Spline = _splines->GetSo3Spline(Configor::Preference::SO3_SPLINE);
    const auto &scaleSpline = _splines->GetRdSpline(Configor::Preference::SCALE_SPLINE);
    double minTime = std::max(so3Spline.MinTime(), scaleSpline.MinTime());
    double maxTime = std::min(so3Spline.MaxTime(), scaleSpline.MaxTime());
    for (double t = minTime; t < maxTime;) {
        if (!_splines->TimeInRange(t, so3Spline) || !_splines->TimeInRange(t, scaleSpline)) {
            t += dt;
            continue;
        }

        Sophus::SO3d so3 = so3Spline.Evaluate(t);
        Eigen::Vector3d linScale = scaleSpline.Evaluate(t) * pScale;
        // coordinate
        entities.push_back(ns_viewer::Coordinate::Create(
            ns_viewer::Posed(so3.matrix(), linScale).cast<float>(), 0.1f));
        t += dt;
    }
    const auto &knots = scaleSpline.GetKnots();
    for (const auto &k : knots) {
        entities.push_back(ns_viewer::Landmark::Create(k.cast<float>() * pScale, 0.1f,
                                                       ns_viewer::Colour::Black()));
    }
    for (int i = 0; i < static_cast<int>(knots.size()) - 1; ++i) {
        const int j = i + 1;
        const Eigen::Vector3d ki = knots.at(i) * pScale;
        const Eigen::Vector3d kj = knots.at(j) * pScale;
        entities.push_back(ns_viewer::Line::Create(ki.cast<float>(), kj.cast<float>(), 0.1f,
                                                   ns_viewer::Colour::Black()));
    }
    // gravity
    entities.push_back(Gravity());

    this->AddEntityLocal(entities);

    return *this;
}

void Viewer::SetSpline(const SplineBundleType::Ptr &splines) { _splines = splines; }

void Viewer::SetParMgr(const CalibParamManagerPtr &parMgr) { _parMagr = parMgr; }

void Viewer::SetKeptEntityCount(int val) { keptEntityCount = val; }

ns_viewer::Entity::Ptr Viewer::Gravity() const {
    return ns_viewer::Arrow::Create(_parMagr->GRAVITY.normalized().cast<float>(),
                                    Eigen::Vector3f::Zero(), ns_viewer::Colour::Blue());
}

void Viewer::ZoomInSpatialScaleCallBack() {
    Configor::Preference::EventViewerSpatialTemporalScale.first += 0.005;
    if (_splines != nullptr) {
        UpdateSplineViewer();
    }
}

void Viewer::ZoomOutSpatialScaleCallBack() {
    if (Configor::Preference::EventViewerSpatialTemporalScale.first >= 0.01) {
        Configor::Preference::EventViewerSpatialTemporalScale.first -= 0.005;
    }
    if (_splines != nullptr) {
        UpdateSplineViewer();
    }
}

void Viewer::ZoomInTemporalScaleCallBack() {
    Configor::Preference::EventViewerSpatialTemporalScale.second += 5.0;
}

void Viewer::ZoomOutTemporalScaleCallBack() {
    if (Configor::Preference::EventViewerSpatialTemporalScale.second >= 10.0) {
        Configor::Preference::EventViewerSpatialTemporalScale.second -= 5.0;
    }
}
}  // namespace ns_ekalibr
