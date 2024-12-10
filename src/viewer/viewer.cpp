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

namespace ns_ekalibr {
using ColorPoint = pcl::PointXYZRGBA;
using ColorPointCloud = pcl::PointCloud<ColorPoint>;

Viewer::Viewer(int keptEntityCount)
    : Parent(GenViewerConfigor()),
      keptEntityCount(keptEntityCount) {
    // run
    this->RunInMultiThread();
}

std::shared_ptr<Viewer> Viewer::Create(int keptEntityCount) {
    return std::make_shared<Viewer>(keptEntityCount);
}

ns_viewer::ViewerConfigor Viewer::GenViewerConfigor() {
    ns_viewer::ViewerConfigor viewConfig("eKalibr");
    viewConfig.grid.showGrid = false;
    viewConfig.WithScreenShotSaveDir(Configor::DataStream::OutputPath);
    return viewConfig;
}

Viewer &Viewer::ClearViewer() {
    this->RemoveEntity({_entities.begin(), _entities.end()});
    _entities.clear();
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
}  // namespace ns_ekalibr
