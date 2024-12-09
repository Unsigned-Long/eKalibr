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

#ifndef VIEWER_H
#define VIEWER_H

#include "tiny-viewer/core/viewer.h"
#include "tiny-viewer/entity/entity.h"

namespace ns_ekalibr {
struct EventArray;
using EventArrayPtr = std::shared_ptr<EventArray>;
struct Event;
using EventPtr = std::shared_ptr<Event>;

class Viewer : public ns_viewer::Viewer {
public:
    using Ptr = std::shared_ptr<Viewer>;
    using Parent = ns_viewer::Viewer;

private:
    std::vector<std::size_t> _entities;

public:
    explicit Viewer();

    static Ptr Create();

    Viewer &ClearViewer();

    Viewer &PopBackEntity();

    Viewer &AddEntityLocal(const std::vector<ns_viewer::Entity::Ptr> &entities);

    // add discrete pos
    Viewer &AddSpatioTemporalTrace(const std::vector<Eigen::Vector3d> &trace,
                                   float sTime,
                                   float size = 0.5f,
                                   const ns_viewer::Colour &color = ns_viewer::Colour::Blue(),
                                   const std::pair<float, float> &ptScales = {0.01f, 2.0f});

    Viewer &AddEventData(const std::vector<EventArrayPtr>::const_iterator &sIter,
                         const std::vector<EventArrayPtr>::const_iterator &eIter,
                         float sTime,
                         const std::pair<float, float> &ptScales = {0.01f, 2.0f},
                         float ptSize = 1.0f);

    Viewer &AddEventData(const EventArrayPtr &ary,
                         float sTime,
                         const std::pair<float, float> &ptScales = {0.01f, 2.0f},
                         const std::optional<ns_viewer::Colour> &color = {},
                         float ptSize = 1.0f);

    Viewer &AddEventData(const std::list<EventPtr> &ary,
                         float sTime,
                         const std::pair<float, float> &ptScales = {0.01f, 2.0f},
                         const std::optional<ns_viewer::Colour> &color = {},
                         float ptSize = 1.0f);

protected:
    ns_viewer::ViewerConfigor GenViewerConfigor();
};
}  // namespace ns_ekalibr

#endif  // VIEWER_H
