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

#ifndef EVENT_H
#define EVENT_H

#include "sensor/event.h"
#include "Eigen/Dense"
#include "tiny-viewer/core/utils.hpp"
#include "cereal/cereal.hpp"

namespace ns_ekalibr {
class Event {
public:
    using Ptr = std::shared_ptr<Event>;
    using PosType = Eigen::Vector2<std::uint16_t>;

private:
    // the timestamp of this event
    double _timestamp;
    PosType _pos;
    bool _polarity;

public:
    explicit Event(double timestamp, PosType pos = PosType::Zero(), bool polarity = {});

    static Ptr Create(double timestamp, const PosType& pos = PosType::Zero(), bool polarity = {});

    [[nodiscard]] double GetTimestamp() const;

    void SetTimestamp(double timestamp);

    [[nodiscard]] PosType GetPos() const;

    [[nodiscard]] bool GetPolarity() const;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
    template <class Archive>
    void serialize(Archive& ar) {
        ar(cereal::make_nvp("timestamp", _timestamp), cereal::make_nvp("pos", _pos),
           cereal::make_nvp("polarity", _polarity));
    }
};

class EventArray {
public:
    using Ptr = std::shared_ptr<EventArray>;

private:
    double _timestamp;
    std::vector<Event::Ptr> _events;

public:
    explicit EventArray(double timestamp = INVALID_TIME_STAMP,
                        const std::vector<Event::Ptr>& events = {});

    static Ptr Create(double timestamp = INVALID_TIME_STAMP,
                      const std::vector<Event::Ptr>& events = {});

    [[nodiscard]] double GetTimestamp() const;

    [[nodiscard]] const std::vector<Event::Ptr>& GetEvents() const;

    void SetTimestamp(double timestamp);

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
    template <class Archive>
    void serialize(Archive& ar) {
        ar(cereal::make_nvp("timestamp", _timestamp), cereal::make_nvp("events", _events));
    }
};
}  // namespace ns_ekalibr

#endif  // EVENT_H
