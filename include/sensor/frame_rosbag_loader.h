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

#ifndef EKALIBR_FRAME_ROSBAG_LOADER_H
#define EKALIBR_FRAME_ROSBAG_LOADER_H

#include "sensor/frame.h"
#include "sensor/sensor_model.h"
#include "util/enum_cast.hpp"
#include "map"

namespace ros {
class Time;
}

namespace rosbag {
class Bag;
class MessageInstance;
}  // namespace rosbag

namespace ns_ekalibr {
std::map<std::string, std::vector<Frame::Ptr>> LoadFramesFromROSBag(
    rosbag::Bag* bag,
    // topic, type
    const std::map<std::string, std::string>& topics,
    const ros::Time& begTime,
    const ros::Time& endTime);

std::map<std::string, std::vector<Frame::Ptr>> LoadFramesFromROSBag(
    const std::string& bagPath,
    // topic, type
    const std::map<std::string, std::string>& topics,
    // negative values mean loading all data
    double beginTime = -1.0,
    double duration = -1.0);

class FrameDataLoader {
public:
    using Ptr = std::shared_ptr<FrameDataLoader>;

protected:
    FrameModelType _model;

public:
    explicit FrameDataLoader(FrameModelType& model);

    virtual Frame::Ptr UnpackData(const rosbag::MessageInstance& msg) = 0;

    static FrameDataLoader::Ptr GetLoader(const std::string& modelStr);

    [[nodiscard]] FrameModelType GetFrameModel() const;

    virtual ~FrameDataLoader() = default;

protected:
    template <class MsgType>
    void CheckMessage(typename MsgType::ConstPtr msg) {
        if (msg == nullptr) {
            throw std::runtime_error(
                "Wrong sensor model: '" + std::string(EnumCast::enumToString(GetFrameModel())) +
                "' for event cameras! It's incompatible with the type of ros message to load in!");
        }
    }
};

class SensorImageLoader : public FrameDataLoader {
public:
    using Ptr = std::shared_ptr<SensorImageLoader>;

public:
    explicit SensorImageLoader(FrameModelType model);

    static SensorImageLoader::Ptr Create(FrameModelType model);

    Frame::Ptr UnpackData(const rosbag::MessageInstance& msgInstance) override;
};

class SensorImageCompLoader : public FrameDataLoader {
public:
    using Ptr = std::shared_ptr<SensorImageCompLoader>;

public:
    explicit SensorImageCompLoader(FrameModelType model);

    static SensorImageCompLoader::Ptr Create(FrameModelType model);

    Frame::Ptr UnpackData(const rosbag::MessageInstance& msgInstance) override;
};
}  // namespace ns_ekalibr

#endif  // EKALIBR_FRAME_ROSBAG_LOADER_H
