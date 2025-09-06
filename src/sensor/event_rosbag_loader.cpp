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

#include "sensor/event_rosbag_loader.h"
#include "util/status.hpp"
#include "ekalibr/PropheseeEventArray.h"
#include "ekalibr/DVSEventArray.h"
#include "filesystem"
#include "spdlog/spdlog.h"
#include "rosbag/view.h"
#include "util/tqdm.h"

namespace ns_ekalibr {

std::map<std::string, std::vector<EventArray::Ptr>> LoadEventsFromROSBag(
    rosbag::Bag *bag,
    const std::map<std::string, std::string> &topics,
    const ros::Time &begTime,
    const ros::Time &endTime) {
    if (topics.empty() || bag == nullptr) {
        return {};
    }
    auto view = rosbag::View();

    std::vector<std::string> topicsToQuery;
    // add topics to vector
    for (const auto &[topic, _] : topics) {
        topicsToQuery.push_back(topic);
    }
    view.addQuery(*bag, rosbag::TopicQuery(topicsToQuery), begTime, endTime);

    // create data loader
    std::map<std::string, EventDataLoader::Ptr> eventDataLoaders;
    std::map<std::string, std::vector<EventArray::Ptr>> eventMes;

    auto MessageNumInTopic = [](const rosbag::Bag *bag, const std::string &topic,
                                const ros::Time &begTime, const ros::Time &endTime) {
        auto view = rosbag::View();
        view.addQuery(*bag, rosbag::TopicQuery({topic}), begTime, endTime);
        return view.size();
    };

    // get type enum from the string
    for (const auto &[topic, type] : topics) {
        eventDataLoaders.insert({topic, EventDataLoader::GetLoader(type)});
        // reserve tp speed up the data loading
        auto size = MessageNumInTopic(bag, topic, begTime, endTime);
        if (size > 0) {
            eventMes[topic].reserve(size);
        }
    }

    // read raw data
    auto bar = std::make_shared<tqdm>();
    int idx = 0;
    for (auto iter = view.begin(); iter != view.end(); ++iter, ++idx) {
        bar->progress(idx, static_cast<int>(view.size()));
        const auto &item = *iter;
        const std::string &topic = item.getTopic();
        if (eventDataLoaders.cend() != eventDataLoaders.find(topic)) {
            // an event array
            auto mes = eventDataLoaders.at(topic)->UnpackData(item);
            if (mes != nullptr) {
                eventMes.at(topic).push_back(mes);
            }
        }
    }
    bar->finish();

    return eventMes;
}

std::map<std::string, std::vector<EventArray::Ptr>> LoadEventsFromROSBag(
    const std::string &bagPath,
    const std::map<std::string, std::string> &topics,
    double beginTime,
    double duration) {
    // open the ros bag
    auto bag = std::make_unique<rosbag::Bag>();
    if (!std::filesystem::exists(bagPath)) {
        spdlog::error("the ros bag path '{}' is invalid!", bagPath);
    } else {
        bag->open(bagPath, rosbag::BagMode::Read);
    }

    // using a temp view to check the time range of the source ros bag
    auto viewTemp = rosbag::View();

    std::vector<std::string> topicsToQuery;
    // add topics to vector
    for (const auto &[topic, _] : topics) {
        topicsToQuery.push_back(topic);
    }

    viewTemp.addQuery(*bag, rosbag::TopicQuery(topicsToQuery));
    auto begTime = viewTemp.getBeginTime();
    auto endTime = viewTemp.getEndTime();
    spdlog::info("source data duration: from '{:.5f}' to '{:.5f}'.", begTime.toSec(),
                 endTime.toSec());

    // adjust the data time range
    if (beginTime > 0.0) {
        begTime += ros::Duration(beginTime);
        if (begTime > endTime) {
            spdlog::warn(
                "begin time '{:.5f}' is out of the bag's data range, set begin time to '{:.5f}'.",
                begTime.toSec(), viewTemp.getBeginTime().toSec());
            begTime = viewTemp.getBeginTime();
        }
    }
    if (duration > 0.0) {
        endTime = begTime + ros::Duration(duration);
        if (endTime > viewTemp.getEndTime()) {
            spdlog::warn(
                "end time '{:.5f}' is out of the bag's data range, set end time to '{:.5f}'.",
                endTime.toSec(), viewTemp.getEndTime().toSec());
            endTime = viewTemp.getEndTime();
        }
    }
    spdlog::info("expect data duration: from '{:.5f}' to '{:.5f}'.", begTime.toSec(),
                 endTime.toSec());

    auto mes = LoadEventsFromROSBag(bag.get(), topics, begTime, endTime);
    bag->close();

    return mes;
}

EventDataLoader::EventDataLoader(EventModelType model)
    : _model(model) {}

EventDataLoader::Ptr EventDataLoader::GetLoader(const std::string &modelStr) {
    // try extract radar model
    EventModelType model = EventModel::FromString(modelStr);

    EventDataLoader::Ptr dataLoader;
    switch (model) {
        case EventModelType::PROPHESEE_EVENT:
            dataLoader = PropheseeEventDataLoader::Create(model);
            break;
        case EventModelType::DVS_EVENT:
            dataLoader = DVSEventDataLoader::Create(model);
            break;
        default:
            throw Status(Status::ERROR, EventModel::UnsupportedEventModelMsg(modelStr));
    }
    return dataLoader;
}

EventModelType EventDataLoader::GetEventModel() const { return _model; }

PropheseeEventDataLoader::PropheseeEventDataLoader(EventModelType model)
    : EventDataLoader(model) {}

PropheseeEventDataLoader::Ptr PropheseeEventDataLoader::Create(EventModelType model) {
    return std::make_shared<PropheseeEventDataLoader>(model);
}

EventArray::Ptr PropheseeEventDataLoader::UnpackData(const rosbag::MessageInstance &msgInstance) {
    ekalibr::PropheseeEventArrayPtr msg = msgInstance.instantiate<ekalibr::PropheseeEventArray>();

    CheckMessage<ekalibr::PropheseeEventArray>(msg);

    std::vector<Event::Ptr> events(msg->events.size());

    for (int i = 0; i < static_cast<int>(msg->events.size()); i++) {
        const auto &event = msg->events.at(i);
        events.at(i) =
            Event::Create(event.ts.toSec(), Event::PosType(event.x, event.y), event.polarity);
    }

    if (msg->header.stamp.isZero()) {
        return EventArray::Create(events.back()->GetTimestamp(), events);
    } else {
        return EventArray::Create(msg->header.stamp.toSec(), events);
    }
}

DVSEventDataLoader::DVSEventDataLoader(EventModelType model)
    : EventDataLoader(model) {}

DVSEventDataLoader::Ptr DVSEventDataLoader::Create(EventModelType model) {
    return std::make_shared<DVSEventDataLoader>(model);
}

EventArray::Ptr DVSEventDataLoader::UnpackData(const rosbag::MessageInstance &msgInstance) {
    ekalibr::DVSEventArrayPtr msg = msgInstance.instantiate<ekalibr::DVSEventArray>();

    CheckMessage<ekalibr::DVSEventArray>(msg);

    std::vector<Event::Ptr> events(msg->events.size());

    for (int i = 0; i < static_cast<int>(msg->events.size()); i++) {
        const auto &event = msg->events.at(i);
        events.at(i) =
            Event::Create(event.ts.toSec(), Event::PosType(event.x, event.y), event.polarity);
    }
    if (msg->header.stamp.isZero()) {
        return EventArray::Create(events.back()->GetTimestamp(), events);
    } else {
        return EventArray::Create(msg->header.stamp.toSec(), events);
    }
}
}  // namespace ns_ekalibr