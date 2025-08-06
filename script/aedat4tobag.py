#  eKalibr, Copyright 2024, the School of Geodesy and Geomatics (SGG), Wuhan University, China
#  https://github.com/Unsigned-Long/eKalibr.git
#  Author: Shuolong Chen (shlchen@whu.edu.cn)
#  GitHub: https://github.com/Unsigned-Long
#   ORCID: 0000-0002-5283-9057
#  Purpose: See .h/.hpp file.
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#  * Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#  * The names of its contributors can not be
#    used to endorse or promote products derived from this software without
#    specific prior written permission.
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
#  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
#  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
#  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
#  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
#  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
#  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
#  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.

import rospy
import rosbag
from sensor_msgs.msg import Image, Imu
from dvs_msgs.msg import Event, EventArray
from dv import AedatFile
import os
from cv_bridge import CvBridge
import argparse


def add_imu_to_bag(topic, bag, aedat_file):
    print(f"Adding IMU data to topic {topic} from {aedat_file}...")
    with AedatFile(aedat_file) as aedat:
        for imu in aedat["imu"]:
            imu_msg = Imu()
            imu_msg.header.stamp = rospy.Time.from_sec(imu.timestamp * 1e-6)
            imu_msg.linear_acceleration.x = imu.accelerometer[0]
            imu_msg.linear_acceleration.y = imu.accelerometer[1]
            imu_msg.linear_acceleration.z = imu.accelerometer[2]
            imu_msg.angular_velocity.x = imu.gyroscope[0]
            imu_msg.angular_velocity.y = imu.gyroscope[1]
            imu_msg.angular_velocity.z = imu.gyroscope[2]
            bag.write(topic, imu_msg, imu_msg.header.stamp)


def add_events_to_bag(topic, bag, aedat_file, ev_ary_dt_sec):
    print(f"Adding events to topic {topic} from {aedat_file}...")
    last_sec = None
    with AedatFile(aedat_file) as aedat:
        height, width = aedat["events"].size
        events = []
        for event in aedat["events"]:
            if last_sec is None:
                last_sec = event.timestamp * 1e-6
            event_msg = Event()
            event_msg.ts = rospy.Time.from_sec(event.timestamp * 1e-6)
            event_msg.x = event.x
            event_msg.y = event.y
            event_msg.polarity = event.polarity
            events.append(event_msg)
            if (event.timestamp * 1e-6 - last_sec) >= ev_ary_dt_sec:
                event_array_msg = EventArray()
                event_array_msg.header.stamp = rospy.Time.from_sec(
                    event.timestamp * 1e-6
                )
                # Fill the EventArray message
                event_array_msg.events = events
                event_array_msg.height = height
                event_array_msg.width = width
                bag.write(topic, event_array_msg, event_array_msg.header.stamp)
                last_sec = event.timestamp * 1e-6
                events = []
        if events:
            event_array_msg = EventArray()
            event_array_msg.header.stamp = rospy.Time.from_sec(event.timestamp * 1e-6)
            event_array_msg.events = events
            event_array_msg.height = height
            event_array_msg.width = width
            bag.write(topic, event_array_msg, event_array_msg.header.stamp)


def add_frames_to_bag(topic, bag, aedat_file):
    print(f"Adding frames to topic {topic} from {aedat_file}...")
    with AedatFile(aedat_file) as aedat:
        height, width = aedat["events"].size
        frame_cnt = 0
        for frame in aedat["frames"]:
            img_msg = Image()
            bridge = CvBridge()

            img_msg.height = height
            img_msg.width = width

            img_msg = bridge.cv2_to_imgmsg(frame.image, encoding="passthrough")

            img_msg.header.seq = frame_cnt
            img_msg.header.stamp = rospy.Time.from_sec(frame.timestamp * 1e-6)

            bag.write(topic, img_msg, t=img_msg.header.stamp)
            frame_cnt += 1


if __name__ == "__main__":
    print(
      "\033[93mThis script is deprecated. The process efficiency is low!!! "
      "To convert AEDAT4 to ROS bag, please use the official aedat2bag tool:\n"
      "https://gitlab.com/inivation/dv/dv-ros/-/blob/master/dv_ros_aedat4/src/convert_aedat4.cpp\033[0m"
    )
    raise SystemExit()
    # example:
    # python3 aedat4tobag.py 'davis_left:/media/csl/samsung/eKalibr/dataset/record/data1/dvSave_left-2025_08_06_14_04_47.aedat4' 'davis_right:/media/csl/samsung/eKalibr/dataset/record/data1/dvSave_right-2025_08_06_14_04_48.aedat4' '/media/csl/samsung/eKalibr/dataset/record/data1/stereo_davis.bag'
    parser = argparse.ArgumentParser(description="Convert Aedat files to ROS bag")
    # input string list [prefix_tag:filename, ...]
    parser.add_argument(
        "input",
        type=str,
        nargs="+",
        help="Input Aedat files with prefix tags: [prefix_tag:filename, ...]",
    )
    # output bag file
    parser.add_argument("output", type=str, help="Output ROS bag file")
    args = parser.parse_args()
    input_files = [
        {"prefix_tag": item.split(":")[0], "filename": item.split(":")[1]}
        for item in args.input
    ]
    output_file = args.output
    bag = rosbag.Bag(output_file, "w")
    for input in input_files:
        prefix_tag = input["prefix_tag"]
        aedat_file = input["filename"]
        if not os.path.exists(aedat_file):
            raise FileNotFoundError(f"Aedat file {aedat_file} does not exist.")

        add_events_to_bag(f"/{prefix_tag}/events", bag, aedat_file, ev_ary_dt_sec=1.0)
        add_imu_to_bag(f"/{prefix_tag}/imu", bag, aedat_file)
        add_frames_to_bag(f"/{prefix_tag}/frames", bag, aedat_file)
    bag.close()
    print(f"Converted Aedat files to ROS bag: {output_file}")
