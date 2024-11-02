// MIT License
//
// Copyright (c) 2022 Alvin Sun
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "vrpn_mocap/tracker.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <regex>
#include <string>

namespace vrpn_mocap
{

using geometry_msgs::msg::PoseStamped;
using namespace std::chrono_literals;

std::string Tracker::ValidNodeName(const std::string & tracker_name)
{
  // replace non alphanum characters with _
  const std::string alnum_name = std::regex_replace(tracker_name, std::regex("[^a-zA-Z0-9_]"), "_");
  // strip consecutive underscores
  const std::string node_name = std::regex_replace(alnum_name, std::regex("_+"), "_");

  return node_name;
}

Tracker::Tracker(const std::string & tracker_name)
: Node(ValidNodeName(tracker_name)),
  name_(tracker_name),
  multi_sensor_(declare_parameter("multi_sensor", false)),
  frame_id_(declare_parameter("frame_id", "world")),
  sensor_data_qos_(declare_parameter("sensor_data_qos", true)),
  use_vrpn_timestamps_(declare_parameter("use_vrpn_timestamps", false)),
  vrpn_tracker_(name_.c_str())
{
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  Init();

  // start main loop when instantiated as a standalone node
  const double update_freq = this->declare_parameter("update_freq", 100.);
  timer_ = this->create_wall_timer(1s / update_freq, std::bind(&Tracker::MainLoop, this));
}

Tracker::Tracker(
  const rclcpp::Node & base_node, const std::string & tracker_name,
  const std::shared_ptr<vrpn_Connection> & connection)
: Node(base_node, ValidNodeName(tracker_name)),
  name_(tracker_name),
  multi_sensor_(base_node.get_parameter("multi_sensor").as_bool()),
  frame_id_(base_node.get_parameter("frame_id").as_string()),
  sensor_data_qos_(base_node.get_parameter("sensor_data_qos").as_bool()),
  use_vrpn_timestamps_(base_node.get_parameter("use_vrpn_timestamps").as_bool()),
  vrpn_tracker_(name_.c_str(), connection.get())
{
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  Init();
}

Tracker::~Tracker()
{
  vrpn_tracker_.unregister_change_handler(this, &Tracker::HandlePose);

  RCLCPP_INFO_STREAM(this->get_logger(), "Destroyed new tracker " << name_);
}

void Tracker::Init()
{
  vrpn_tracker_.register_change_handler(this, &Tracker::HandlePose);
  vrpn_tracker_.shutup = true;

  RCLCPP_INFO_STREAM(this->get_logger(), "Created new tracker " << name_);
}

void Tracker::MainLoop() {vrpn_tracker_.mainloop();}

builtin_interfaces::msg::Time Tracker::GetTimestamp(struct timeval vrpn_timestamp)
{
  if (this->use_vrpn_timestamps_) {
    builtin_interfaces::msg::Time stamp;
    stamp.sec = vrpn_timestamp.tv_sec;
    stamp.nanosec = vrpn_timestamp.tv_usec * 1000;

    return stamp;
  }
  return this->get_clock()->now();
}

void VRPN_CALLBACK Tracker::HandlePose(void * data, const vrpn_TRACKERCB tracker_pose)
{
  Tracker * tracker = static_cast<Tracker *>(data);

  auto current_timestamp = tracker->GetTimestamp(tracker_pose.msg_time);

  auto pub = tracker->GetOrCreatePublisher<PoseStamped>(
    static_cast<size_t>(tracker_pose.sensor), "pose", &tracker->pose_pubs_);

  PoseStamped msg;
  msg.header.frame_id = tracker->frame_id_;
  msg.header.stamp = current_timestamp;

  msg.pose.position.x = tracker_pose.pos[0];
  msg.pose.position.y = tracker_pose.pos[1];
  msg.pose.position.z = tracker_pose.pos[2];

  msg.pose.orientation.x = tracker_pose.quat[0];
  msg.pose.orientation.y = tracker_pose.quat[1];
  msg.pose.orientation.z = tracker_pose.quat[2];
  msg.pose.orientation.w = tracker_pose.quat[3];

  pub->publish(msg);

  // publish tf
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = current_timestamp;
  transform.header.frame_id = tracker->frame_id_;
  transform.child_frame_id = tracker->name_;

  transform.transform.translation.x = tracker_pose.pos[0];
  transform.transform.translation.y = tracker_pose.pos[1];
  transform.transform.translation.z = tracker_pose.pos[2];

  transform.transform.rotation.x = tracker_pose.quat[0];
  transform.transform.rotation.y = tracker_pose.quat[1];
  transform.transform.rotation.z = tracker_pose.quat[2];
  transform.transform.rotation.w = tracker_pose.quat[3];

  tracker->tf_broadcaster_->sendTransform(transform);
}

}  // namespace vrpn_mocap