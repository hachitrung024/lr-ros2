// Copyright 2026 Landfill Rover Maintainers
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/quaternion_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/exact_time.h"
#include "message_filters/synchronizer.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.hpp"
#include "tf2_ros/message_filter.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

class PointcloudTransformNode : public rclcpp::Node
{
public:
  PointcloudTransformNode()
  : Node("pointcloud_transform_node"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    input_topic_ = declare_parameter<std::string>(
      "input_topic", "/zed/zed_node/point_cloud/cloud_registered");
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/lr/point_cloud/cloud_in_map");
    target_frame_ = declare_parameter<std::string>("target_frame", "map");
    transform_timeout_sec_ = declare_parameter<double>("transform_timeout_sec", 0.5);
    pose_source_ = declare_parameter<std::string>("pose_source", "tf");
    mavlink_position_topic_ = declare_parameter<std::string>(
      "mavlink_position_topic", "/lr/mavlink/position_enu");
    mavlink_attitude_topic_ = declare_parameter<std::string>(
      "mavlink_attitude_topic", "/lr/mavlink/attitude_enu");
    mavlink_base_frame_ = declare_parameter<std::string>(
      "mavlink_base_frame", "lr_base_link");
    camera_link_frame_ = declare_parameter<std::string>(
      "camera_link_frame", "zed_camera_link");
    base_to_camera_x_m_ = declare_parameter<double>("base_to_camera_x_m", 0.0);
    base_to_camera_y_m_ = declare_parameter<double>("base_to_camera_y_m", 0.0);
    base_to_camera_z_m_ = declare_parameter<double>("base_to_camera_z_m", 0.0);
    base_to_camera_roll_deg_ = declare_parameter<double>("base_to_camera_roll_deg", 0.0);
    base_to_camera_pitch_deg_ = declare_parameter<double>("base_to_camera_pitch_deg", 0.0);
    base_to_camera_yaw_deg_ = declare_parameter<double>("base_to_camera_yaw_deg", 0.0);
    accumulate_cloud_ = declare_parameter<bool>("accumulate_cloud", true);
    frame_step_ = declare_parameter<int>("frame_step", 3);
    point_stride_ = declare_parameter<int>("point_stride", 8);
    max_map_points_ = declare_parameter<int>("max_map_points", 500000);

    if (input_topic_.empty()) {
      throw std::invalid_argument("Parameter 'input_topic' must not be empty");
    }
    if (output_topic_.empty()) {
      throw std::invalid_argument("Parameter 'output_topic' must not be empty");
    }
    if (target_frame_.empty()) {
      throw std::invalid_argument("Parameter 'target_frame' must not be empty");
    }
    if (transform_timeout_sec_ < 0.0) {
      throw std::invalid_argument("Parameter 'transform_timeout_sec' must be non-negative");
    }
    if (pose_source_ != "tf" && pose_source_ != "mavlink") {
      throw std::invalid_argument("Parameter 'pose_source' must be 'tf' or 'mavlink'");
    }
    if (frame_step_ <= 0 || point_stride_ <= 0 || max_map_points_ <= 0) {
      throw std::invalid_argument("frame_step, point_stride and max_map_points must be positive");
    }
    if (pose_source_ == "mavlink") {
      validate_mavlink_parameters();
      configure_mavlink_tf();
    }

    const auto input_qos = rclcpp::SensorDataQoS();
    const auto output_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    const auto transform_timeout = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(transform_timeout_sec_));

    tf_buffer_.setCreateTimerInterface(
      std::make_shared<tf2_ros::CreateTimerROS>(
        get_node_base_interface(), get_node_timers_interface()));

    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, output_qos);
    tf_filter_ = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>>(
      tf_buffer_, target_frame_, 30,
      get_node_logging_interface(), get_node_clock_interface(),
      transform_timeout);
    tf_filter_->registerCallback(
      std::bind(&PointcloudTransformNode::pointcloud_callback, this, std::placeholders::_1));

    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, input_qos,
      [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
        tf_filter_->add(message);
      });

    RCLCPP_INFO(
      get_logger(),
      "Transforming PointCloud2: %s -> %s (target frame: %s, pose source: %s)",
      input_topic_.c_str(), output_topic_.c_str(), target_frame_.c_str(), pose_source_.c_str());
    RCLCPP_INFO(
      get_logger(), "Cloud accumulation=%s, frame_step=%d, point_stride=%d, max_points=%d",
      accumulate_cloud_ ? "true" : "false", frame_step_, point_stride_, max_map_points_);
  }

private:
  using Position = geometry_msgs::msg::PointStamped;
  using Attitude = geometry_msgs::msg::QuaternionStamped;
  using PoseSyncPolicy = message_filters::sync_policies::ExactTime<Position, Attitude>;

  static double degrees_to_radians(double degrees)
  {
    constexpr double pi = 3.14159265358979323846;
    return degrees * pi / 180.0;
  }

  void validate_mavlink_parameters() const
  {
    if (mavlink_position_topic_.empty() || mavlink_attitude_topic_.empty()) {
      throw std::invalid_argument("MAVLink pose topics must not be empty");
    }
    if (mavlink_base_frame_.empty() || camera_link_frame_.empty()) {
      throw std::invalid_argument("MAVLink base and camera link frames must not be empty");
    }
    const double values[] = {
      base_to_camera_x_m_, base_to_camera_y_m_, base_to_camera_z_m_,
      base_to_camera_roll_deg_, base_to_camera_pitch_deg_, base_to_camera_yaw_deg_};
    for (const double value : values) {
      if (!std::isfinite(value)) {
        throw std::invalid_argument("Base-to-camera extrinsic values must be finite");
      }
    }
  }

  void configure_mavlink_tf()
  {
    dynamic_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);

    geometry_msgs::msg::TransformStamped camera_transform;
    camera_transform.header.stamp = now();
    camera_transform.header.frame_id = mavlink_base_frame_;
    camera_transform.child_frame_id = camera_link_frame_;
    camera_transform.transform.translation.x = base_to_camera_x_m_;
    camera_transform.transform.translation.y = base_to_camera_y_m_;
    camera_transform.transform.translation.z = base_to_camera_z_m_;
    tf2::Quaternion camera_rotation;
    camera_rotation.setRPY(
      degrees_to_radians(base_to_camera_roll_deg_),
      degrees_to_radians(base_to_camera_pitch_deg_),
      degrees_to_radians(base_to_camera_yaw_deg_));
    camera_rotation.normalize();
    camera_transform.transform.rotation.x = camera_rotation.x();
    camera_transform.transform.rotation.y = camera_rotation.y();
    camera_transform.transform.rotation.z = camera_rotation.z();
    camera_transform.transform.rotation.w = camera_rotation.w();
    static_tf_broadcaster_->sendTransform(camera_transform);

    const bool zero_extrinsic =
      base_to_camera_x_m_ == 0.0 && base_to_camera_y_m_ == 0.0 &&
      base_to_camera_z_m_ == 0.0 && base_to_camera_roll_deg_ == 0.0 &&
      base_to_camera_pitch_deg_ == 0.0 && base_to_camera_yaw_deg_ == 0.0;
    if (zero_extrinsic) {
      RCLCPP_WARN(
        get_logger(),
        "All base-to-camera extrinsic values are zero; assuming coincident frames");
    }

    position_subscription_.subscribe(this, mavlink_position_topic_);
    attitude_subscription_.subscribe(this, mavlink_attitude_topic_);
    pose_synchronizer_ = std::make_shared<message_filters::Synchronizer<PoseSyncPolicy>>(
      PoseSyncPolicy(30), position_subscription_, attitude_subscription_);
    pose_synchronizer_->registerCallback(
      std::bind(
        &PointcloudTransformNode::mavlink_pose_callback, this,
        std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(
      get_logger(), "MAVLink TF: %s -> %s -> %s, pose topics: %s and %s",
      target_frame_.c_str(), mavlink_base_frame_.c_str(), camera_link_frame_.c_str(),
      mavlink_position_topic_.c_str(), mavlink_attitude_topic_.c_str());
  }

  void mavlink_pose_callback(
    const Position::ConstSharedPtr & position,
    const Attitude::ConstSharedPtr & attitude)
  {
    if (
      position->header.frame_id != target_frame_ ||
      attitude->header.frame_id != target_frame_)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Ignoring MAVLink pose whose frame is not target frame '%s' (position='%s', attitude='%s')",
        target_frame_.c_str(), position->header.frame_id.c_str(),
        attitude->header.frame_id.c_str());
      return;
    }

    const auto & point = position->point;
    const auto & input_rotation = attitude->quaternion;
    const bool finite =
      std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z) &&
      std::isfinite(input_rotation.x) && std::isfinite(input_rotation.y) &&
      std::isfinite(input_rotation.z) && std::isfinite(input_rotation.w);
    const double quaternion_norm = std::sqrt(
      input_rotation.x * input_rotation.x + input_rotation.y * input_rotation.y +
      input_rotation.z * input_rotation.z + input_rotation.w * input_rotation.w);
    if (!finite || quaternion_norm < 1e-12) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Ignoring MAVLink pose containing an invalid position or quaternion");
      return;
    }

    geometry_msgs::msg::TransformStamped base_transform;
    base_transform.header.stamp = position->header.stamp;
    base_transform.header.frame_id = target_frame_;
    base_transform.child_frame_id = mavlink_base_frame_;
    base_transform.transform.translation.x = point.x;
    base_transform.transform.translation.y = point.y;
    base_transform.transform.translation.z = point.z;
    base_transform.transform.rotation.x = input_rotation.x / quaternion_norm;
    base_transform.transform.rotation.y = input_rotation.y / quaternion_norm;
    base_transform.transform.rotation.z = input_rotation.z / quaternion_norm;
    base_transform.transform.rotation.w = input_rotation.w / quaternion_norm;
    dynamic_tf_broadcaster_->sendTransform(base_transform);
  }

  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr message)
  {
    ++cloud_count_;
    if ((cloud_count_ - 1) % static_cast<size_t>(frame_step_) != 0) {
      return;
    }
    if (message->header.frame_id.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Ignoring PointCloud2 with an empty frame_id");
      return;
    }

    try {
      const auto transform = tf_buffer_.lookupTransform(
        target_frame_, message->header.frame_id,
        rclcpp::Time(message->header.stamp));

      sensor_msgs::msg::PointCloud2 transformed;
      tf2::doTransform(*message, transformed, transform);
      transformed.header.stamp = message->header.stamp;
      transformed.header.frame_id = target_frame_;
      if (!accumulate_cloud_) {
        publisher_->publish(transformed);
        return;
      }
      accumulate_and_publish(transformed);
    } catch (const tf2::TransformException & exception) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Cannot transform point cloud from '%s' to '%s': %s",
        message->header.frame_id.c_str(), target_frame_.c_str(), exception.what());
    }
  }

  void accumulate_and_publish(const sensor_msgs::msg::PointCloud2 & cloud)
  {
    if (cloud.point_step == 0 || cloud.data.empty()) {
      return;
    }
    uint32_t x_offset = 0;
    uint32_t y_offset = 0;
    uint32_t z_offset = 0;
    bool has_x = false;
    bool has_y = false;
    bool has_z = false;
    for (const auto & field : cloud.fields) {
      if (field.name == "x") {x_offset = field.offset; has_x = true;}
      if (field.name == "y") {y_offset = field.offset; has_y = true;}
      if (field.name == "z") {z_offset = field.offset; has_z = true;}
    }
    if (!has_x || !has_y || !has_z) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Cloud has no x/y/z fields");
      return;
    }

    if (!map_initialized_) {
      map_cloud_ = cloud;
      map_cloud_.height = 1;
      map_cloud_.width = 0;
      map_cloud_.row_step = 0;
      map_cloud_.data.clear();
      map_cloud_.is_dense = true;
      map_initialized_ = true;
    } else if (cloud.point_step != map_cloud_.point_step || cloud.fields != map_cloud_.fields) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "PointCloud2 layout changed; ignoring frame");
      return;
    }

    const size_t input_points = cloud.data.size() / cloud.point_step;
    std::vector<uint8_t> sampled;
    sampled.reserve((input_points / static_cast<size_t>(point_stride_) + 1) * cloud.point_step);
    for (size_t index = 0; index < input_points; index += static_cast<size_t>(point_stride_)) {
      const size_t offset = index * cloud.point_step;
      float x = 0.0F;
      float y = 0.0F;
      float z = 0.0F;
      std::memcpy(&x, cloud.data.data() + offset + x_offset, sizeof(float));
      std::memcpy(&y, cloud.data.data() + offset + y_offset, sizeof(float));
      std::memcpy(&z, cloud.data.data() + offset + z_offset, sizeof(float));
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }
      sampled.insert(
        sampled.end(), cloud.data.begin() + offset,
        cloud.data.begin() + offset + cloud.point_step);
    }
    map_cloud_.data.insert(map_cloud_.data.end(), sampled.begin(), sampled.end());

    size_t map_points = map_cloud_.data.size() / map_cloud_.point_step;
    if (map_points > static_cast<size_t>(max_map_points_)) {
      std::vector<uint8_t> reduced;
      reduced.resize(static_cast<size_t>(max_map_points_) * map_cloud_.point_step);
      for (size_t output_index = 0; output_index < static_cast<size_t>(max_map_points_);
        ++output_index)
      {
        const size_t input_index = output_index * map_points / static_cast<size_t>(max_map_points_);
        std::memcpy(
          reduced.data() + output_index * map_cloud_.point_step,
          map_cloud_.data.data() + input_index * map_cloud_.point_step,
          map_cloud_.point_step);
      }
      map_cloud_.data.swap(reduced);
      map_points = static_cast<size_t>(max_map_points_);
    }
    map_cloud_.header = cloud.header;
    map_cloud_.header.frame_id = target_frame_;
    map_cloud_.height = 1;
    map_cloud_.width = static_cast<uint32_t>(map_points);
    map_cloud_.row_step = map_cloud_.point_step * map_cloud_.width;
    publisher_->publish(map_cloud_);
    ++map_frame_count_;
    if (map_frame_count_ == 1 || map_frame_count_ % 10 == 0) {
      RCLCPP_INFO(
        get_logger(), "Accumulated frames=%zu, new_points=%zu, map_points=%zu",
        map_frame_count_, sampled.size() / map_cloud_.point_step, map_points);
    }
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string target_frame_;
  std::string pose_source_;
  std::string mavlink_position_topic_;
  std::string mavlink_attitude_topic_;
  std::string mavlink_base_frame_;
  std::string camera_link_frame_;
  double transform_timeout_sec_;
  double base_to_camera_x_m_;
  double base_to_camera_y_m_;
  double base_to_camera_z_m_;
  double base_to_camera_roll_deg_;
  double base_to_camera_pitch_deg_;
  double base_to_camera_yaw_deg_;
  bool accumulate_cloud_;
  int frame_step_;
  int point_stride_;
  int max_map_points_;
  size_t cloud_count_{0};
  size_t map_frame_count_{0};
  bool map_initialized_{false};
  sensor_msgs::msg::PointCloud2 map_cloud_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> dynamic_tf_broadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  message_filters::Subscriber<Position> position_subscription_;
  message_filters::Subscriber<Attitude> attitude_subscription_;
  std::shared_ptr<message_filters::Synchronizer<PoseSyncPolicy>> pose_synchronizer_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>> tf_filter_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(std::make_shared<PointcloudTransformNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("pointcloud_transform_node"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
