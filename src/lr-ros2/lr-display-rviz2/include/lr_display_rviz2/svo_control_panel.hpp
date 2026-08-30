// Copyright 2026 LR
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

#ifndef LR_DISPLAY_RVIZ2__SVO_CONTROL_PANEL_HPP_
#define LR_DISPLAY_RVIZ2__SVO_CONTROL_PANEL_HPP_

#include <cstdint>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <zed_msgs/msg/svo_status.hpp>
#include <zed_msgs/srv/set_svo_frame.hpp>

class QLabel;
class QPushButton;
class QSlider;
class QTimer;
class QWidget;

namespace lr_display_rviz2
{

class SvoControlPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit SvoControlPanel(QWidget * parent = nullptr);
  ~SvoControlPanel() override;

  void onInitialize() override;

private Q_SLOTS:
  void togglePlayback();
  void seekToStart();
  void seekFromSlider();
  void refreshControls();

private:
  void handleStatus(const zed_msgs::msg::SvoStatus::SharedPtr msg);
  void updateStatus(const zed_msgs::msg::SvoStatus & msg);
  void seekToFrame(int frame);
  void setFeedback(const QString & text, bool error = false);
  void hideDockWhenLive();

  QLabel * mode_label_;
  QLabel * file_label_;
  QLabel * frame_label_;
  QLabel * feedback_label_;
  QSlider * position_slider_;
  QPushButton * restart_button_;
  QPushButton * play_pause_button_;
  QTimer * refresh_timer_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<zed_msgs::msg::SvoStatus>::SharedPtr status_subscription_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr toggle_client_;
  rclcpp::Client<zed_msgs::srv::SetSvoFrame>::SharedPtr seek_client_;

  uint8_t playback_status_ = zed_msgs::msg::SvoStatus::STATUS_END;
  bool received_status_ = false;
  bool real_time_mode_ = false;
  bool request_pending_ = false;
};

}  // namespace lr_display_rviz2

#endif  // LR_DISPLAY_RVIZ2__SVO_CONTROL_PANEL_HPP_
