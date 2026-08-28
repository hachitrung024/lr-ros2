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

#ifndef LR_DISPLAY_RVIZ2__SVO_PLAYBACK_PANEL_HPP_
#define LR_DISPLAY_RVIZ2__SVO_PLAYBACK_PANEL_HPP_

#include <QElapsedTimer>
#include <QPointer>
#include <QTimer>

#include <cstdint>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "zed_msgs/msg/svo_status.hpp"
#include "zed_msgs/srv/set_svo_frame.hpp"

#include "lr_display_rviz2/svo_playback_logic.hpp"

class QLabel;
class QPushButton;
class QSlider;
class QString;
class QWidget;

namespace lr_display_rviz2
{

class SvoPlaybackPanel : public rviz_common::Panel
{
public:
  explicit SvoPlaybackPanel(QWidget * parent = nullptr);
  ~SvoPlaybackPanel() override = default;

  void onInitialize() override;

private:
  using SvoStatus = zed_msgs::msg::SvoStatus;
  using Trigger = std_srvs::srv::Trigger;
  using SetSvoFrame = zed_msgs::srv::SetSvoFrame;

  enum class TogglePurpose
  {
    User,
    PauseForSeek,
    ResumeAfterSeek,
  };

  void status_callback(const SvoStatus::SharedPtr message);
  void apply_status(const SvoStatus & message);
  void update_ui();
  void update_controls();
  void set_feedback(const QString & text, bool error = false);

  void toggle_playback();
  void request_relative_seek(int offset, SeekPolicy policy);
  void request_seek(int64_t target, SeekPolicy policy);
  void dispatch_seek_when_ready(uint64_t token);
  void dispatch_seek(uint64_t token);
  void call_toggle(TogglePurpose purpose, uint64_t token);

  void handle_toggle_response(
    TogglePurpose purpose, uint64_t token, bool success,
    const std::string & message);
  void handle_seek_response(
    uint64_t token, bool success, const std::string & message);
  void start_timeout(uint64_t token, const QString & operation);
  void finish_operation(const QString & feedback);
  void fail_operation(const QString & feedback);
  PlaybackState playback_state() const;

  QLabel * file_label_{nullptr};
  QLabel * status_label_{nullptr};
  QLabel * frame_label_{nullptr};
  QLabel * feedback_label_{nullptr};
  QSlider * slider_{nullptr};
  QPushButton * back_ten_button_{nullptr};
  QPushButton * back_one_button_{nullptr};
  QPushButton * play_pause_button_{nullptr};
  QPushButton * forward_one_button_{nullptr};
  QPushButton * forward_ten_button_{nullptr};
  QTimer service_timeout_;
  QTimer cooldown_timer_;
  QElapsedTimer last_seek_dispatch_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<SvoStatus>::SharedPtr status_subscription_;
  rclcpp::Client<Trigger>::SharedPtr toggle_client_;
  rclcpp::Client<SetSvoFrame>::SharedPtr seek_client_;

  SeekTransaction seek_transaction_;
  uint64_t operation_token_{0};
  uint32_t current_frame_{0};
  uint32_t total_frames_{0};
  uint8_t status_{SvoStatus::STATUS_END};
  uint8_t user_toggle_target_status_{SvoStatus::STATUS_PAUSED};
  std::string file_name_;
  bool have_status_{false};
  bool dragging_{false};
  bool user_toggle_active_{false};
};

}  // namespace lr_display_rviz2

#endif  // LR_DISPLAY_RVIZ2__SVO_PLAYBACK_PANEL_HPP_
