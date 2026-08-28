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

#include "lr_display_rviz2/svo_playback_panel.hpp"

#include <QFileInfo>
#include <QFont>
#include <QHBoxLayout>
#include <QLabel>
#include <QMetaObject>
#include <QPushButton>
#include <QSlider>
#include <QString>
#include <QVBoxLayout>

#include <algorithm>
#include <cstdint>
#include <exception>
#include <limits>
#include <memory>
#include <string>

#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"

namespace lr_display_rviz2
{

namespace
{
constexpr int kServiceTimeoutMs = 2000;
constexpr int kSeekCooldownMs = 550;

QString status_text(uint8_t status)
{
  switch (status) {
    case zed_msgs::msg::SvoStatus::STATUS_PLAYING:
      return "Playing";
    case zed_msgs::msg::SvoStatus::STATUS_PAUSED:
      return "Paused";
    case zed_msgs::msg::SvoStatus::STATUS_END:
      return "End of SVO";
    default:
      return "Unknown";
  }
}
}  // namespace

SvoPlaybackPanel::SvoPlaybackPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  auto * layout = new QVBoxLayout(this);
  layout->setContentsMargins(8, 8, 8, 8);

  file_label_ = new QLabel("No SVO status", this);
  file_label_->setWordWrap(true);
  QFont file_font = file_label_->font();
  file_font.setBold(true);
  file_label_->setFont(file_font);
  layout->addWidget(file_label_);

  status_label_ = new QLabel("SVO unavailable", this);
  layout->addWidget(status_label_);

  slider_ = new QSlider(Qt::Horizontal, this);
  slider_->setRange(0, 0);
  slider_->setTracking(true);
  slider_->setToolTip("Drag and release to seek to an SVO frame");
  layout->addWidget(slider_);

  frame_label_ = new QLabel("Frame -- / --", this);
  frame_label_->setAlignment(Qt::AlignCenter);
  layout->addWidget(frame_label_);

  auto * controls = new QHBoxLayout();
  back_ten_button_ = new QPushButton("-10", this);
  back_one_button_ = new QPushButton("-1", this);
  play_pause_button_ = new QPushButton("Play", this);
  forward_one_button_ = new QPushButton("+1", this);
  forward_ten_button_ = new QPushButton("+10", this);
  back_ten_button_->setToolTip("Seek backward 10 frames and preserve playback state");
  forward_ten_button_->setToolTip("Seek forward 10 frames and preserve playback state");
  back_one_button_->setToolTip("Step backward one frame and pause");
  forward_one_button_->setToolTip("Step forward one frame and pause");
  controls->addWidget(back_ten_button_);
  controls->addWidget(back_one_button_);
  controls->addWidget(play_pause_button_, 1);
  controls->addWidget(forward_one_button_);
  controls->addWidget(forward_ten_button_);
  layout->addLayout(controls);

  feedback_label_ = new QLabel("Waiting for ZED SVO status...", this);
  feedback_label_->setWordWrap(true);
  layout->addWidget(feedback_label_);
  layout->addStretch(1);

  service_timeout_.setSingleShot(true);
  cooldown_timer_.setSingleShot(true);

  connect(
    slider_, &QSlider::sliderPressed, this, [this]() {
      dragging_ = true;
      set_feedback("Release the slider to seek");
    });
  connect(
    slider_, &QSlider::sliderMoved, this, [this](int value) {
      frame_label_->setText(
        QString("Frame %1 / %2").arg(value).arg(total_frames_));
    });
  connect(
    slider_, &QSlider::sliderReleased, this, [this]() {
      dragging_ = false;
      request_seek(slider_->value(), SeekPolicy::PreservePlayback);
    });
  connect(
    play_pause_button_, &QPushButton::clicked, this, [this]() {
      toggle_playback();
    });
  connect(
    back_ten_button_, &QPushButton::clicked, this, [this]() {
      request_relative_seek(-10, SeekPolicy::PreservePlayback);
    });
  connect(
    forward_ten_button_, &QPushButton::clicked, this, [this]() {
      request_relative_seek(10, SeekPolicy::PreservePlayback);
    });
  connect(
    back_one_button_, &QPushButton::clicked, this, [this]() {
      request_relative_seek(-1, SeekPolicy::PauseAtTarget);
    });
  connect(
    forward_one_button_, &QPushButton::clicked, this, [this]() {
      request_relative_seek(1, SeekPolicy::PauseAtTarget);
    });

  update_controls();
}

void SvoPlaybackPanel::onInitialize()
{
  rviz_common::Panel::onInitialize();
  const auto abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!abstraction) {
    set_feedback("RViz ROS node is unavailable", true);
    return;
  }

  node_ = abstraction->get_raw_node();
  if (!node_) {
    set_feedback("RViz ROS node is unavailable", true);
    return;
  }

  const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  status_subscription_ = node_->create_subscription<SvoStatus>(
    "zed_node/status/svo", qos,
    [this](const SvoStatus::SharedPtr message) {status_callback(message);});
  toggle_client_ = node_->create_client<Trigger>("zed_node/toggle_svo_pause");
  seek_client_ = node_->create_client<SetSvoFrame>("zed_node/set_svo_frame");

  set_feedback("Waiting for ZED SVO status...");
}

void SvoPlaybackPanel::status_callback(const SvoStatus::SharedPtr message)
{
  const SvoStatus copy = *message;
  QPointer<SvoPlaybackPanel> guard(this);
  QMetaObject::invokeMethod(
    this,
    [guard, copy]() {
      if (guard) {
        guard->apply_status(copy);
      }
    },
    Qt::QueuedConnection);
}

void SvoPlaybackPanel::apply_status(const SvoStatus & message)
{
  const bool first_status = !have_status_;
  have_status_ = true;
  current_frame_ = message.frame_id;
  total_frames_ = message.total_frames;
  status_ = message.status;
  file_name_ = message.file_name;
  if (first_status) {
    set_feedback("SVO controls ready");
  }
  update_ui();
}

void SvoPlaybackPanel::update_ui()
{
  if (!have_status_) {
    file_label_->setText("No SVO status");
    status_label_->setText("SVO unavailable");
    frame_label_->setText("Frame -- / --");
    play_pause_button_->setText("Play");
    update_controls();
    return;
  }

  if (!dragging_ && !seek_transaction_.active() && !user_toggle_active_) {
    const uint32_t slider_maximum = total_frames_ == 0U ? 0U : total_frames_ - 1U;
    const uint32_t safe_maximum = std::min<uint32_t>(
      slider_maximum, static_cast<uint32_t>(std::numeric_limits<int>::max()));
    slider_->setRange(0, static_cast<int>(safe_maximum));
    slider_->setValue(static_cast<int>(std::min(current_frame_, safe_maximum)));
  }

  const QFileInfo file(QString::fromStdString(file_name_));
  file_label_->setText(file.fileName().isEmpty() ? "SVO playback" : file.fileName());
  status_label_->setText("Status: " + status_text(status_));
  if (!dragging_) {
    frame_label_->setText(
      QString("Frame %1 / %2").arg(current_frame_).arg(total_frames_));
  }
  play_pause_button_->setText(
    status_ == SvoStatus::STATUS_PLAYING ? "Pause" :
    status_ == SvoStatus::STATUS_END ? "Restart" : "Play");
  update_controls();
}

void SvoPlaybackPanel::update_controls()
{
  const bool services_ready = toggle_client_ && seek_client_ &&
    toggle_client_->service_is_ready() && seek_client_->service_is_ready();
  const bool busy = seek_transaction_.active() || user_toggle_active_;
  const bool enabled = have_status_ && total_frames_ > 0U && services_ready && !busy;
  slider_->setEnabled(enabled);
  play_pause_button_->setEnabled(enabled);
  back_ten_button_->setEnabled(enabled && current_frame_ > 0U);
  back_one_button_->setEnabled(enabled && current_frame_ > 0U);
  forward_one_button_->setEnabled(enabled && current_frame_ + 1U < total_frames_);
  forward_ten_button_->setEnabled(enabled && current_frame_ + 1U < total_frames_);
}

void SvoPlaybackPanel::set_feedback(const QString & text, bool error)
{
  feedback_label_->setText(text);
  feedback_label_->setStyleSheet(error ? "QLabel { color: #e57373; }" : QString());
}

PlaybackState SvoPlaybackPanel::playback_state() const
{
  if (status_ == SvoStatus::STATUS_PLAYING) {
    return PlaybackState::Playing;
  }
  if (status_ == SvoStatus::STATUS_PAUSED) {
    return PlaybackState::Paused;
  }
  return PlaybackState::End;
}

void SvoPlaybackPanel::toggle_playback()
{
  if (!have_status_ || seek_transaction_.active() || user_toggle_active_) {
    return;
  }
  if (status_ == SvoStatus::STATUS_END) {
    request_seek(0, SeekPolicy::ResumeAtTarget);
    return;
  }

  user_toggle_active_ = true;
  user_toggle_target_status_ = status_ == SvoStatus::STATUS_PLAYING ?
    SvoStatus::STATUS_PAUSED : SvoStatus::STATUS_PLAYING;
  const uint64_t token = ++operation_token_;
  set_feedback(
    user_toggle_target_status_ == SvoStatus::STATUS_PAUSED ?
    "Pausing SVO..." : "Starting SVO...");
  update_controls();
  call_toggle(TogglePurpose::User, token);
}

void SvoPlaybackPanel::request_relative_seek(int offset, SeekPolicy policy)
{
  request_seek(static_cast<int64_t>(current_frame_) + offset, policy);
}

void SvoPlaybackPanel::request_seek(int64_t target, SeekPolicy policy)
{
  if (!have_status_ || total_frames_ == 0U || user_toggle_active_) {
    return;
  }
  if (!seek_transaction_.begin(target, total_frames_, playback_state(), policy)) {
    return;
  }

  const uint64_t token = ++operation_token_;
  update_controls();
  if (seek_transaction_.stage() == SeekStage::Pausing) {
    set_feedback("Pausing before seek...");
    call_toggle(TogglePurpose::PauseForSeek, token);
  } else {
    dispatch_seek_when_ready(token);
  }
}

void SvoPlaybackPanel::dispatch_seek_when_ready(uint64_t token)
{
  if (token != operation_token_ || seek_transaction_.stage() != SeekStage::SeekPending) {
    return;
  }
  if (last_seek_dispatch_.isValid() && last_seek_dispatch_.elapsed() < kSeekCooldownMs) {
    const int remaining = kSeekCooldownMs - static_cast<int>(last_seek_dispatch_.elapsed());
    set_feedback("Waiting for SVO seek cooldown...");
    cooldown_timer_.stop();
    cooldown_timer_.disconnect();
    connect(
      &cooldown_timer_, &QTimer::timeout, this, [this, token]() {
        dispatch_seek_when_ready(token);
      });
    cooldown_timer_.start(std::max(1, remaining));
    return;
  }
  dispatch_seek(token);
}

void SvoPlaybackPanel::dispatch_seek(uint64_t token)
{
  if (token != operation_token_ || !seek_transaction_.seek_started()) {
    return;
  }
  if (!seek_client_ || !seek_client_->service_is_ready()) {
    fail_operation("SVO seek service is unavailable");
    return;
  }

  auto request = std::make_shared<SetSvoFrame::Request>();
  request->frame_id = seek_transaction_.target_frame();
  set_feedback(QString("Seeking to frame %1...").arg(request->frame_id));
  start_timeout(token, "SVO seek");
  last_seek_dispatch_.restart();

  QPointer<SvoPlaybackPanel> guard(this);
  seek_client_->async_send_request(
    request,
    [guard, token](rclcpp::Client<SetSvoFrame>::SharedFuture future) {
      bool success = false;
      std::string message;
      try {
        const auto response = future.get();
        success = response->success;
        message = response->message;
      } catch (const std::exception & exception) {
        message = exception.what();
      }
      if (!guard) {
        return;
      }
      QMetaObject::invokeMethod(
        guard.data(),
        [guard, token, success, message]() {
          if (guard) {
            guard->handle_seek_response(token, success, message);
          }
        },
        Qt::QueuedConnection);
    });
}

void SvoPlaybackPanel::call_toggle(TogglePurpose purpose, uint64_t token)
{
  if (!toggle_client_ || !toggle_client_->service_is_ready()) {
    fail_operation("SVO play/pause service is unavailable");
    return;
  }

  auto request = std::make_shared<Trigger::Request>();
  start_timeout(token, "SVO play/pause");
  QPointer<SvoPlaybackPanel> guard(this);
  toggle_client_->async_send_request(
    request,
    [guard, purpose, token](rclcpp::Client<Trigger>::SharedFuture future) {
      bool success = false;
      std::string message;
      try {
        const auto response = future.get();
        success = response->success;
        message = response->message;
      } catch (const std::exception & exception) {
        message = exception.what();
      }
      if (!guard) {
        return;
      }
      QMetaObject::invokeMethod(
        guard.data(),
        [guard, purpose, token, success, message]() {
          if (guard) {
            guard->handle_toggle_response(purpose, token, success, message);
          }
        },
        Qt::QueuedConnection);
    });
}

void SvoPlaybackPanel::handle_toggle_response(
  TogglePurpose purpose, uint64_t token, bool success,
  const std::string & message)
{
  if (token != operation_token_) {
    return;
  }
  service_timeout_.stop();
  if (!success) {
    fail_operation(
      QString::fromStdString(
        message.empty() ? "SVO play/pause failed" : message));
    return;
  }

  if (purpose == TogglePurpose::User) {
    status_ = user_toggle_target_status_;
    user_toggle_active_ = false;
    finish_operation(QString::fromStdString(message));
    return;
  }
  if (purpose == TogglePurpose::PauseForSeek) {
    status_ = SvoStatus::STATUS_PAUSED;
    if (!seek_transaction_.pause_succeeded()) {
      fail_operation("Unexpected seek state after pausing");
      return;
    }
    dispatch_seek_when_ready(token);
    return;
  }

  status_ = SvoStatus::STATUS_PLAYING;
  if (!seek_transaction_.resume_succeeded()) {
    fail_operation("Unexpected seek state after resuming");
    return;
  }
  finish_operation(QString("Playing from frame %1").arg(current_frame_));
}

void SvoPlaybackPanel::handle_seek_response(
  uint64_t token, bool success, const std::string & message)
{
  if (token != operation_token_) {
    return;
  }
  service_timeout_.stop();
  if (!success) {
    fail_operation(
      QString::fromStdString(
        message.empty() ? "SVO seek failed" : message));
    return;
  }

  current_frame_ = static_cast<uint32_t>(seek_transaction_.target_frame());
  status_ = SvoStatus::STATUS_PAUSED;
  if (!seek_transaction_.seek_succeeded()) {
    fail_operation("Unexpected seek state after setting frame");
    return;
  }
  if (seek_transaction_.stage() == SeekStage::Resuming) {
    set_feedback("Resuming SVO...");
    call_toggle(TogglePurpose::ResumeAfterSeek, token);
    return;
  }
  finish_operation(QString("Paused at frame %1").arg(current_frame_));
}

void SvoPlaybackPanel::start_timeout(uint64_t token, const QString & operation)
{
  service_timeout_.stop();
  service_timeout_.disconnect();
  connect(
    &service_timeout_, &QTimer::timeout, this, [this, token, operation]() {
      if (token == operation_token_) {
        fail_operation(operation + " timed out");
      }
    });
  service_timeout_.start(kServiceTimeoutMs);
}

void SvoPlaybackPanel::finish_operation(const QString & feedback)
{
  service_timeout_.stop();
  cooldown_timer_.stop();
  seek_transaction_.reset();
  user_toggle_active_ = false;
  ++operation_token_;
  set_feedback(feedback);
  update_ui();
}

void SvoPlaybackPanel::fail_operation(const QString & feedback)
{
  service_timeout_.stop();
  cooldown_timer_.stop();
  seek_transaction_.fail();
  seek_transaction_.reset();
  user_toggle_active_ = false;
  ++operation_token_;
  set_feedback(feedback, true);
  update_ui();
}

}  // namespace lr_display_rviz2

PLUGINLIB_EXPORT_CLASS(lr_display_rviz2::SvoPlaybackPanel, rviz_common::Panel)
