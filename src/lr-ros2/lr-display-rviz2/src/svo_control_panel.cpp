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

#include "lr_display_rviz2/svo_control_panel.hpp"

#include <QDockWidget>
#include <QFileInfo>
#include <QFont>
#include <QHBoxLayout>
#include <QLabel>
#include <QMetaObject>
#include <QPushButton>
#include <QSlider>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>
#include <algorithm>
#include <limits>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

namespace lr_display_rviz2
{

SvoControlPanel::SvoControlPanel(QWidget * parent)
: rviz_common::Panel(parent),
  mode_label_(new QLabel("Waiting for SVO playback...", this)),
  file_label_(new QLabel("No SVO status received", this)),
  frame_label_(new QLabel("Frame: -- / --", this)),
  feedback_label_(new QLabel(this)),
  position_slider_(new QSlider(Qt::Horizontal, this)),
  restart_button_(new QPushButton("|<", this)),
  play_pause_button_(new QPushButton("Play", this)),
  refresh_timer_(new QTimer(this))
{
  QFont mode_font = mode_label_->font();
  mode_font.setBold(true);
  mode_label_->setFont(mode_font);

  file_label_->setWordWrap(true);
  position_slider_->setRange(0, 0);
  position_slider_->setEnabled(false);
  restart_button_->setToolTip("Seek to the first SVO frame");
  play_pause_button_->setToolTip("Pause or resume SVO playback");
  restart_button_->setEnabled(false);
  play_pause_button_->setEnabled(false);
  feedback_label_->setWordWrap(true);

  auto * button_layout = new QHBoxLayout();
  button_layout->addWidget(restart_button_);
  button_layout->addWidget(play_pause_button_, 1);

  auto * layout = new QVBoxLayout(this);
  layout->addWidget(mode_label_);
  layout->addWidget(file_label_);
  layout->addWidget(position_slider_);
  layout->addWidget(frame_label_);
  layout->addLayout(button_layout);
  layout->addWidget(feedback_label_);
  layout->addStretch(1);
  setLayout(layout);

  connect(play_pause_button_, &QPushButton::clicked, this, &SvoControlPanel::togglePlayback);
  connect(restart_button_, &QPushButton::clicked, this, &SvoControlPanel::seekToStart);
  connect(position_slider_, &QSlider::sliderReleased, this, &SvoControlPanel::seekFromSlider);
  connect(refresh_timer_, &QTimer::timeout, this, &SvoControlPanel::refreshControls);
}

SvoControlPanel::~SvoControlPanel()
{
  refresh_timer_->stop();
  status_subscription_.reset();
  toggle_client_.reset();
  seek_client_.reset();
}

void SvoControlPanel::onInitialize()
{
  auto node_abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!node_abstraction) {
    setFeedback("RViz ROS node is unavailable", true);
    return;
  }

  node_ = node_abstraction->get_raw_node();
  status_subscription_ = node_->create_subscription<zed_msgs::msg::SvoStatus>(
    "zed_node/status/svo", rclcpp::QoS(10),
    [this](zed_msgs::msg::SvoStatus::SharedPtr msg) {  // Forward to the Qt thread.
      handleStatus(msg);
    });
  toggle_client_ = node_->create_client<std_srvs::srv::Trigger>("zed_node/toggle_svo_pause");
  seek_client_ = node_->create_client<zed_msgs::srv::SetSvoFrame>("zed_node/set_svo_frame");

  refresh_timer_->start(500);

  bool svo_mode = true;
  if (node_->has_parameter("svo_mode")) {
    svo_mode = node_->get_parameter("svo_mode").as_bool();
  } else {
    svo_mode = node_->declare_parameter<bool>("svo_mode", true);
  }
  if (!svo_mode) {
    QTimer::singleShot(0, this, &SvoControlPanel::hideDockWhenLive);
  }
}

void SvoControlPanel::handleStatus(const zed_msgs::msg::SvoStatus::SharedPtr msg)
{
  QMetaObject::invokeMethod(
    this,
    [this, msg]() {  // Widgets may only be updated by the Qt thread.
      updateStatus(*msg);
    },
    Qt::QueuedConnection);
}

void SvoControlPanel::updateStatus(const zed_msgs::msg::SvoStatus & msg)
{
  received_status_ = true;
  playback_status_ = msg.status;
  real_time_mode_ = msg.real_time_mode;

  const QFileInfo file_info(QString::fromStdString(msg.file_name));
  const QString file_name =
    file_info.fileName().isEmpty() ? QString::fromStdString(msg.file_name) : file_info.fileName();
  file_label_->setText(file_name);
  file_label_->setToolTip(QString::fromStdString(msg.file_name));

  const uint32_t last_frame = msg.total_frames > 0 ? msg.total_frames - 1 : 0;
  const int slider_max = static_cast<int>(
    std::min<uint32_t>(last_frame, static_cast<uint32_t>(std::numeric_limits<int>::max())));
  position_slider_->setRange(0, slider_max);
  if (!position_slider_->isSliderDown()) {
    position_slider_->setValue(static_cast<int>(std::min<uint32_t>(msg.frame_id, slider_max)));
  }

  frame_label_->setText(QString("Frame: %1 / %2").arg(msg.frame_id).arg(last_frame));

  switch (msg.status) {
    case zed_msgs::msg::SvoStatus::STATUS_PAUSED:
      mode_label_->setText("Paused");
      play_pause_button_->setText("Play");
      break;
    case zed_msgs::msg::SvoStatus::STATUS_END:
      mode_label_->setText("End of SVO");
      play_pause_button_->setText("Replay");
      break;
    default:
      mode_label_->setText(msg.real_time_mode ? "Playing (real time)" : "Playing");
      play_pause_button_->setText("Pause");
      break;
  }

  if (msg.loop_active) {
    mode_label_->setText(mode_label_->text() + QString(" - loop %1").arg(msg.loop_count));
  }
  refreshControls();
}

void SvoControlPanel::refreshControls()
{
  const bool seek_ready = seek_client_ && seek_client_->service_is_ready();
  const bool toggle_ready = toggle_client_ && toggle_client_->service_is_ready();
  const bool active = received_status_ && !request_pending_;

  position_slider_->setEnabled(active && seek_ready && position_slider_->maximum() > 0);
  restart_button_->setEnabled(active && seek_ready);
  const bool at_end = playback_status_ == zed_msgs::msg::SvoStatus::STATUS_END;
  const bool replay_ready = at_end && seek_ready;
  const bool pause_ready = toggle_ready;
  play_pause_button_->setEnabled(active && (replay_ready || pause_ready));

  if (received_status_ && !at_end && !toggle_ready) {
    play_pause_button_->setToolTip("Pause/play service is unavailable");
  } else {
    play_pause_button_->setToolTip("Pause or resume SVO playback");
  }
}

void SvoControlPanel::togglePlayback()
{
  if (playback_status_ == zed_msgs::msg::SvoStatus::STATUS_END) {
    seekToFrame(0);
    return;
  }
  if (!toggle_client_ || !toggle_client_->service_is_ready() || request_pending_) {
    setFeedback("Pause/play service is unavailable", true);
    return;
  }

  request_pending_ = true;
  refreshControls();
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  toggle_client_->async_send_request(
    request, [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
      const auto response = future.get();
      QMetaObject::invokeMethod(
        this,
        [this, response]() {
          request_pending_ = false;
          setFeedback(QString::fromStdString(response->message), !response->success);
          refreshControls();
        },
        Qt::QueuedConnection);
    });
}

void SvoControlPanel::seekToStart()
{
  // Keep this as a slot so it can be connected directly to the button.
  seekToFrame(0);
}

void SvoControlPanel::seekFromSlider()
{
  // Seek only on release to respect the wrapper's rate limit.
  seekToFrame(position_slider_->value());
}

void SvoControlPanel::seekToFrame(int frame)
{
  if (!seek_client_ || !seek_client_->service_is_ready() || request_pending_) {
    setFeedback("SVO seek service is unavailable", true);
    return;
  }

  request_pending_ = true;
  refreshControls();
  auto request = std::make_shared<zed_msgs::srv::SetSvoFrame::Request>();
  request->frame_id = frame;
  seek_client_->async_send_request(
    request, [this](rclcpp::Client<zed_msgs::srv::SetSvoFrame>::SharedFuture future) {
      const auto response = future.get();
      QMetaObject::invokeMethod(
        this,
        [this, response]() {
          request_pending_ = false;
          setFeedback(QString::fromStdString(response->message), !response->success);
          refreshControls();
        },
        Qt::QueuedConnection);
    });
}

void SvoControlPanel::setFeedback(const QString & text, bool error)
{
  feedback_label_->setText(text);
  feedback_label_->setStyleSheet(error ? "color: #e57373;" : "");
}

void SvoControlPanel::hideDockWhenLive()
{
  QWidget * widget = this;
  while (widget && !qobject_cast<QDockWidget *>(widget)) {
    widget = widget->parentWidget();
  }
  if (widget) {
    widget->hide();
  } else {
    hide();
  }
}

}  // namespace lr_display_rviz2

PLUGINLIB_EXPORT_CLASS(lr_display_rviz2::SvoControlPanel, rviz_common::Panel)
