#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"
#include "mavros_msgs/msg/timesync_status.hpp"
#include "std_msgs/msg/string.hpp"
#include "lr_mavros_bridge/live_time.hpp"
#include "lr_mavros_bridge/sync_health.hpp"
#include <mutex>

namespace lr_mavros_bridge {
class SourceTimePlugin : public mavros::plugin::Plugin {
public:
  explicit SourceTimePlugin(mavros::plugin::UASPtr uas) : Plugin(uas, "source_time") {
    rclcpp::PublisherOptions options;
    options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
    publisher_ = node->create_publisher<std_msgs::msg::String>(
      "/lr/source/time_state", rclcpp::QoS(1).reliable().transient_local(), options);
    sync_sub_ = node->create_subscription<mavros_msgs::msg::TimesyncStatus>(
      "/mavros/timesync_status", rclcpp::SensorDataQoS(),
      [this](mavros_msgs::msg::TimesyncStatus::ConstSharedPtr m) {
        std::lock_guard<std::mutex> guard(mutex_);
        if (!connected_) return;
        const auto epoch = health_.epoch;
        health_.observe(steady_.now().nanoseconds(), node->now().nanoseconds(),
          m->remote_timestamp_ns, m->observed_offset_ns, m->estimated_offset_ns,
          int64_t(this->uas->get_time_offset()), m->round_trip_time_ms);
        if (health_.epoch != epoch) utc_.reset();
      });
    enable_connection_cb();
    timer_ = node->create_wall_timer(std::chrono::milliseconds(100), [this]() { publish(); });
  }
  Subscriptions get_subscriptions() override {
    return {make_handler(&SourceTimePlugin::handle), make_handler(&SourceTimePlugin::mission),
            make_handler(&SourceTimePlugin::system_time)};
  }
private:
  void connection_cb(bool connected) override {
    std::lock_guard<std::mutex> guard(mutex_);
    if (connected != connected_) { health_.reset(); utc_.reset(); }
    connected_ = connected;
    if (!connected) { uas->set_time_offset(0); last_session_ns_ = 0; mission_seen_ = 0; }
  }
  void handle(const mavlink::mavlink_message_t *, mavlink::common::msg::TUNNEL &packet,
              mavros::plugin::filter::SystemAndOk) {
    if (packet.payload_type != 32768) return;
    std::lock_guard<std::mutex> guard(mutex_);
    if (session_.accept(packet.payload.data(), packet.payload_length))
      last_session_ns_ = steady_.now().nanoseconds();
  }
  void mission(const mavlink::mavlink_message_t *msg,
               mavlink::common::msg::MISSION_CURRENT &packet,
               mavros::plugin::filter::SystemAndOk) {
    std::lock_guard<std::mutex> guard(mutex_);
    mission_seen_ = steady_.now().nanoseconds(); mission_seq_ = packet.seq;
    mission_system_ = msg->sysid; mission_component_ = msg->compid;
    mission_state_ = packet.mission_state; mission_mode_ = packet.mission_mode;
    mission_total_ = packet.total; mission_id_ = packet.mission_id;
  }
  void system_time(const mavlink::mavlink_message_t *msg,
                   mavlink::common::msg::SYSTEM_TIME &packet,
                   mavros::plugin::filter::SystemAndOk) {
    if (msg->compid != uas->get_tgt_component()) return;
    std::lock_guard<std::mutex> guard(mutex_);
    if (health_.seen && std::abs(double(health_.remote) -
        double(packet.time_boot_ms)*1000000.) > 2000000000.) {
      utc_.reset(); return;  // Late anchors from a previous boot cannot revive fusion.
    }
    utc_.observe(steady_.now().nanoseconds(), packet.time_unix_usec, packet.time_boot_ms);
  }
  void publish() {
    std::lock_guard<std::mutex> guard(mutex_);
    const auto now = steady_.now().nanoseconds();
    health_.expire(now);
    const auto offset = int64_t(uas->get_time_offset());
    const bool calibrated = last_session_ns_ > 0 && now - last_session_ns_ < 3000000000LL;
    Json state = {{"version", 1}, {"connected", connected_},
      {"sync_ready", connected_ && health_.ready(now) &&
        std::abs(double(offset)-double(health_.offset)) <= 1000000.}, {"offset_ns", offset},
      {"sync_samples", health_.samples},
      {"sync_age_sec", health_.seen ? (now - health_.seen) / 1e9 : -1.},
      {"rtt_ms", health_.rtt}, {"telemetry_epoch", health_.epoch},
      {"utc_ready", connected_ && utc_.ready(now)},
      {"utc_samples", utc_.samples}, {"utc_minus_boot_ns", utc_.utc_minus_boot},
      {"utc_age_sec", utc_.seen ? (now-utc_.seen)/1e9 : -1.},
      {"session_id", calibrated ? session_.id : ""}, {"sequence", session_.sequence},
      {"record_anchor_ns", calibrated ? session_.record_ns : 0},
      {"boot_anchor_ns", calibrated ? session_.boot_ns : 0},
      {"mission_seq", mission_seq_},
      {"mission_state", mission_state_}, {"mission_mode", mission_mode_},
      {"mission_total", mission_total_}, {"mission_id", mission_id_},
      {"mission_age_sec", mission_seen_ ? (now - mission_seen_) / 1e9 : -1.},
      {"mission_system_id", mission_system_}, {"mission_component_id", mission_component_}};
    std_msgs::msg::String message; message.data = state.dump(); publisher_->publish(message);
  }
  std::mutex mutex_;
  SourceSession session_;
  SyncHealth health_;
  UtcHealth utc_;
  bool connected_ = false;
  rclcpp::Clock steady_{RCL_STEADY_TIME};
  int64_t last_session_ns_ = 0, mission_seen_ = 0;
  unsigned mission_seq_ = 0, mission_system_ = 0, mission_component_ = 0;
  unsigned mission_state_ = 0, mission_mode_ = 0, mission_total_ = 0;
  uint32_t mission_id_ = 0;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<mavros_msgs::msg::TimesyncStatus>::SharedPtr sync_sub_;
};
}
#include "mavros/mavros_plugin_register_macro.hpp"
MAVROS_PLUGIN_REGISTER(lr_mavros_bridge::SourceTimePlugin)
