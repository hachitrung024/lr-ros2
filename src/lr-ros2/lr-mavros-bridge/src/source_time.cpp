#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"
#include "std_msgs/msg/string.hpp"
#include "lr_mavros_bridge/live_time.hpp"
#include <mutex>

namespace lr_mavros_bridge {
class SourceTimePlugin : public mavros::plugin::Plugin {
public:
  explicit SourceTimePlugin(mavros::plugin::UASPtr uas) : Plugin(uas, "source_time") {
    rclcpp::PublisherOptions options;
    options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
    publisher_ = node->create_publisher<std_msgs::msg::String>(
      "/lr/source/time_state", rclcpp::QoS(1).reliable().transient_local(), options);
    enable_connection_cb();
    timer_ = node->create_wall_timer(std::chrono::milliseconds(100), [this]() { publish(); });
  }
  Subscriptions get_subscriptions() override { return {make_handler(&SourceTimePlugin::handle)}; }

private:
  void connection_cb(bool connected) override {
    std::lock_guard<std::mutex> guard(mutex_);
    connected_ = connected;
    if (!connected) { uas->set_time_offset(0); last_session_ns_ = 0; }
  }
  void handle(const mavlink::mavlink_message_t *, mavlink::common::msg::TUNNEL &packet,
              mavros::plugin::filter::SystemAndOk) {
    if (packet.payload_type != 32768) return;
    std::lock_guard<std::mutex> guard(mutex_);
    if (session_.accept(packet.payload.data(), packet.payload_length)) {
      last_session_ns_ = steady_.now().nanoseconds();
    }
  }
  void publish() {
    std::lock_guard<std::mutex> guard(mutex_);
    const auto offset = int64_t(uas->get_time_offset());
    const bool calibrated = last_session_ns_ > 0 &&
      steady_.now().nanoseconds() - last_session_ns_ < 3000000000LL;
    Json state = {{"version", 1}, {"connected", connected_},
      {"sync_ready", connected_ && offset > 0}, {"offset_ns", offset},
      {"session_id", calibrated ? session_.id : ""}, {"sequence", session_.sequence},
      {"record_anchor_ns", calibrated ? session_.record_ns : 0},
      {"boot_anchor_ns", calibrated ? session_.boot_ns : 0}};
    std_msgs::msg::String message;
    message.data = state.dump();
    publisher_->publish(message);
  }
  std::mutex mutex_;
  SourceSession session_;
  bool connected_ = false;
  rclcpp::Clock steady_{RCL_STEADY_TIME};
  int64_t last_session_ns_ = 0;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};
}  // namespace lr_mavros_bridge
#include "mavros/mavros_plugin_register_macro.hpp"
MAVROS_PLUGIN_REGISTER(lr_mavros_bridge::SourceTimePlugin)
