#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"
#include "std_msgs/msg/u_int64.hpp"

namespace lr_mavros_bridge {
class ReplayTimePlugin : public mavros::plugin::Plugin {
public:
  explicit ReplayTimePlugin(mavros::plugin::UASPtr uas)
      : Plugin(uas, "replay_time") {
    rclcpp::PublisherOptions options;
    options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
    publisher_ = node->create_publisher<std_msgs::msg::UInt64>(
        "~/epoch_us", rclcpp::QoS(1).reliable().transient_local(), options);
    uas->set_timesync_mode(mavros::uas::timesync_mode::PASSTHROUGH);
    enable_connection_cb();
  }
  Subscriptions get_subscriptions() override {
    return {make_handler(&ReplayTimePlugin::handle_time)};
  }

private:
  void connection_cb(bool connected) override {
    if (!connected) {
      uas->set_time_offset(0);
      publisher_->publish(std_msgs::msg::UInt64());
    }
  }
  void handle_time(const mavlink::mavlink_message_t *,
                   mavlink::common::msg::SYSTEM_TIME &time,
                   mavros::plugin::filter::SystemAndOk) {
    const uint64_t boot_us = static_cast<uint64_t>(time.time_boot_ms) * 1000ULL;
    // Replay source rounds both fields together. Reject malformed epochs.
    if (time.time_unix_usec < 1000000000000000ULL ||
        time.time_unix_usec > 2100000000000000ULL ||
        time.time_unix_usec < boot_us) {
      return;
    }
    const uint64_t epoch = time.time_unix_usec - boot_us;
    uas->set_time_offset(epoch * 1000ULL);
    std_msgs::msg::UInt64 ready;
    ready.data = epoch;
    publisher_->publish(ready);
  }
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr publisher_;
};
} // namespace lr_mavros_bridge
#include "mavros/mavros_plugin_register_macro.hpp"
MAVROS_PLUGIN_REGISTER(lr_mavros_bridge::ReplayTimePlugin)
