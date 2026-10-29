#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

namespace lr_mavros_bridge {
// Parsing, session checks and ENU conversion live in the independent path node.
// The FCU connection and MAVLink parser remain owned exclusively by MAVROS.
class FuturePathPlugin : public mavros::plugin::Plugin {
public:
  explicit FuturePathPlugin(mavros::plugin::UASPtr uas) : Plugin(uas, "future_path") {
    publisher_ = node->create_publisher<std_msgs::msg::UInt8MultiArray>(
      "/lr/mavlink/path_fragment", rclcpp::QoS(256).reliable());
  }
  Subscriptions get_subscriptions() override { return {make_handler(&FuturePathPlugin::handle)}; }

private:
  void handle(const mavlink::mavlink_message_t *, mavlink::common::msg::TUNNEL &packet,
              mavros::plugin::filter::SystemAndOk) {
    if (packet.payload_type != 32769 || packet.payload_length < 52 ||
        packet.payload_length > packet.payload.size()) return;
    std_msgs::msg::UInt8MultiArray message;
    message.data.assign(packet.payload.begin(), packet.payload.begin() + packet.payload_length);
    publisher_->publish(message);
  }
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr publisher_;
};
}  // namespace lr_mavros_bridge

#include "mavros/mavros_plugin_register_macro.hpp"
MAVROS_PLUGIN_REGISTER(lr_mavros_bridge::FuturePathPlugin)
