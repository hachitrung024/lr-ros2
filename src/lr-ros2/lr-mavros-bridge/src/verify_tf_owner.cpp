#include <iomanip>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <set>
#include <sstream>
#include <tf2_msgs/msg/tf_message.hpp>
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("verify_camera_tf_owner");
  std::set<std::string> owners;
  size_t frames = 0;
  auto sub = node->create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf", rclcpp::SensorDataQoS(),
      [&](tf2_msgs::msg::TFMessage::ConstSharedPtr msg,
          const rclcpp::MessageInfo &info) {
        for (auto &t : msg->transforms) {
          if (t.header.frame_id == "map" &&
              t.child_frame_id == "zed_camera_link") {
            std::ostringstream s;
            for (auto b : info.get_rmw_message_info().publisher_gid.data)
              s << std::hex << std::setw(2) << std::setfill('0') << int(b);
            owners.insert(s.str());
            frames++;
          }
        }
      });
  auto end = std::chrono::steady_clock::now() + std::chrono::seconds(8);
  while (rclcpp::ok() && std::chrono::steady_clock::now() < end) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  std::cout << "{\"camera_tf_owner_count\":" << owners.size()
            << ",\"received_frames\":" << frames << ",\"passed\":"
            << ((owners.size() == 1 && frames > 10) ? "true" : "false") << "}"
            << std::endl;
  rclcpp::shutdown();
  return owners.size() == 1 && frames > 10 ? 0 : 1;
}
