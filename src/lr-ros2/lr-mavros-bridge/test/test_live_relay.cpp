#include <gtest/gtest.h>
#include "../src/net_sensor_relay.cpp"

using namespace lr_mavros_bridge;

TEST(SourceSession, RejectMalformedDuplicateReorderedAndRetired) {
  uint8_t packet[48] = {'L', 'R', 'S', 'T', 1};
  packet[8] = 1;
  auto put = [&](int start, uint64_t value) {
    for (int i = 0; i < 8; ++i) packet[start + i] = uint8_t(value >> (i * 8));
  };
  put(32, 1783700660123456789ULL);
  put(40, 123456789ULL);
  SourceSession session;
  ASSERT_TRUE(session.accept(packet, 48));
  EXPECT_EQ(session.record_ns, 1783700660123456789LL);
  EXPECT_FALSE(session.accept(packet, 48));
  put(24, 2);
  EXPECT_TRUE(session.accept(packet, 48));
  put(24, 1);
  EXPECT_FALSE(session.accept(packet, 48));
  packet[8] = 2;
  put(24, 0);
  put(32, 1783700660000000000ULL);
  EXPECT_TRUE(session.accept(packet, 48));
  packet[8] = 1;
  put(24, 100);
  EXPECT_FALSE(session.accept(packet, 48));
  packet[0] = 'X';
  EXPECT_FALSE(session.accept(packet, 48));
  EXPECT_FALSE(session.accept(packet, 49));
}

TEST(NetRelay, ExactFrameStampCacheAndGenerationGate) {
  rclcpp::init(0, nullptr);
  auto relay = std::make_shared<NetSensorRelay>(rclcpp::NodeOptions());
  auto observer = std::make_shared<rclcpp::Node>("relay_test");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(relay);
  executor.add_node(observer);
  auto qos = rclcpp::QoS(1).reliable().transient_local();
  auto state = observer->create_publisher<std_msgs::msg::String>("/lr/source/time_state", qos);
  auto gate = observer->create_publisher<std_msgs::msg::String>("/lr/source/generation", qos);
  auto image = observer->create_publisher<sensor_msgs::msg::Image>("/zed/zed_node/_raw/rgb/color/rect/image", 3);
  auto cloud = observer->create_publisher<sensor_msgs::msg::PointCloud2>("/zed/zed_node/_raw/point_cloud/cloud_registered", 3);
  int images = 0, clouds = 0;
  int64_t image_stamp = 0, cloud_stamp = 0;
  auto isub = observer->create_subscription<sensor_msgs::msg::Image>("/zed/zed_node/rgb/color/rect/image", 3,
    [&](sensor_msgs::msg::Image::SharedPtr m) { ++images; image_stamp = rclcpp::Time(m->header.stamp).nanoseconds(); EXPECT_EQ(m->data, std::vector<uint8_t>({1, 2, 3})); });
  auto csub = observer->create_subscription<sensor_msgs::msg::PointCloud2>("/zed/zed_node/point_cloud/cloud_registered", 3,
    [&](sensor_msgs::msg::PointCloud2::SharedPtr m) { ++clouds; cloud_stamp = rclcpp::Time(m->header.stamp).nanoseconds(); });
  auto spin = [&]() { for (int i = 0; i < 25; ++i) { executor.spin_some(); std::this_thread::sleep_for(std::chrono::milliseconds(10)); } };
  spin();
  const int64_t raw = 1783700660123456789LL, target = observer->now().nanoseconds();
  std_msgs::msg::String s;
  Json data = {{"sync_ready", true}, {"session_id", "test"}, {"record_anchor_ns", raw},
               {"boot_anchor_ns", 1000000000LL}, {"offset_ns", target - 1000000000LL}};
  s.data = data.dump(); state->publish(s);
  s.data = Json({{"generation", 1}, {"ready", true}}).dump(); gate->publish(s);
  spin();
  sensor_msgs::msg::Image im; im.header.stamp = rclcpp::Time(raw); im.data = {1, 2, 3};
  image->publish(im); spin(); ASSERT_EQ(images, 1); EXPECT_EQ(image_stamp, target);
  data["offset_ns"] = target - 1000000000LL + 1234567;
  s.data = data.dump(); state->publish(s); spin();
  sensor_msgs::msg::PointCloud2 pc; pc.header.stamp = im.header.stamp;
  cloud->publish(pc); spin(); EXPECT_EQ(clouds, 1); EXPECT_EQ(cloud_stamp, image_stamp);
  s.data = Json({{"generation", 2}, {"ready", false}}).dump(); gate->publish(s); spin();
  image->publish(im); cloud->publish(pc); spin(); EXPECT_EQ(images, 1); EXPECT_EQ(clouds, 1);
  data["session_id"] = "";
  s.data = data.dump(); state->publish(s);
  s.data = Json({{"generation", 3}, {"ready", true}}).dump(); gate->publish(s); spin();
  const auto native = observer->now().nanoseconds();
  im.header.stamp = rclcpp::Time(native);
  image->publish(im); spin(); EXPECT_EQ(images, 2); EXPECT_EQ(image_stamp, native);
  image->publish(im); spin(); EXPECT_EQ(images, 2);
  executor.remove_node(relay); executor.remove_node(observer);
  relay.reset(); observer.reset(); rclcpp::shutdown();
}
