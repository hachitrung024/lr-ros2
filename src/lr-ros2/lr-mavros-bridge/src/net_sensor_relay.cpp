#include "lr_mavros_bridge/live_time.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/string.hpp"
#include <map>
#include <mutex>
#include <functional>

namespace lr_mavros_bridge {
class NetSensorRelay : public rclcpp::Node {
public:
  explicit NetSensorRelay(const rclcpp::NodeOptions &options) : Node("net_sensor_relay", options) {
    const auto camera = declare_parameter("camera_name", "zed");
    const auto base = "/" + camera + "/zed_node/";
    frame_ = create_publisher<std_msgs::msg::Header>("/lr/net/frame_stamp", 10);
    observed_ = create_publisher<std_msgs::msg::Header>("/lr/net/raw_frame", 10);
    const auto qos = rclcpp::QoS(1).reliable().transient_local();
    fusion_sub_ = create_subscription<std_msgs::msg::String>("/lr/source/generation", qos,
      [this](std_msgs::msg::String::ConstSharedPtr m) {
        try {
          const auto state = Json::parse(m->data);
          std::lock_guard<std::mutex> guard(mutex_);
          fusion_enabled_ = state.at("ready").get<bool>();
          fusion_start_ = state.at("start_ns").get<int64_t>();
          fusion_seen_ = steady_.now().nanoseconds();
        } catch (const std::exception &) { }
      });
    gate_sub_ = create_subscription<std_msgs::msg::String>("/lr/source/camera_generation", qos,
      [this](std_msgs::msg::String::ConstSharedPtr m) {
        try {
          const auto state = Json::parse(m->data);
          std::lock_guard<std::mutex> guard(mutex_);
          const auto generation = state.at("generation").get<uint64_t>();
          const auto runtime = state.value("runtime_id", std::string());
          if (retired_runtimes_.count(runtime) ||
              (runtime == runtime_id_ && generation < generation_)) return;
          if (runtime != runtime_id_ && !runtime_id_.empty()) retired_runtimes_.insert(runtime_id_);
          if (generation != generation_ || runtime != runtime_id_) {
            cache_.clear(); last_stamps_.clear(); last_frame_ = 0;
            generation_ = generation; runtime_id_ = runtime;
          }
          enabled_ = state.at("ready").get<bool>();
          state_ = state; state_seen_ = steady_.now().nanoseconds();
        } catch (const std::exception &) { }
      });
    add<sensor_msgs::msg::Image>(base, "rgb/color/rect/image", true);
    add<sensor_msgs::msg::Image>(base, "depth/depth_registered");
    add<sensor_msgs::msg::CameraInfo>(base, "rgb/color/rect/camera_info");
    add<sensor_msgs::msg::CameraInfo>(base, "depth/camera_info");
    add<sensor_msgs::msg::PointCloud2>(base, "point_cloud/cloud_registered");
    add<sensor_msgs::msg::Imu>(base, "imu/data", false, true);
    refresh_timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {
      for (auto &refresh : refresh_) refresh();
    });
  }

private:
  template<class Message> void add(const std::string &base, const std::string &suffix,
                                  bool rgb = false, bool lazy = false) {
    auto pub = create_publisher<Message>(base + suffix, rclcpp::QoS(3).reliable());
    pubs_.push_back(pub);
    auto callback = [this, pub, rgb, suffix](typename Message::UniquePtr message) {
        std::lock_guard<std::mutex> guard(mutex_);
        if (rgb) observed_->publish(message->header);
        if (!enabled_ || steady_.now().nanoseconds() - state_seen_ > 2000000000LL) return;
        const auto raw = rclcpp::Time(message->header.stamp).nanoseconds();
        auto cached = cache_.find(raw);
        int64_t mapped;
        if (cached != cache_.end()) mapped = cached->second;
        else {
          if (raw < state_.value("raw_floor_ns", int64_t(0)) ||
              raw < state_.value("record_anchor_ns", int64_t(0))) return;
          const auto offset = state_.value("stamp_offset_ns", int64_t(0));
          if ((offset > 0 && raw > INT64_MAX - offset) ||
              (offset < 0 && raw < -offset)) return;
          mapped = raw + offset;
          cache_[raw] = mapped;
          while (cache_.size() > 256) cache_.erase(cache_.begin());
        }
        const bool system_time = state_.value("stamp_is_system", false);
        const auto reference = system_time ? now().nanoseconds() :
          state_.value("latest_stamp_ns", mapped);
        const auto age = reference - mapped;
        if (mapped <= 0 || age < -500000000LL || age > 2000000000LL ||
            mapped <= last_stamps_[suffix]) return;
        last_stamps_[suffix] = mapped;
        if (rgb && mapped <= last_frame_) return;
        message->header.stamp = rclcpp::Time(mapped);
        if (rgb) {
          last_frame_ = mapped;
          if (system_time && fusion_enabled_ && mapped >= fusion_start_ &&
              steady_.now().nanoseconds() - fusion_seen_ < 2000000000LL)
            frame_->publish(message->header);
        }
        pub->publish(std::move(message));
      };
    auto create = [this, base, suffix, callback]() {
      return create_subscription<Message>(base + "_raw/" + suffix,
        rclcpp::SensorDataQoS().keep_last(3), callback);
    };
    if (lazy) {
      auto slot = std::make_shared<rclcpp::SubscriptionBase::SharedPtr>();
      refresh_.push_back([pub, slot, create]() {
        if (pub->get_subscription_count() > 0) { if (!*slot) *slot = create(); }
        else slot->reset();
      });
    } else subs_.push_back(create());
  }
  std::mutex mutex_;
  Json state_ = Json::object();
  rclcpp::Clock steady_{RCL_STEADY_TIME};
  int64_t state_seen_ = 0, last_frame_ = 0;
  uint64_t generation_ = 0;
  std::string runtime_id_;
  std::unordered_set<std::string> retired_runtimes_;
  bool enabled_ = false, fusion_enabled_ = false;
  int64_t fusion_start_ = 0, fusion_seen_ = 0;
  std::map<int64_t, int64_t> cache_;
  std::map<std::string, int64_t> last_stamps_;
  std::vector<rclcpp::PublisherBase::SharedPtr> pubs_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;
  std::vector<std::function<void()>> refresh_;
  rclcpp::TimerBase::SharedPtr refresh_timer_;
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr frame_, observed_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr fusion_sub_, gate_sub_;
};
}  // namespace lr_mavros_bridge
RCLCPP_COMPONENTS_REGISTER_NODE(lr_mavros_bridge::NetSensorRelay)
