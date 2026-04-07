#pragma once

#include <rclcpp/generic_publisher.hpp>
#include <rclcpp/generic_subscription.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialized_message.hpp>

#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <type_traits>
#include <vector>

#include "capra_stamp_demux/unstamper.hpp"

class StampDemuxNode : public rclcpp::Node
{
public:
  explicit StampDemuxNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using StampExtractor = std::function<rclcpp::Time(const rclcpp::SerializedMessage &)>;

  void register_supported_types();
  void on_serialized_msg(const std::shared_ptr<rclcpp::SerializedMessage> msg, const std::string & source_topic);
  rclcpp::QoS make_qos() const;

  template<typename MsgT>
  void add_stamp_extractor(const std::string & type_name)
  {
    stamp_extractors_[type_name] = [](const rclcpp::SerializedMessage & serialized_msg) {
        rclcpp::Serialization<MsgT> ser;
        MsgT typed_msg;
        ser.deserialize_message(&serialized_msg, &typed_msg);

        if constexpr (requires { typed_msg.header.stamp; }) {
          return rclcpp::Time(typed_msg.header.stamp);
        } else if constexpr (requires { typed_msg.stamp; }) {
          return rclcpp::Time(typed_msg.stamp);
        } else {
          static_assert(!std::is_same_v<MsgT, MsgT>, "Message type must have header.stamp or stamp");
        }
      };
  }

  std::string input_type_;
  std::string output_type_;
  std::vector<std::string> input_topics_;
  std::string output_topic_;

  double stale_delay_sec_{1.0};
  bool remove_stamps_{false};

  int qos_depth_{10};
  std::string qos_reliability_{"reliable"};
  std::string qos_durability_{"volatile"};

  std::unordered_map<std::string, StampExtractor> stamp_extractors_;
  Unstamper unstamper_;

  std::vector<rclcpp::GenericSubscription::SharedPtr> subscriptions_;
  rclcpp::GenericPublisher::SharedPtr publisher_;

  bool has_last_forwarded_stamp_{false};
  rclcpp::Time last_forwarded_stamp_{0, 0, RCL_ROS_TIME};
  std::mutex stamp_mutex_;
};
