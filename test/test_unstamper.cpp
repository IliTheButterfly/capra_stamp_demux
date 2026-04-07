#include "capra_stamp_demux/unstamper.hpp"

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

TEST(UnstamperTest, has_conversion_and_unstamp_twist)
{
  Unstamper unstamper;
  unstamper.add<geometry_msgs::msg::TwistStamped, geometry_msgs::msg::Twist>(
    "geometry_msgs/msg/TwistStamped",
    "geometry_msgs/msg/Twist",
    [](const geometry_msgs::msg::TwistStamped & in_msg) {
      return in_msg.twist;
    });

  EXPECT_TRUE(unstamper.has_conversion("geometry_msgs/msg/TwistStamped", "geometry_msgs/msg/Twist"));
  EXPECT_FALSE(unstamper.has_conversion("geometry_msgs/msg/TwistStamped", "geometry_msgs/msg/Vector3"));

  geometry_msgs::msg::TwistStamped in_msg;
  in_msg.twist.linear.x = 1.23;
  in_msg.twist.angular.z = -0.5;

  rclcpp::Serialization<geometry_msgs::msg::TwistStamped> ser_in;
  rclcpp::Serialization<geometry_msgs::msg::Twist> ser_out;

  rclcpp::SerializedMessage in_serialized;
  ser_in.serialize_message(&in_msg, &in_serialized);

  rclcpp::SerializedMessage out_serialized;
  EXPECT_TRUE(unstamper.unstamp(
    "geometry_msgs/msg/TwistStamped",
    "geometry_msgs/msg/Twist",
    in_serialized,
    out_serialized));

  geometry_msgs::msg::Twist out_msg;
  ser_out.deserialize_message(&out_serialized, &out_msg);

  EXPECT_DOUBLE_EQ(out_msg.linear.x, 1.23);
  EXPECT_DOUBLE_EQ(out_msg.angular.z, -0.5);
}

TEST(UnstamperTest, unstamp_returns_false_for_unknown_conversion)
{
  Unstamper unstamper;
  rclcpp::SerializedMessage in_serialized;
  rclcpp::SerializedMessage out_serialized;

  EXPECT_FALSE(unstamper.unstamp(
    "geometry_msgs/msg/TwistStamped",
    "geometry_msgs/msg/Twist",
    in_serialized,
    out_serialized));
}
