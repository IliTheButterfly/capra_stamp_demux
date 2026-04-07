#include "capra_stamp_demux/stamp_demux_node.hpp"

#include <capra_control_msgs/msg/bool_stamped.hpp>
#include <capra_control_msgs/msg/flippers.hpp>
#include <capra_control_msgs/msg/flippers_stamped.hpp>
#include <capra_control_msgs/msg/tracks.hpp>
#include <capra_control_msgs/msg/tracks_stamped.hpp>
#include <geometry_msgs/msg/accel.hpp>
#include <geometry_msgs/msg/accel_stamped.hpp>
#include <geometry_msgs/msg/inertia_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/illuminance.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/multi_dof_joint_state.hpp>
#include <sensor_msgs/msg/multi_echo_laser_scan.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <sensor_msgs/msg/relative_humidity.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <sensor_msgs/msg/time_reference.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/header.hpp>

#include <algorithm>
#include <stdexcept>

namespace
{
constexpr const char * kTwistStampedType = "geometry_msgs/msg/TwistStamped";
constexpr const char * kTwistType = "geometry_msgs/msg/Twist";
constexpr const char * kBoolStampedType = "capra_control_msgs/msg/BoolStamped";
constexpr const char * kBoolType = "std_msgs/msg/Bool";
constexpr const char * kFlippersStampedType = "capra_control_msgs/msg/FlippersStamped";
constexpr const char * kFlippersType = "capra_control_msgs/msg/Flippers";
constexpr const char * kTracksStampedType = "capra_control_msgs/msg/TracksStamped";
constexpr const char * kTracksType = "capra_control_msgs/msg/Tracks";

template<typename TracksStampedT>
capra_control_msgs::msg::Tracks extract_tracks_payload(const TracksStampedT & in_msg)
{
  if constexpr (requires { in_msg.tracks; }) {
    return in_msg.tracks;
  } else {
    return in_msg.flippers;
  }
}
}  // namespace

StampDemuxNode::StampDemuxNode(const rclcpp::NodeOptions & options)
: Node("stamp_demux", options)
{
  this->declare_parameter<std::string>("input_type", kTwistStampedType);
  this->declare_parameter<std::string>("output_type", "");
  this->declare_parameter<std::vector<std::string>>("input_topics", std::vector<std::string>{});
  this->declare_parameter<std::string>("output_topic", "");
  this->declare_parameter<double>("stale_delay", 1.0);

  this->declare_parameter<int>("qos_depth", 10);
  this->declare_parameter<std::string>("qos_reliability", "reliable");
  this->declare_parameter<std::string>("qos_durability", "volatile");

  input_type_ = this->get_parameter("input_type").as_string();
  output_type_ = this->get_parameter("output_type").as_string();
  input_topics_ = this->get_parameter("input_topics").as_string_array();
  output_topic_ = this->get_parameter("output_topic").as_string();
  stale_delay_sec_ = this->get_parameter("stale_delay").as_double();

  qos_depth_ = this->get_parameter("qos_depth").as_int();
  qos_reliability_ = this->get_parameter("qos_reliability").as_string();
  qos_durability_ = this->get_parameter("qos_durability").as_string();

  register_supported_types();

  if (output_type_.empty()) {
    output_type_ = input_type_;
  }

  if (input_topics_.empty()) {
    throw std::runtime_error("Parameter 'input_topics' must not be empty");
  }

  if (output_topic_.empty()) {
    throw std::runtime_error("Parameter 'output_topic' must not be empty");
  }

  if (stamp_extractors_.find(input_type_) == stamp_extractors_.end()) {
    throw std::runtime_error("Unsupported input_type: '" + input_type_ + "'");
  }

  remove_stamps_ = (output_type_ != input_type_);
  if (remove_stamps_ && !unstamper_.has_conversion(input_type_, output_type_)) {
    throw std::runtime_error(
            "Unsupported output_type '" + output_type_ + "' for input_type '" + input_type_ + "'");
  }

  const auto qos = make_qos();

  publisher_ = this->create_generic_publisher(output_topic_, output_type_, qos);

  subscriptions_.reserve(input_topics_.size());
  for (const auto & topic : input_topics_) {
    auto callback = [this, topic](const std::shared_ptr<rclcpp::SerializedMessage> msg) {
        on_serialized_msg(msg, topic);
      };

    subscriptions_.push_back(
      this->create_generic_subscription(topic, input_type_, qos, callback));
  }

  RCLCPP_INFO(
    this->get_logger(),
    "Initialized stamp demux with %zu inputs. Input type: %s, output type: %s, stale_delay: %.3fs",
    input_topics_.size(), input_type_.c_str(), output_type_.c_str(), stale_delay_sec_);
}

void StampDemuxNode::register_supported_types()
{
  // capra_control_msgs
  add_stamp_extractor<capra_control_msgs::msg::BoolStamped>(kBoolStampedType);
  add_stamp_extractor<capra_control_msgs::msg::FlippersStamped>(kFlippersStampedType);
  add_stamp_extractor<capra_control_msgs::msg::TracksStamped>(kTracksStampedType);

  unstamper_.add<capra_control_msgs::msg::BoolStamped, std_msgs::msg::Bool>(
    kBoolStampedType, kBoolType,
    [](const capra_control_msgs::msg::BoolStamped & in_msg) {
      std_msgs::msg::Bool out_msg;
      out_msg.data = in_msg.data;
      return out_msg;
    });

  unstamper_.add<capra_control_msgs::msg::FlippersStamped, capra_control_msgs::msg::Flippers>(
    kFlippersStampedType, kFlippersType,
    [](const capra_control_msgs::msg::FlippersStamped & in_msg) {
      return in_msg.flippers;
    });

  unstamper_.add<capra_control_msgs::msg::TracksStamped, capra_control_msgs::msg::Tracks>(
    kTracksStampedType, kTracksType,
    [](const capra_control_msgs::msg::TracksStamped & in_msg) {
      return extract_tracks_payload(in_msg);
    });

  // geometry_msgs stamped wrappers
  add_stamp_extractor<geometry_msgs::msg::AccelStamped>("geometry_msgs/msg/AccelStamped");
  add_stamp_extractor<geometry_msgs::msg::InertiaStamped>("geometry_msgs/msg/InertiaStamped");
  add_stamp_extractor<geometry_msgs::msg::PointStamped>("geometry_msgs/msg/PointStamped");
  add_stamp_extractor<geometry_msgs::msg::PolygonStamped>("geometry_msgs/msg/PolygonStamped");
  add_stamp_extractor<geometry_msgs::msg::PoseStamped>("geometry_msgs/msg/PoseStamped");
  add_stamp_extractor<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "geometry_msgs/msg/PoseWithCovarianceStamped");
  add_stamp_extractor<geometry_msgs::msg::QuaternionStamped>("geometry_msgs/msg/QuaternionStamped");
  add_stamp_extractor<geometry_msgs::msg::TransformStamped>("geometry_msgs/msg/TransformStamped");
  add_stamp_extractor<geometry_msgs::msg::TwistStamped>(kTwistStampedType);
  add_stamp_extractor<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "geometry_msgs/msg/TwistWithCovarianceStamped");
  add_stamp_extractor<geometry_msgs::msg::Vector3Stamped>("geometry_msgs/msg/Vector3Stamped");
  add_stamp_extractor<geometry_msgs::msg::WrenchStamped>("geometry_msgs/msg/WrenchStamped");

  unstamper_.add<geometry_msgs::msg::AccelStamped, geometry_msgs::msg::Accel>(
    "geometry_msgs/msg/AccelStamped", "geometry_msgs/msg/Accel",
    [](const geometry_msgs::msg::AccelStamped & in_msg) {
      return in_msg.accel;
    });
  unstamper_.add<geometry_msgs::msg::InertiaStamped, geometry_msgs::msg::Inertia>(
    "geometry_msgs/msg/InertiaStamped", "geometry_msgs/msg/Inertia",
    [](const geometry_msgs::msg::InertiaStamped & in_msg) {
      return in_msg.inertia;
    });
  unstamper_.add<geometry_msgs::msg::PointStamped, geometry_msgs::msg::Point>(
    "geometry_msgs/msg/PointStamped", "geometry_msgs/msg/Point",
    [](const geometry_msgs::msg::PointStamped & in_msg) {
      return in_msg.point;
    });
  unstamper_.add<geometry_msgs::msg::PolygonStamped, geometry_msgs::msg::Polygon>(
    "geometry_msgs/msg/PolygonStamped", "geometry_msgs/msg/Polygon",
    [](const geometry_msgs::msg::PolygonStamped & in_msg) {
      return in_msg.polygon;
    });
  unstamper_.add<geometry_msgs::msg::PoseStamped, geometry_msgs::msg::Pose>(
    "geometry_msgs/msg/PoseStamped", "geometry_msgs/msg/Pose",
    [](const geometry_msgs::msg::PoseStamped & in_msg) {
      return in_msg.pose;
    });
  unstamper_.add<geometry_msgs::msg::PoseWithCovarianceStamped, geometry_msgs::msg::PoseWithCovariance>(
    "geometry_msgs/msg/PoseWithCovarianceStamped", "geometry_msgs/msg/PoseWithCovariance",
    [](const geometry_msgs::msg::PoseWithCovarianceStamped & in_msg) {
      return in_msg.pose;
    });
  unstamper_.add<geometry_msgs::msg::QuaternionStamped, geometry_msgs::msg::Quaternion>(
    "geometry_msgs/msg/QuaternionStamped", "geometry_msgs/msg/Quaternion",
    [](const geometry_msgs::msg::QuaternionStamped & in_msg) {
      return in_msg.quaternion;
    });
  unstamper_.add<geometry_msgs::msg::TransformStamped, geometry_msgs::msg::Transform>(
    "geometry_msgs/msg/TransformStamped", "geometry_msgs/msg/Transform",
    [](const geometry_msgs::msg::TransformStamped & in_msg) {
      return in_msg.transform;
    });
  unstamper_.add<geometry_msgs::msg::TwistStamped, geometry_msgs::msg::Twist>(
    kTwistStampedType, kTwistType,
    [](const geometry_msgs::msg::TwistStamped & in_msg) {
      return in_msg.twist;
    });
  unstamper_.add<geometry_msgs::msg::TwistWithCovarianceStamped, geometry_msgs::msg::TwistWithCovariance>(
    "geometry_msgs/msg/TwistWithCovarianceStamped", "geometry_msgs/msg/TwistWithCovariance",
    [](const geometry_msgs::msg::TwistWithCovarianceStamped & in_msg) {
      return in_msg.twist;
    });
  unstamper_.add<geometry_msgs::msg::Vector3Stamped, geometry_msgs::msg::Vector3>(
    "geometry_msgs/msg/Vector3Stamped", "geometry_msgs/msg/Vector3",
    [](const geometry_msgs::msg::Vector3Stamped & in_msg) {
      return in_msg.vector;
    });
  unstamper_.add<geometry_msgs::msg::WrenchStamped, geometry_msgs::msg::Wrench>(
    "geometry_msgs/msg/WrenchStamped", "geometry_msgs/msg/Wrench",
    [](const geometry_msgs::msg::WrenchStamped & in_msg) {
      return in_msg.wrench;
    });

  // sensor_msgs (messages with std_msgs/Header)
  add_stamp_extractor<sensor_msgs::msg::BatteryState>("sensor_msgs/msg/BatteryState");
  add_stamp_extractor<sensor_msgs::msg::CameraInfo>("sensor_msgs/msg/CameraInfo");
  add_stamp_extractor<sensor_msgs::msg::CompressedImage>("sensor_msgs/msg/CompressedImage");
  add_stamp_extractor<sensor_msgs::msg::FluidPressure>("sensor_msgs/msg/FluidPressure");
  add_stamp_extractor<sensor_msgs::msg::Illuminance>("sensor_msgs/msg/Illuminance");
  add_stamp_extractor<sensor_msgs::msg::Image>("sensor_msgs/msg/Image");
  add_stamp_extractor<sensor_msgs::msg::Imu>("sensor_msgs/msg/Imu");
  add_stamp_extractor<sensor_msgs::msg::JointState>("sensor_msgs/msg/JointState");
  add_stamp_extractor<sensor_msgs::msg::Joy>("sensor_msgs/msg/Joy");
  add_stamp_extractor<sensor_msgs::msg::LaserScan>("sensor_msgs/msg/LaserScan");
  add_stamp_extractor<sensor_msgs::msg::MagneticField>("sensor_msgs/msg/MagneticField");
  add_stamp_extractor<sensor_msgs::msg::MultiDOFJointState>("sensor_msgs/msg/MultiDOFJointState");
  add_stamp_extractor<sensor_msgs::msg::MultiEchoLaserScan>("sensor_msgs/msg/MultiEchoLaserScan");
  add_stamp_extractor<sensor_msgs::msg::NavSatFix>("sensor_msgs/msg/NavSatFix");
  add_stamp_extractor<sensor_msgs::msg::PointCloud>("sensor_msgs/msg/PointCloud");
  add_stamp_extractor<sensor_msgs::msg::PointCloud2>("sensor_msgs/msg/PointCloud2");
  add_stamp_extractor<sensor_msgs::msg::Range>("sensor_msgs/msg/Range");
  add_stamp_extractor<sensor_msgs::msg::RelativeHumidity>("sensor_msgs/msg/RelativeHumidity");
  add_stamp_extractor<sensor_msgs::msg::Temperature>("sensor_msgs/msg/Temperature");
  add_stamp_extractor<sensor_msgs::msg::TimeReference>("sensor_msgs/msg/TimeReference");
}

rclcpp::QoS StampDemuxNode::make_qos() const
{
  auto depth = std::max(1, qos_depth_);
  rclcpp::QoS qos(static_cast<size_t>(depth));

  if (qos_reliability_ == "best_effort") {
    qos.best_effort();
  } else {
    qos.reliable();
  }

  if (qos_durability_ == "transient_local") {
    qos.transient_local();
  } else {
    qos.durability_volatile();
  }

  return qos;
}

void StampDemuxNode::on_serialized_msg(
  const std::shared_ptr<rclcpp::SerializedMessage> msg,
  const std::string & source_topic)
{
  const auto stamp_it = stamp_extractors_.find(input_type_);
  if (stamp_it == stamp_extractors_.end()) {
    RCLCPP_ERROR(this->get_logger(), "No stamp extractor registered for '%s'", input_type_.c_str());
    return;
  }

  const auto msg_stamp = stamp_it->second(*msg);
  const auto now = this->get_clock()->now();

  if (stale_delay_sec_ >= 0.0 && (now - msg_stamp).seconds() > stale_delay_sec_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Dropping stale message from '%s' (age %.3fs > stale_delay %.3fs)",
      source_topic.c_str(), (now - msg_stamp).seconds(), stale_delay_sec_);
    return;
  }

  {
    std::lock_guard<std::mutex> lock(stamp_mutex_);
    if (has_last_forwarded_stamp_ && !(msg_stamp > last_forwarded_stamp_)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Dropping non-newer message from '%s' (stamp not greater than last forwarded)",
        source_topic.c_str());
      return;
    }

    if (remove_stamps_) {
      rclcpp::SerializedMessage out;
      const bool converted = unstamper_.unstamp(input_type_, output_type_, *msg, out);
      if (!converted) {
        RCLCPP_ERROR_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Failed conversion for %s -> %s", input_type_.c_str(), output_type_.c_str());
        return;
      }
      publisher_->publish(out);
    } else {
      publisher_->publish(*msg);
    }

    last_forwarded_stamp_ = msg_stamp;
    has_last_forwarded_stamp_ = true;
  }
}
