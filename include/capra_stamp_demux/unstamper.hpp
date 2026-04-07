#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rclcpp/serialization.hpp>

#include <unordered_map>
#include <functional>
#include <string>

class Unstamper
{
public:
  using ConvertFunc =
    std::function<void(const rclcpp::SerializedMessage&, rclcpp::SerializedMessage&)>;

  void unstamp(
    const std::string & from,
    const std::string & to,
    const rclcpp::SerializedMessage & in,
    rclcpp::SerializedMessage & out) const
  {
    auto key = make_key(from, to);

    auto it = conversions_.find(key);
    if (it == conversions_.end()) {
      RCLCPP_ERROR(
        rclcpp::get_logger("Unstamper"),
        "Unsupported conversion %s -> %s",
        from.c_str(), to.c_str());
      return;
    }

    it->second(in, out);
  }

  template<typename InT, typename OutT>
  void add(
    const std::string & from,
    const std::string & to,
    std::function<OutT(const InT &)> extractor)
  {
    conversions_[make_key(from, to)] =
      [extractor](const rclcpp::SerializedMessage & in,
                  rclcpp::SerializedMessage & out)
      {
        rclcpp::Serialization<InT> ser_in;
        rclcpp::Serialization<OutT> ser_out;

        InT in_msg;
        ser_in.deserialize_message(&in, &in_msg);

        OutT out_msg = extractor(in_msg);

        ser_out.serialize_message(&out_msg, &out);
      };
  }

private:
  std::unordered_map<std::string, ConvertFunc> conversions_;

  static std::string make_key(const std::string & from, const std::string & to)
  {
    return from + "|" + to;
  }
};