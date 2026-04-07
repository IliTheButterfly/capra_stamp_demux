#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

#include <functional>
#include <string>
#include <unordered_map>

class Unstamper
{
public:
  using ConvertFunc =
    std::function<void(const rclcpp::SerializedMessage &, rclcpp::SerializedMessage &)>;

  bool has_conversion(const std::string & from, const std::string & to) const
  {
    return conversions_.find(make_key(from, to)) != conversions_.end();
  }

  bool unstamp(
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
      return false;
    }

    it->second(in, out);
    return true;
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
