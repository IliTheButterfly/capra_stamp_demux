#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/generic_subscription.hpp>
#include <rclcpp/generic_publisher.hpp>
#include <rclcpp/serialized_message.hpp>

#include <vector>
#include <string>
#include <sstream>
#include <stdexcept>

class ParamsHelperException : public std::runtime_error
{
public:
  explicit ParamsHelperException(const std::string & what)
  : std::runtime_error(what) {}
};

template<class T>
void fetch_param(std::shared_ptr<rclcpp::Node> nh,
                 const std::string & param_name,
                 T & output)
{
  rclcpp::Parameter param;
  if (!nh->get_parameter(param_name, param)) {
    std::ostringstream err;
    err << "Missing param '" << param_name
        << "' (ns: " << nh->get_namespace() << ")";
    throw ParamsHelperException(err.str());
  }
  output = param.get_value<T>();
}

struct Config
{
    std::vector<std::string> input_topics;
    std::string type;
    bool remove_stamps = false;
    float timeout = 1.f;
};

class StampDemuxNode : public rclcpp::Node
{
public:

};