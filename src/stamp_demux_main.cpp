#include "capra_stamp_demux/stamp_demux_node.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;

  auto node = std::make_shared<StampDemuxNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
