#include "LIVMapper.h"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::Node::SharedPtr nh;
  LIVMapper mapper(nh, "impact_livo_replay");
  // The community mapper constructs its node internally; use that same node.
  nh = mapper.node;
  image_transport::ImageTransport it_(nh);
  mapper.initializeSubscribersAndPublishers(nh, it_);
  mapper.run(nh);
  rclcpp::shutdown();
  return 0;
}
