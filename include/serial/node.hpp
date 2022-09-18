/*
 * Copyright 2022 OpenVMP Authors
 *
 * Licensed under HIPPOCRATIC LICENSE Version 3.0.
 * Generated using
 * https://firstdonoharm.dev/version/3/0/bds-bod-cl-eco-ffd-media-mil-soc-sup-sv.md
 * See https://github.com/openvmp/openvmp/blob/main/docs/License.md for more
 * details.
 *
 */

#ifndef OPENVMP_SERIAL_NODE_H
#define OPENVMP_SERIAL_NODE_H

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "serial/interface_ros.hpp"
#include "serial/port.hpp"
#include "serial/worker.hpp"
#include "std_msgs/msg/string.hpp"

namespace serial {

class Node : public rclcpp::Node {
 public:
  Node();

  void init_parameters_();

  std::shared_ptr<InterfaceRos> intf_ros;
  std::shared_ptr<InterfaceNative> intf_native;

 private:
  // node parameters
  std::shared_ptr<PortSettings> port_settings_;
  rclcpp::Parameter topics_prefix_;

  // topics
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_input_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_output_;

  // port worker
  std::shared_ptr<Worker> worker_;
};

}  // namespace serial

#endif  // OPENVMP_SERIAL_NODE_H
