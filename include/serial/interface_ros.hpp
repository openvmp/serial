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

#ifndef OPENVMP_SERIAL_INTERFACE_ROS_H
#define OPENVMP_SERIAL_INTERFACE_ROS_H

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "serial/srv/inject_input.hpp"
#include "serial/srv/inject_output.hpp"
#include "std_msgs/msg/string.hpp"

namespace serial {

class Node;

class InterfaceRos {
 public:
  InterfaceRos(Node *node, const std::string &interface_prefix);

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr inspect_input;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr inspect_output;

  rclcpp::Service<serial::srv::InjectInput>::SharedPtr inject_input;
  rclcpp::Service<serial::srv::InjectOutput>::SharedPtr inject_output;

 private:
  Node *node_;

  void inject_input_handler_(
      const std::shared_ptr<serial::srv::InjectInput::Request> request,
      std::shared_ptr<serial::srv::InjectInput::Response> response);
  void inject_output_handler_(
      const std::shared_ptr<serial::srv::InjectOutput::Request> request,
      std::shared_ptr<serial::srv::InjectOutput::Response> response);
};

}  // namespace serial

#endif  // OPENVMP_SERIAL_INTERFACE_ROS_H
