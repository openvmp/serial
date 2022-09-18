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

#define _BSD_SOURCE
#include "serial/interface_ros.hpp"

#include <functional>

#include "serial/node.hpp"
#include "serial/utils.hpp"

namespace serial {

void InterfaceRos::inject_input_handler_(
    const std::shared_ptr<serial::srv::InjectInput::Request> request,
    std::shared_ptr<serial::srv::InjectInput::Response> response) {
  (void)response;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
              "Incoming request to inject input\ndata: %s",
              utils::bin2hex(request->data).c_str());

  node_->intf_native->inject_read(request->data);

  auto message = std_msgs::msg::String();
  message.data = request->data;
  inspect_input->publish(message);
}

void InterfaceRos::inject_output_handler_(
    const std::shared_ptr<serial::srv::InjectOutput::Request> request,
    std::shared_ptr<serial::srv::InjectOutput::Response> response) {
  (void)response;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
              "Incoming request to inject output\ndata: %s",
              utils::bin2hex(request->data).c_str());

  node_->intf_native->write(request->data);

  auto message = std_msgs::msg::String();
  message.data = request->data;
  inspect_output->publish(message);
}

InterfaceRos::InterfaceRos(Node *node, const std::string &topics_prefix)
    : node_{node} {
  inspect_input = node->create_publisher<std_msgs::msg::String>(
      topics_prefix + "/inspect/input", 10);
  inspect_output = node->create_publisher<std_msgs::msg::String>(
      topics_prefix + "/inspect/output", 10);

  inject_input = node->create_service<serial::srv::InjectInput>(
      "inject/input", std::bind(&InterfaceRos::inject_input_handler_, this,
                                std::placeholders::_1, std::placeholders::_2));
  inject_output = node->create_service<serial::srv::InjectOutput>(
      "inject/output", std::bind(&InterfaceRos::inject_output_handler_, this,
                                 std::placeholders::_1, std::placeholders::_2));
}

}  // namespace serial
