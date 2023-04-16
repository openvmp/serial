/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-09-24
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "ros2_serial/implementation.hpp"

#include "ros2_serial/utils.hpp"
#include "ros2_serial/worker.hpp"

namespace ros2_serial {

Implementation::Implementation(rclcpp::Node *node,
                               const std::string &default_prefix)
    : Interface(node, default_prefix) {
  auto prefix = get_prefix_();

  inspect_input = node->create_publisher<std_msgs::msg::String>(
      prefix + SERIAL_TOPIC_INPUT, 10);
  inspect_output = node->create_publisher<std_msgs::msg::String>(
      prefix + SERIAL_TOPIC_OUTPUT, 10);

  inject_input_ = node->create_service<srv::InjectInput>(
      prefix + SERIAL_SERVICE_INJECT_INPUT,
      std::bind(&Implementation::inject_input_handler_, this,
                std::placeholders::_1, std::placeholders::_2));
  inject_output_ = node->create_service<srv::InjectOutput>(
      prefix + SERIAL_SERVICE_INJECT_OUTPUT,
      std::bind(&Implementation::inject_output_handler_, this,
                std::placeholders::_1, std::placeholders::_2));
}

void Implementation::inject_input_handler_(
    const std::shared_ptr<srv::InjectInput::Request> request,
    std::shared_ptr<srv::InjectInput::Response> response) {
  (void)response;

  RCLCPP_DEBUG(node_->get_logger(), "Incoming request to inject input data: %s",
               utils::bin2hex(request->data).c_str());

  inject_input(request->data);

  auto message = std_msgs::msg::String();
  message.data = request->data;
  inspect_input->publish(message);
}

void Implementation::inject_output_handler_(
    const std::shared_ptr<srv::InjectOutput::Request> request,
    std::shared_ptr<srv::InjectOutput::Response> response) {
  (void)response;

  RCLCPP_DEBUG(node_->get_logger(),
               "Incoming request to inject output data: %s",
               utils::bin2hex(request->data).c_str());

  output(request->data);
}

}  // namespace ros2_serial
