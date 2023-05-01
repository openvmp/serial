/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-09-24
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "remote_serial/implementation.hpp"

#include "remote_serial/utils.hpp"
#include "remote_serial/worker.hpp"

namespace remote_serial {

Implementation::Implementation(rclcpp::Node *node,
                               const std::string &default_prefix)
    : Interface(node, default_prefix) {
  callback_group_ =
      node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
}

void Implementation::init_serial_() {
  auto prefix = get_prefix_();
  inspect_input = node_->create_publisher<std_msgs::msg::UInt8MultiArray>(
      prefix + SERIAL_TOPIC_INPUT, 10);
  inspect_output = node_->create_publisher<std_msgs::msg::UInt8MultiArray>(
      prefix + SERIAL_TOPIC_OUTPUT, 10);

  sub_inject_input_ =
      node_->create_subscription<std_msgs::msg::UInt8MultiArray>(
          prefix + SERIAL_TOPIC_INJECT_INPUT, 10,
          std::bind(&Implementation::inject_input_handler_, this,
                    std::placeholders::_1));
  sub_inject_output_ =
      node_->create_subscription<std_msgs::msg::UInt8MultiArray>(
          prefix + SERIAL_TOPIC_INJECT_OUTPUT, 10,
          std::bind(&Implementation::inject_output_handler_, this,
                    std::placeholders::_1));

  srv_flush_ = node_->create_service<std_srvs::srv::Empty>(
      prefix + SERIAL_SERVICE_FLUSH,
      std::bind(&Implementation::flush_handler_, this, std::placeholders::_1,
                std::placeholders::_2),
      ::rmw_qos_profile_default, callback_group_);
}

void Implementation::inject_input_handler_(
    const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
  auto str = std::string(msg->data.begin(), msg->data.end());
  RCLCPP_DEBUG(node_->get_logger(), "Incoming request to inject input data: %s",
               utils::bin2hex(str).c_str());
  inject_input(str);

  auto message = std_msgs::msg::UInt8MultiArray();
  message.data = msg->data;
  inspect_input->publish(message);
}

void Implementation::inject_output_handler_(
    const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
  auto str = std::string(msg->data.begin(), msg->data.end());
  RCLCPP_DEBUG(node_->get_logger(),
               "Incoming request to inject output data: %s",
               utils::bin2hex(str).c_str());
  output(str);
}

void Implementation::flush_handler_(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response) {
  (void)request;
  (void)response;
}

}  // namespace remote_serial
