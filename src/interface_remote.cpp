/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-10-01
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "remote_serial/interface_remote.hpp"

#include <functional>

namespace remote_serial {

RemoteInterface::RemoteInterface(rclcpp::Node *node)
    : Interface(node), input_cb_{nullptr}, input_cb_user_data_{nullptr} {
  callback_group_ =
      node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  auto prefix = get_prefix_();

  RCLCPP_DEBUG(node_->get_logger(),
               "serial::RemoteInterface::RemoteInterface(): Connecting to the "
               "remote interface: %s",
               prefix.c_str());

  sub_input_ = node->create_subscription<std_msgs::msg::UInt8MultiArray>(
      prefix + SERIAL_TOPIC_INPUT, 10,
      std::bind(&RemoteInterface::input_handler, this, std::placeholders::_1));

  pub_inject_input_ = node->create_publisher<std_msgs::msg::UInt8MultiArray>(
      prefix + SERIAL_TOPIC_INJECT_INPUT, 10);
  pub_inject_output_ = node->create_publisher<std_msgs::msg::UInt8MultiArray>(
      prefix + SERIAL_TOPIC_INJECT_OUTPUT, 10);
  clnt_flush_ = node->create_client<std_srvs::srv::Empty>(
      prefix + SERIAL_SERVICE_FLUSH, ::rmw_qos_profile_default,
      callback_group_);

  clnt_flush_->wait_for_service();

  RCLCPP_DEBUG(node_->get_logger(), "Connected to the remote interface: %s",
               prefix.c_str());
}

void RemoteInterface::output(const std::string &data) {
  auto msg = std_msgs::msg::UInt8MultiArray();
  msg.data = std::vector<uint8_t>(data.begin(), data.end());
  pub_inject_output_->publish(msg);
}

void RemoteInterface::inject_input(const std::string &data) {
  auto msg = std_msgs::msg::UInt8MultiArray();
  msg.data = std::vector<uint8_t>(data.begin(), data.end());
  pub_inject_input_->publish(msg);
}

// having pure pointers would improve performance here
// but it would be against the religion of so many
void RemoteInterface::register_input_cb(void (*input_cb)(const std::string &msg,
                                                         void *user_data),
                                        void *user_data) {
  input_mutex_.lock();
  input_cb_ = input_cb;
  input_cb_user_data_ = user_data;
  input_mutex_.unlock();
}

void RemoteInterface::input_handler(
    const std_msgs::msg::UInt8MultiArray::SharedPtr request) {
  input_mutex_.lock();
  auto str = std::string(request->data.begin(), request->data.end());
  if (input_cb_) input_cb_(str, input_cb_user_data_);
  input_mutex_.unlock();
}

}  // namespace remote_serial
