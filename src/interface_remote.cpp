/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-10-01
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "ros2_serial/interface_remote.hpp"

#include <functional>

namespace ros2_serial {

RemoteInterface::RemoteInterface(rclcpp::Node *node)
    : Interface(node), input_cb_{nullptr}, input_cb_user_data_{nullptr} {
  callback_group_ =
      node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  auto prefix = get_prefix_();

  RCLCPP_DEBUG(node_->get_logger(),
               "serial::RemoteInterface::RemoteInterface(): Connecting to the "
               "remote interface: %s",
               prefix.c_str());

  sub_input = node->create_subscription<std_msgs::msg::String>(
      prefix + SERIAL_TOPIC_INPUT, 10,
      std::bind(&RemoteInterface::input_handler, this, std::placeholders::_1));

  clnt_inject_input = node->create_client<srv::InjectInput>(
      prefix + SERIAL_SERVICE_INJECT_INPUT, ::rmw_qos_profile_default,
      callback_group_);
  clnt_inject_output = node->create_client<srv::InjectOutput>(
      prefix + SERIAL_SERVICE_INJECT_OUTPUT, ::rmw_qos_profile_default,
      callback_group_);

  clnt_inject_input->wait_for_service();
  clnt_inject_output->wait_for_service();

  RCLCPP_DEBUG(node_->get_logger(), "Connected to the remote interface: %s",
               prefix.c_str());
}

void RemoteInterface::output(const std::string &data) {
  auto request = std::make_shared<srv::InjectOutput::Request>();
  request->data = data;
  auto f = clnt_inject_output->async_send_request(request);
  f.wait();
}

void RemoteInterface::inject_input(const std::string &data) {
  auto request = std::make_shared<srv::InjectInput::Request>();
  request->data = data;
  auto f = clnt_inject_input->async_send_request(request);
  f.wait();
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
    const std_msgs::msg::String::SharedPtr data) {
  input_mutex_.lock();
  if (input_cb_) input_cb_(data->data, input_cb_user_data_);
  input_mutex_.unlock();
}

}  // namespace ros2_serial
