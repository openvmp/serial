/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-10-01
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_SERIAL_INTERFACE_REMOTE_H
#define OPENVMP_SERIAL_INTERFACE_REMOTE_H

#include <future>
#include <memory>
#include <string>

#include "rclcpp/callback_group.hpp"
#include "rclcpp/rclcpp.hpp"
#include "serial/interface.hpp"
#include "serial/srv/inject_input.hpp"
#include "serial/srv/inject_output.hpp"
#include "std_msgs/msg/string.hpp"

namespace serial {

class RemoteInterface : public Interface {
 public:
  RemoteInterface(rclcpp::Node *node);
  virtual ~RemoteInterface() {}

  virtual void output(const std::string &) override;
  virtual void inject_input(const std::string &) override;

  // having pure pointers would improve performance here
  // but it would be against the religion of so many
  virtual void register_input_cb(void (*)(const std::string &msg,
                                          void *user_data),
                                 void *user_data) override;

 protected:
  void input_handler(const std_msgs::msg::String::SharedPtr data);
  // void output_handler(const std_msgs::msg::String &data);

 private:
  std::mutex input_mutex_;
  void (*input_cb_)(const std::string &msg, void *user_data);
  void *input_cb_user_data_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_input;

  rclcpp::Client<serial::srv::InjectInput>::SharedPtr clnt_inject_input;
  rclcpp::Client<serial::srv::InjectOutput>::SharedPtr clnt_inject_output;
};

}  // namespace serial

#endif  // OPENVMP_SERIAL_INTERFACE_REMOTE_H
