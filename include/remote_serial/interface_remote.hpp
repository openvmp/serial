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
#include "remote_serial/interface.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "std_srvs/srv/empty.hpp"

namespace remote_serial {

class RemoteInterface final : public Interface {
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
  void input_handler(const std_msgs::msg::UInt8MultiArray::SharedPtr);

 private:
  std::mutex input_mutex_;
  void (*input_cb_)(const std::string &msg, void *user_data);
  void *input_cb_user_data_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr sub_input_;

  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr
      pub_inject_input_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr
      pub_inject_output_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr clnt_flush_;
};

}  // namespace remote_serial

#endif  // OPENVMP_SERIAL_INTERFACE_REMOTE_H
