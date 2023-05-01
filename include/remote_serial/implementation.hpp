/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-09-24
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_SERIAL_IMPLEMENTATION_H
#define OPENVMP_SERIAL_IMPLEMENTATION_H

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "remote_serial/interface.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "std_srvs/srv/empty.hpp"

namespace remote_serial {

class Implementation : public Interface {
 public:
  Implementation(rclcpp::Node *node, const std::string &default_prefix = "");

  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr inspect_output;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr inspect_input;

 protected:
  // init_serial is called by child constructors or other child members
  // when all the downstream modules are initialized and this modules is
  // ready to take requests from the upstreams.
  void init_serial_();

 private:
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  // topics
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr publisher_input_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr
      publisher_output_;

  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr
      sub_inject_input_;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr
      sub_inject_output_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_flush_;

  void flush_handler_(
      const std::shared_ptr<std_srvs::srv::Empty::Request> request,
      std::shared_ptr<std_srvs::srv::Empty::Response> response);
  void inject_input_handler_(const std_msgs::msg::UInt8MultiArray::SharedPtr);
  void inject_output_handler_(const std_msgs::msg::UInt8MultiArray::SharedPtr);
};

}  // namespace remote_serial

#endif  // OPENVMP_SERIAL_IMPLEMENTATION_H
