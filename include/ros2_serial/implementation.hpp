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
#include "ros2_serial/interface.hpp"
#include "ros2_serial/srv/inject_input.hpp"
#include "ros2_serial/srv/inject_output.hpp"
#include "std_msgs/msg/string.hpp"

namespace ros2_serial {

class Implementation : public Interface {
 public:
  Implementation(rclcpp::Node *node, const std::string &default_prefix = "");

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr inspect_output;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr inspect_input;

 private:
  // topics
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_input_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_output_;

  rclcpp::Service<srv::InjectInput>::SharedPtr inject_input_;
  rclcpp::Service<srv::InjectOutput>::SharedPtr inject_output_;

  void inject_input_handler_(
      const std::shared_ptr<srv::InjectInput::Request> request,
      std::shared_ptr<srv::InjectInput::Response> response);
  void inject_output_handler_(
      const std::shared_ptr<srv::InjectOutput::Request> request,
      std::shared_ptr<srv::InjectOutput::Response> response);
};

}  // namespace ros2_serial

#endif  // OPENVMP_SERIAL_IMPLEMENTATION_H
