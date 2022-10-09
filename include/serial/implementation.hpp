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
#include "serial/interface.hpp"
#include "serial/port.hpp"
#include "serial/srv/inject_input.hpp"
#include "serial/srv/inject_output.hpp"
#include "std_msgs/msg/string.hpp"

namespace serial {

class Worker;

class Implementation final : public Interface {
  friend Worker;  // Let the Worker class access 'node_'

 public:
  Implementation(rclcpp::Node *node);
  ~Implementation() {}

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr inspect_output;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr inspect_input;

  virtual void output(const std::string &msg) override;

  // having pure pointers would improve performance here
  // but it would be against the religion of so many
  virtual void register_input_cb(void (*input_cb)(const std::string &msg,
                                                  void *user_data),
                                 void *user_data) override;

  virtual void inject_input(const std::string &msg) override;

 private:
  // node parameters
  std::shared_ptr<PortSettings> port_settings_;

  // topics
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_input_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_output_;

  rclcpp::Service<serial::srv::InjectInput>::SharedPtr inject_input_;
  rclcpp::Service<serial::srv::InjectOutput>::SharedPtr inject_output_;

  void inject_input_handler_(
      const std::shared_ptr<serial::srv::InjectInput::Request> request,
      std::shared_ptr<serial::srv::InjectInput::Response> response);
  void inject_output_handler_(
      const std::shared_ptr<serial::srv::InjectOutput::Request> request,
      std::shared_ptr<serial::srv::InjectOutput::Response> response);

  const rclcpp::Logger get_logger_();

  // port worker
  std::shared_ptr<Worker> worker_;
};

}  // namespace serial

#endif  // OPENVMP_SERIAL_IMPLEMENTATION_H
