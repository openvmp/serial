/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-09-24
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_SERIAL_INTERFACE_H
#define OPENVMP_SERIAL_INTERFACE_H

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#define SERIAL_TOPIC_INPUT "/inspect/input"
#define SERIAL_TOPIC_OUTPUT "/inspect/output"
#define SERIAL_TOPIC_INJECT_INPUT "/inject/input"
#define SERIAL_TOPIC_INJECT_OUTPUT "/inject/output"
#define SERIAL_SERVICE_FLUSH "/flush"

namespace remote_serial {

class Interface {
 public:
  Interface(rclcpp::Node *node, const std::string &default_prefix = "");
  virtual ~Interface() {}

  // output writes bytes to the serial line
  virtual void output(const std::string &) = 0;

  // register_input_cb is used to set the callback function which
  // is called every time data is received from the serial line
  virtual void register_input_cb(void (*)(const std::string &msg,
                                          void *user_data),
                                 void *user_data) = 0;

  virtual void inject_input(const std::string &) = 0;

 protected:
  rclcpp::Node *node_;

  std::string get_prefix_();

 private:
  rclcpp::Parameter interface_prefix_;
};

}  // namespace remote_serial

#endif  // OPENVMP_SERIAL_INTERFACE_H
