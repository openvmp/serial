/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-09-24
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_SERIAL_PORT_H
#define OPENVMP_SERIAL_PORT_H

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "remote_serial/implementation.hpp"

namespace remote_serial {

class PortSettings {
 public:
  rclcpp::Parameter dev_name;
  rclcpp::Parameter skip_init;
  rclcpp::Parameter baud_rate;
  rclcpp::Parameter data;
  rclcpp::Parameter parity;
  rclcpp::Parameter stop;
  rclcpp::Parameter flow_control;
  rclcpp::Parameter sw_flow_control;

  static const int BUFFER_SIZE_MAX = 1048576;  // 1MB
  rclcpp::Parameter bs;

  int setup(int old_fd);
};

class Worker;

class Port final : public Implementation {
  friend Worker;  // Let the Worker class access 'node_'

 public:
  Port(rclcpp::Node *node);

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

  // port worker
  std::shared_ptr<Worker> worker_;
};

}  // namespace remote_serial

#endif  // OPENVMP_SERIAL_PORT_H
