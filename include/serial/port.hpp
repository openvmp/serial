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

#include <string>

#include "rclcpp/rclcpp.hpp"

namespace serial {

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

}  // namespace serial

#endif  // OPENVMP_SERIAL_PORT_H