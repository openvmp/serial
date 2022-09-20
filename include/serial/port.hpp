/*
 * Copyright 2022 OpenVMP Authors
 *
 * Licensed under HIPPOCRATIC LICENSE Version 3.0.
 * Generated using
 * https://firstdonoharm.dev/version/3/0/bds-bod-cl-eco-ffd-media-mil-soc-sup-sv.md
 * See https://github.com/openvmp/openvmp/blob/main/docs/License.md for more
 * details.
 *
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

  static const int BUFFER_SIZE_MAX = 1048576;  // 1MB
  rclcpp::Parameter bs;

  int setup();
};

}  // namespace serial

#endif  // OPENVMP_SERIAL_PORT_H