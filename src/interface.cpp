/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-10-01
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "ros2_serial/interface.hpp"

#include <functional>

namespace ros2_serial {

Interface::Interface(rclcpp::Node *node) : node_{node} {
  node->declare_parameter("serial_prefix",
                          "/serial/" + std::string(node_->get_name()));
  node->get_parameter("serial_prefix", interface_prefix_);
}

std::string Interface::get_prefix_() {
  std::string prefix = std::string(node_->get_namespace());
  if (prefix.length() > 0 && prefix[prefix.length() - 1] == '/') {
    prefix = prefix.substr(0, prefix.length() - 1);
  }
  prefix += interface_prefix_.as_string();
  return prefix;
}

}  // namespace ros2_serial
