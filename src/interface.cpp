/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-10-01
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "remote_serial/interface.hpp"

#include <functional>

namespace remote_serial {

Interface::Interface(rclcpp::Node *node, const std::string &default_prefix)
    : node_{node} {
  auto prefix = default_prefix;
  if (prefix == "") {
    prefix = "/serial/" + std::string(node_->get_name());
  }

  int index = 0;
  std::string parameter_name;
  do {
    parameter_name = "serial_prefix";
    if (index++ != 0) {
      parameter_name += "_" + std::to_string(index);
    }
  } while (node->has_parameter(parameter_name));

  node->declare_parameter(parameter_name, prefix);
  node->get_parameter(parameter_name, interface_prefix_);
}

std::string Interface::get_prefix_() {
  std::string prefix = std::string(node_->get_namespace());
  if (prefix.length() > 0 && prefix[prefix.length() - 1] == '/') {
    prefix = prefix.substr(0, prefix.length() - 1);
  }
  prefix += interface_prefix_.as_string();
  return prefix;
}

}  // namespace remote_serial
