/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-10-01
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "serial/interface.hpp"

#include <functional>

namespace serial {

Interface::Interface(rclcpp::Node *node) : node_{node} {
  node->declare_parameter("serial_prefix", "/serial/com1");
  node->get_parameter("serial_prefix", interface_prefix_);
}

}  // namespace serial
