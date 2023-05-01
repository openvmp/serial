/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-10-01
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "remote_serial/factory.hpp"

#include <exception>

#include "remote_serial/interface_remote.hpp"
#include "remote_serial/port.hpp"

namespace remote_serial {

std::shared_ptr<Interface> Factory::New(rclcpp::Node *node) {
  rclcpp::Parameter is_remote;
  node->declare_parameter("serial_is_remote", true);
  node->get_parameter("serial_is_remote", is_remote);

  if (is_remote.as_bool()) {
    return std::make_shared<RemoteInterface>(node);
  } else {
    return std::make_shared<Port>(node);
  }
}

}  // namespace remote_serial
