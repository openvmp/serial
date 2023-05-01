/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-10-01
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_SERIAL_FACTORY_H
#define OPENVMP_SERIAL_FACTORY_H

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "remote_serial/interface.hpp"

namespace remote_serial {

class Factory {
 public:
  static std::shared_ptr<Interface> New(rclcpp::Node *node);
};

}  // namespace remote_serial

#endif  // OPENVMP_SERIAL_FACTORY_H
