/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-09-24
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_SERIAL_NODE_H
#define OPENVMP_SERIAL_NODE_H

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "serial/interface.hpp"

namespace serial {

class Node : public rclcpp::Node {
 public:
  Node();

 private:
  std::shared_ptr<Interface> interface_;
};

}  // namespace serial

#endif  // OPENVMP_SERIAL_NODE_H
