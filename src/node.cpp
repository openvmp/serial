/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-09-24
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "serial/node.hpp"

namespace serial {

Node::Node() : rclcpp::Node::Node("serial") {
  interface_ = std::make_shared<Interface>(this);
}

}  // namespace serial
