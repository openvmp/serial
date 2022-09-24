/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-09-24
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "rclcpp/rclcpp.hpp"
#include "serial/node.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<serial::Node>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
