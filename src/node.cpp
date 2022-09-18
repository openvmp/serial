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

#include "serial/node.hpp"

namespace serial {

Node::Node()
    : rclcpp::Node::Node("serial"), port_settings_(new PortSettings()) {
  init_parameters_();

  // Topics are initialized prior to the worker.
  // That is done to put the burden of null checks on the DDS size.
  // Should the burden be on the device driver side,
  // then the conditions for the serial line saturation will be met more often.
  intf_ros = std::shared_ptr<InterfaceRos>(
      new InterfaceRos(this, interface_prefix_.as_string()));

  worker_ = std::shared_ptr<Worker>(new Worker(intf_ros, port_settings_));
  intf_native = worker_;
}

void Node::init_parameters_() {
  // port settings
  this->declare_parameter("dev_name", "/dev/ttyS0");
  this->declare_parameter("baud_rate", 115200);
  this->declare_parameter("data", 1);
  this->declare_parameter("parity", false);
  this->declare_parameter("stop", 1);
  this->declare_parameter("flow_control", false);
  this->declare_parameter("bs", 1024);
  this->get_parameter("dev_name", port_settings_->dev_name);
  this->get_parameter("baud_rate", port_settings_->baud_rate);
  this->get_parameter("data", port_settings_->data);
  this->get_parameter("stop", port_settings_->stop);
  this->get_parameter("parity", port_settings_->parity);
  this->get_parameter("flow_control", port_settings_->flow_control);
  this->get_parameter("bs", port_settings_->bs);

  // dds settings
  this->declare_parameter("interface_prefix", "/serial");
  this->get_parameter("interface_prefix", interface_prefix_);

  RCLCPP_INFO(this->get_logger(), "Serial node initialization complete for %s",
              port_settings_->dev_name.as_string().c_str());
}

}  // namespace serial
