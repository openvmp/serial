/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-09-24
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "serial/implementation.hpp"

#include "serial/utils.hpp"
#include "serial/worker.hpp"

namespace serial {

Implementation::Implementation(rclcpp::Node *node)
    : Interface(node), port_settings_(new PortSettings()) {
  node_->declare_parameter("serial_dev_name", "/dev/ttyS0");
  node_->declare_parameter("serial_skip_init", false);
  node_->declare_parameter("serial_baud_rate", 115200);
  node_->declare_parameter("serial_data", 8);
  node_->declare_parameter("serial_parity", false);
  node_->declare_parameter("serial_stop", 1);
  node_->declare_parameter("serial_flow_control", true);
  node_->declare_parameter("serial_bs", 1024);
  node_->get_parameter("serial_dev_name", port_settings_->dev_name);
  node_->get_parameter("serial_skip_init", port_settings_->skip_init);
  node_->get_parameter("serial_baud_rate", port_settings_->baud_rate);
  node_->get_parameter("serial_data", port_settings_->data);
  node_->get_parameter("serial_stop", port_settings_->stop);
  node_->get_parameter("serial_parity", port_settings_->parity);
  node_->get_parameter("serial_flow_control", port_settings_->flow_control);
  node_->get_parameter("serial_bs", port_settings_->bs);

  RCLCPP_INFO(node_->get_logger(), "Serial node initialization complete for %s",
              port_settings_->dev_name.as_string().c_str());

  auto prefix = get_prefix_();

  rmw_qos_profile_t rmw = {
      .history = rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_LAST,
      .depth = 1,
      .reliability =
          rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
      .durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
      .deadline = {0, 50000000},
      .lifespan = {0, 50000000},
      .liveliness = RMW_QOS_POLICY_LIVELINESS_AUTOMATIC,
      .liveliness_lease_duration = {0, 0},
      .avoid_ros_namespace_conventions = false,
  };
  auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw), rmw);

  inspect_input = node->create_publisher<std_msgs::msg::String>(
      prefix + SERIAL_TOPIC_INPUT, qos);
  inspect_output = node->create_publisher<std_msgs::msg::String>(
      prefix + SERIAL_TOPIC_OUTPUT, qos);
  ;

  inject_input_ = node->create_service<serial::srv::InjectInput>(
      prefix + SERIAL_SERVICE_INJECT_INPUT,
      std::bind(&Implementation::inject_input_handler_, this,
                std::placeholders::_1, std::placeholders::_2));
  inject_output_ = node->create_service<serial::srv::InjectOutput>(
      prefix + SERIAL_SERVICE_INJECT_OUTPUT,
      std::bind(&Implementation::inject_output_handler_, this,
                std::placeholders::_1, std::placeholders::_2));

  // Topics are initialized prior to the worker.
  // That is done to put the burden of null checks on the DDS size.
  // Should the burden be on the device driver side,
  // then the conditions for the serial line saturation will be met more often.
  worker_ = std::make_shared<Worker>(this, port_settings_);
}

void Implementation::inject_input_handler_(
    const std::shared_ptr<serial::srv::InjectInput::Request> request,
    std::shared_ptr<serial::srv::InjectInput::Response> response) {
  (void)response;

  RCLCPP_INFO(this->get_logger_(), "Incoming request to inject input data: %s",
              utils::bin2hex(request->data).c_str());

  worker_->inject_input(request->data);

  auto message = std_msgs::msg::String();
  message.data = request->data;
  inspect_input->publish(message);
}

void Implementation::inject_output_handler_(
    const std::shared_ptr<serial::srv::InjectOutput::Request> request,
    std::shared_ptr<serial::srv::InjectOutput::Response> response) {
  (void)response;

  RCLCPP_INFO(this->get_logger_(), "Incoming request to inject output data: %s",
              utils::bin2hex(request->data).c_str());

  worker_->output(request->data);
}

void Implementation::output(const std::string &msg) { worker_->output(msg); }

// having pure pointers would improve performance here
// but it would be against the religion of so many
void Implementation::register_input_cb(void (*input_cb)(const std::string &msg,
                                                        void *user_data),
                                       void *user_data) {
  worker_->register_input_cb(input_cb, user_data);
}

void Implementation::inject_input(const std::string &msg) {
  worker_->inject_input(msg);
}

const rclcpp::Logger Implementation::get_logger_() {
  return rclcpp::get_logger(get_prefix_());
}

}  // namespace serial
