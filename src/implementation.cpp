/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-09-24
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "ros2_serial/implementation.hpp"

#include "ros2_serial/utils.hpp"
#include "ros2_serial/worker.hpp"

namespace ros2_serial {

Implementation::Implementation(rclcpp::Node *node,
                               const std::string &default_prefix)
    : Interface(node, default_prefix) {
  auto prefix = get_prefix_();

  rmw_qos_profile_t rmw = {
      .history = rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_LAST,
      .depth = 1,
      .reliability =
          rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
      .durability = RMW_QOS_POLICY_DURABILITY_VOLATILE,
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

  inject_input_ = node->create_service<srv::InjectInput>(
      prefix + SERIAL_SERVICE_INJECT_INPUT,
      std::bind(&Implementation::inject_input_handler_, this,
                std::placeholders::_1, std::placeholders::_2));
  inject_output_ = node->create_service<srv::InjectOutput>(
      prefix + SERIAL_SERVICE_INJECT_OUTPUT,
      std::bind(&Implementation::inject_output_handler_, this,
                std::placeholders::_1, std::placeholders::_2));
}

void Implementation::inject_input_handler_(
    const std::shared_ptr<srv::InjectInput::Request> request,
    std::shared_ptr<srv::InjectInput::Response> response) {
  (void)response;

  RCLCPP_INFO(this->get_logger_(), "Incoming request to inject input data: %s",
              utils::bin2hex(request->data).c_str());

  inject_input(request->data);

  auto message = std_msgs::msg::String();
  message.data = request->data;
  inspect_input->publish(message);
}

void Implementation::inject_output_handler_(
    const std::shared_ptr<srv::InjectOutput::Request> request,
    std::shared_ptr<srv::InjectOutput::Response> response) {
  (void)response;

  RCLCPP_INFO(this->get_logger_(), "Incoming request to inject output data: %s",
              utils::bin2hex(request->data).c_str());

  output(request->data);
}

const rclcpp::Logger Implementation::get_logger_() {
  return rclcpp::get_logger(get_prefix_());
}

}  // namespace ros2_serial
