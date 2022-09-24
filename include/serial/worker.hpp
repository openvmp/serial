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

#ifndef OPENVMP_SERIAL_WORKER_H
#define OPENVMP_SERIAL_WORKER_H

#include <memory>
#include <string>

#include "rclcpp/logger.hpp"
#include "rclcpp/rclcpp.hpp"
#include "serial/interface.hpp"
#include "serial/interface_native.hpp"
#include "serial/port.hpp"

namespace serial {

class Worker final : public InterfaceNative {
 public:
  Worker(Interface *intf, std::shared_ptr<PortSettings> settings);
  virtual ~Worker();

  void stop();

  /// @brief Place the byte array into the send queue for this serial line
  /// @param msg String object containing the byte array to be written to the
  ///            serial line
  void output(const std::string &msg) override;
  void register_input_cb(void (*cb)(const std::string &, void *),
                         void *user_data) override;
  void inject_input(const std::string &msg) override;

 private:
  // port
  Interface *intf_;
  std::shared_ptr<PortSettings> settings_;
  int fd_;

  // thread
  std::shared_ptr<std::thread> thread_;
  int signal_[2];
  volatile bool do_stop_;
  void run_();

  // queues
  std::vector<std::pair<std::string,  // queue entry
                        int           // read position in the entry
                        >>
      output_queue_;
  std::mutex output_queue_mutex_;

  // callbacks
  void (*volatile input_cb_)(const std::string &msg, void *user_data);
  void *volatile input_cb_user_data_;
  std::mutex input_cb_mutex_;

  // misc
  const rclcpp::Logger get_logger_();
};

}  // namespace serial

#endif  // OPENVMP_SERIAL_WORKER_H