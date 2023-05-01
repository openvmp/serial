/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-09-24
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_SERIAL_WORKER_H
#define OPENVMP_SERIAL_WORKER_H

#include <memory>
#include <string>

#include "rclcpp/logger.hpp"
#include "rclcpp/rclcpp.hpp"
#include "remote_serial/interface.hpp"
#include "remote_serial/port.hpp"

namespace remote_serial {

class Worker final {
 public:
  Worker(Port *impl, std::shared_ptr<PortSettings> settings);
  virtual ~Worker();

  void stop();

  /// @brief Place the byte array into the send queue for this serial line
  /// @param msg String object containing the byte array to be written to the
  ///            serial line
  void output(const std::string &msg);
  void register_input_cb(void (*cb)(const std::string &, void *),
                         void *user_data);
  void inject_input(const std::string &msg);

 private:
  // port
  Port *impl_;
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
  const rclcpp::Logger logger_;
};

}  // namespace remote_serial

#endif  // OPENVMP_SERIAL_WORKER_H
