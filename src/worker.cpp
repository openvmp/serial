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

#include "serial/worker.hpp"

#include "serial/utils.hpp"

#define _BSD_SOURCE
#include <stdio.h>
#include <unistd.h>

namespace serial {

Worker::Worker(std::shared_ptr<InterfaceRos> intf_ros,
               std::shared_ptr<PortSettings> settings)
    : intf_ros_(intf_ros),
      settings_(settings),
      fd_(-1),
      signal_{-1, -1},
      do_stop_(false),
      read_cb_{nullptr},
      read_cb_user_data_{nullptr} {
  fd_ = settings->setup();
  if (fd_ < 0) {
    throw std::invalid_argument("failed to conigure the port");
  }

  if (::pipe(signal_) < 0) {
    ::close(fd_);
    fd_ = -1;
    throw std::invalid_argument("failed to creat a signalling channel");
  }
  thread_ = std::shared_ptr<std::thread>(new std::thread(&Worker::run_, this));

  RCLCPP_INFO(this->get_logger_(), "Serial node initialization complete for %s",
              settings_->dev_name.as_string().c_str());
}

Worker::~Worker() { stop(); }

void Worker::write(const std::string &msg) {
  std::lock_guard<std::mutex> guard(send_queue_mutex_);
  send_queue_.push_back(std::pair<std::string, int>(msg, 0));
}

void Worker::register_read_cb(void (*cb)(const std::string &, void *),
                              void *user_data) {
  std::lock_guard<std::mutex> guard(read_cb_mutex_);
  read_cb_ = cb;
  read_cb_user_data_ = user_data;
}

void Worker::inject_read(const std::string &msg) {
  std::lock_guard<std::mutex> guard(read_cb_mutex_);
  read_cb_(msg, read_cb_user_data_);
}

void Worker::stop() {
  if (do_stop_) {
    // already stopped
    return;
  }

  do_stop_ = true;
  ::write(signal_[1], " ", 1);
  thread_->join();
}

void Worker::run_() {
  fd_set read_fds, write_fds, except_fds;
  int max_fds = fd_;
  if (signal_[0] > max_fds) {
    max_fds = signal_[0];
  }

  while (true) {
    ::memset(&read_fds, 0, sizeof(read_fds));
    ::memset(&write_fds, 0, sizeof(write_fds));
    ::memset(&except_fds, 0, sizeof(except_fds));

    FD_SET(fd_, &read_fds);
    FD_SET(signal_[0], &read_fds);
    if (send_queue_.size() > 0) {
      FD_SET(fd_, &write_fds);
    }

    if (::select(max_fds, &read_fds, &write_fds, &except_fds, nullptr) < 0) {
      RCLCPP_ERROR(get_logger_(), "select() failed for %s",
                   settings_->dev_name.as_string().c_str());
      break;
    }

    if (FD_ISSET(signal_[0], &read_fds)) {
      if (do_stop_) {
        RCLCPP_INFO(get_logger_(), "exiting IO loop gracefully");
        break;
      }

      send_queue_mutex_.lock();
      while (send_queue_.size() > 0) {
        auto next = send_queue_.begin();
        send_queue_mutex_.unlock();
        // We do not need to keep the lock during IO,
        // since the other threads use .push_back() and,
        // thus, do not invalidate this iterator.

        // Now attempt to write
        const char *read_pos = next->first.data() + next->second;
        const int remains_to_write = next->first.length() - next->second;
        int wrote = ::write(fd_, read_pos, remains_to_write);
        if (wrote < 0) {
          // TODO(clairbee): check the errno in this thread
          // TODO(clairbee): if not EAGAIN
          // TODO(clairbee):   break the IO loop and re-open the file
          RCLCPP_INFO(get_logger_(), "error while writing data: %d bytes",
                      remains_to_write);
          continue;  // hoping it was EAGAIN
        }
        if (wrote == 0) {
          // TODO(clairbee): break the IO loop and re-open the file
          RCLCPP_ERROR(get_logger_(), "EOF while writing data: %d bytes",
                       remains_to_write);
          break;
        }

        // Now publish what is written successfully
        auto message = std_msgs::msg::String();
        message.data = std::string(read_pos, wrote);
        RCLCPP_INFO(get_logger_(), "Publishing written data: '%s'",
                    utils::bin2hex(message.data).c_str());
        intf_ros_->inspect_output->publish(message);

        // Advance the queue read position
        if (wrote == remains_to_write) {
          send_queue_mutex_.lock();
          send_queue_.erase(next);
        } else {
          next->second += wrote;
        }

        send_queue_mutex_.lock();
      }
      send_queue_mutex_.unlock();
    }

    if (FD_ISSET(fd_, &except_fds)) {
      RCLCPP_INFO(get_logger_(),
                  "exiting IO loop due to an exception while reading");
      break;
    }

    if (FD_ISSET(fd_, &read_fds)) {
      int buffer_size = settings_->bs.as_int();
      if (buffer_size > PortSettings::BUFFER_SIZE_MAX) {
        buffer_size = PortSettings::BUFFER_SIZE_MAX;
      }
      auto buffer = std::unique_ptr<char[]>(new char[buffer_size]);

      int total = ::read(fd_, buffer.get(), buffer_size);
      if (total < 0) {
        // TODO(clairbee): break the IO loop and re-open the file
        RCLCPP_INFO(get_logger_(), "error while reading data");
        continue;  // hoping it was EAGAIN
      }

      if (total == 0) {
        RCLCPP_INFO(get_logger_(), "exiting IO loop gdue to EOF");
        break;
      }

      if (total > 0) {
        auto message = std_msgs::msg::String();
        message.data = std::string(buffer.get(), total);

        read_cb_mutex_.lock();
        if (read_cb_ != nullptr) {
          // having pure pointers would improve performance here
          // by skipping data copying few lines above (message.data)
          // but it would be against the religion of so many
          read_cb_(message.data, read_cb_user_data_);
        }
        read_cb_mutex_.unlock();

        RCLCPP_INFO(get_logger_(), "Publishing: '%s'", message.data.c_str());
        intf_ros_->inspect_input->publish(message);
      }
    }
  }
}

const rclcpp::Logger Worker::get_logger_() {
  return rclcpp::get_logger("serial(" + settings_->dev_name.as_string() + ")");
}

}  // namespace serial