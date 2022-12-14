/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-09-24
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "serial/worker.hpp"

#include "serial/interface.hpp"
#include "serial/utils.hpp"

#define _BSD_SOURCE
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>

namespace serial {

Worker::Worker(Implementation *impl, std::shared_ptr<PortSettings> settings)
    : impl_(impl),
      settings_(settings),
      fd_(-1),
      signal_{-1, -1},
      do_stop_(false),
      input_cb_{nullptr},
      input_cb_user_data_{nullptr} {
  fd_ = settings->setup(fd_);
  if (fd_ < 0) {
    throw std::invalid_argument("failed to conigure the port");
  }

  // Create a pipe to send signals into the worker threads
  if (::pipe(signal_) < 0) {
    ::close(fd_);
    fd_ = -1;
    throw std::invalid_argument("failed to creat a signalling channel");
  }

  // Make the recieving side non-blocking
  int flags = ::fcntl(signal_[0], F_GETFL, 0);
  if (flags == -1) {
    throw std::invalid_argument("failed to get the pipe descriptor attributes");
  }

  flags |= O_NONBLOCK;

  if (::fcntl(signal_[0], F_SETFL, flags)) {
    throw std::invalid_argument("failed to set the pipe descriptor attributes");
  }

  thread_ = std::shared_ptr<std::thread>(new std::thread(&Worker::run_, this));

  RCLCPP_INFO(this->get_logger_(), "Worker initialization complete for %s",
              settings_->dev_name.as_string().c_str());
}

Worker::~Worker() { stop(); }

void Worker::output(const std::string &msg) {
  RCLCPP_DEBUG(get_logger_(), "output() with %lu bytes", msg.size());

  std::lock_guard<std::mutex> guard(output_queue_mutex_);
  output_queue_.push_back(std::pair<std::string, int>(msg, 0));

  ::write(signal_[1], " ", 1);
}

void Worker::register_input_cb(void (*cb)(const std::string &, void *),
                               void *user_data) {
  RCLCPP_DEBUG(get_logger_(), "register_input_cb()");

  std::lock_guard<std::mutex> guard(input_cb_mutex_);
  input_cb_ = cb;
  input_cb_user_data_ = user_data;
}

void Worker::inject_input(const std::string &msg) {
  RCLCPP_DEBUG(get_logger_(), "inject_input() with %lu bytes", msg.size());

  std::lock_guard<std::mutex> guard(input_cb_mutex_);
  if (input_cb_) {
    input_cb_(msg, input_cb_user_data_);
  }
}

void Worker::stop() {
  if (do_stop_) {
    // already stopped, ping the worker thread just in case
    ::write(signal_[1], " ", 1);
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
  max_fds++;

  while (true) {
    FD_ZERO(&read_fds);
    FD_ZERO(&write_fds);
    FD_ZERO(&except_fds);
    FD_SET(fd_, &read_fds);
    FD_SET(signal_[0], &read_fds);
    if (output_queue_.size() > 0) {
      FD_SET(fd_, &write_fds);
    }

    int nevents =
        ::select(max_fds, &read_fds, &write_fds, &except_fds, nullptr);
    if (nevents < 0) {
      RCLCPP_ERROR(get_logger_(), "select() failed for %s",
                   settings_->dev_name.as_string().c_str());
      break;
    }
    RCLCPP_DEBUG(get_logger_(), "woke up from select on %d events", nevents);

    if (FD_ISSET(signal_[0], &read_fds)) {
      // consume up to 32 signals at a time
      char c[32];
      (void)read(signal_[0], &c[0], sizeof(c));

      // check if it's a signal to terminate the threads
      if (do_stop_) {
        RCLCPP_INFO(get_logger_(), "exiting IO loop gracefully");
        break;
      }

      // start processing the send queue
      output_queue_mutex_.lock();
      while (output_queue_.size() > 0) {
        auto next = output_queue_.begin();
        output_queue_mutex_.unlock();
        // We do not need to keep the lock during IO,
        // since the other threads use .push_back() and,
        // thus, do not invalidate this iterator.

        // Now attempt to write
        const char *read_pos = next->first.data() + next->second;
        const int remains_to_write = next->first.length() - next->second;

        RCLCPP_DEBUG(get_logger_(), "attempting to output %d bytes",
                     remains_to_write);

        int wrote = ::write(fd_, read_pos, remains_to_write);
        if (wrote < 0) {
          // TODO(clairbee): check the errno in this thread
          // TODO(clairbee): if not EAGAIN
          // TODO(clairbee):   break the IO loop and re-open the file
          RCLCPP_INFO(get_logger_(), "error while writing data: %d bytes",
                      remains_to_write);
          output_queue_mutex_.lock();
          continue;  // hoping it was EAGAIN
        }
        if (wrote == 0) {
          // TODO(clairbee): break the IO loop and re-open the file
          RCLCPP_ERROR(get_logger_(), "EOF while writing data: %d bytes",
                       remains_to_write);
          output_queue_mutex_.lock();
          break;
        }

        // Now publish what is written successfully
        auto message = std_msgs::msg::String();
        message.data = std::string(read_pos, wrote);
        RCLCPP_INFO(get_logger_(), "Publishing written data: '%s'",
                    utils::bin2hex(message.data).c_str());
        impl_->inspect_output->publish(message);

        // Advance the queue read position
        if (wrote == remains_to_write) {
          output_queue_mutex_.lock();
          output_queue_.erase(next);
        } else {
          next->second += wrote;
          output_queue_mutex_.lock();
        }
      }
      output_queue_mutex_.unlock();
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
        RCLCPP_INFO(get_logger_(), "exiting IO loop due to EOF");
        break;
      }

      if (total > 0) {
        auto message = std_msgs::msg::String();
        message.data = std::string(buffer.get(), total);

        RCLCPP_DEBUG(get_logger_(), "received input of %d bytes", total);

        input_cb_mutex_.lock();
        if (input_cb_ != nullptr) {
          // having pure pointers would improve performance here
          // by skipping data copying few lines above (message.data)
          // but, unfortunately, it would be against the religion of so many
          input_cb_(message.data, input_cb_user_data_);
        }
        input_cb_mutex_.unlock();

        RCLCPP_INFO(get_logger_(), "Publishing received: '%s'",
                    utils::bin2hex(message.data).c_str());
        impl_->inspect_input->publish(message);
      }
    }
  }
}

const rclcpp::Logger Worker::get_logger_() {
  return rclcpp::get_logger("serial(" + settings_->dev_name.as_string() + ")");
}

}  // namespace serial