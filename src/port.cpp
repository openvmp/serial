/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-09-24
 *
 * Licensed under Apache License, Version 2.0.
 */

#define _DEFAULT_SOURCE
#include "remote_serial/port.hpp"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include "remote_serial/worker.hpp"

namespace remote_serial {

Port::Port(rclcpp::Node *node)
    : Implementation(node), port_settings_(new PortSettings()) {
  node_->declare_parameter("serial_dev_name", "/dev/ttyS0");
  node_->declare_parameter("serial_skip_init", false);
  node_->declare_parameter("serial_baud_rate", 115200);
  node_->declare_parameter("serial_data", 8);
  node_->declare_parameter("serial_parity", false);
  node_->declare_parameter("serial_stop", 1);
  node_->declare_parameter("serial_flow_control", false);
  node_->declare_parameter("serial_sw_flow_control", false);
  node_->declare_parameter("serial_bs", 1024);
  node_->get_parameter("serial_dev_name", port_settings_->dev_name);
  node_->get_parameter("serial_skip_init", port_settings_->skip_init);
  node_->get_parameter("serial_baud_rate", port_settings_->baud_rate);
  node_->get_parameter("serial_data", port_settings_->data);
  node_->get_parameter("serial_stop", port_settings_->stop);
  node_->get_parameter("serial_parity", port_settings_->parity);
  node_->get_parameter("serial_flow_control", port_settings_->flow_control);
  node_->get_parameter("serial_sw_flow_control",
                       port_settings_->sw_flow_control);
  node_->get_parameter("serial_bs", port_settings_->bs);

  RCLCPP_INFO(node_->get_logger(), "Serial node initialization complete for %s",
              port_settings_->dev_name.as_string().c_str());

  // Topics are initialized prior to the worker.
  // That is done to put the burden of null checks on the DDS side.
  // Should the burden be on the device driver side,
  // then the conditions for the serial line saturation will be met more often.
  worker_ = std::make_shared<Worker>(this, port_settings_);

  init_serial_();
}

void Port::output(const std::string &msg) { worker_->output(msg); }

// having pure pointers would improve performance here
// but it would be against the religion of so many
void Port::register_input_cb(void (*input_cb)(const std::string &msg,
                                              void *user_data),
                             void *user_data) {
  worker_->register_input_cb(input_cb, user_data);
}

void Port::inject_input(const std::string &msg) { worker_->inject_input(msg); }

int PortSettings::setup(int old_fd) {
  if (old_fd != -1) {
    close(old_fd);
  }

  int fd = ::open(dev_name.as_string().data(), O_RDWR);
  if (fd < 0) {
    throw std::invalid_argument("failed to open the device");
  }

  if (!skip_init.as_bool()) {
    // Get terminal attributes
    struct termios tty;
    ::memset(&tty, 0, sizeof(tty));
    if (::tcgetattr(fd, &tty) != 0) {
      ::close(fd);
      throw std::invalid_argument("failed to get the device attributes");
    }

    // Baud rate
    switch (baud_rate.as_int()) {
      case 0:
        cfsetispeed(&tty, B0);
        break;
      case 50:
        cfsetispeed(&tty, B50);
        break;
      case 75:
        cfsetispeed(&tty, B75);
        break;
      case 110:
        cfsetispeed(&tty, B110);
        break;
      case 134:
        cfsetispeed(&tty, B134);
        break;
      case 150:
        cfsetispeed(&tty, B150);
        break;
      case 200:
        cfsetispeed(&tty, B200);
        break;
      case 300:
        cfsetispeed(&tty, B300);
        break;
      case 600:
        cfsetispeed(&tty, B600);
        break;
      case 1200:
        cfsetispeed(&tty, B1200);
        break;
      case 1800:
        cfsetispeed(&tty, B1800);
        break;
      case 2400:
        cfsetispeed(&tty, B2400);
        break;
      case 4800:
        cfsetispeed(&tty, B4800);
        break;
      case 9600:
        cfsetispeed(&tty, B9600);
        break;
      case 19200:
        cfsetispeed(&tty, B19200);
        break;
      case 38400:
        cfsetispeed(&tty, B38400);
        break;
      case 57600:
        cfsetispeed(&tty, B57600);
        break;
      case 115200:
        cfsetispeed(&tty, B115200);
        break;
      case 230400:
        cfsetispeed(&tty, B230400);
        break;
      // case 460800:
      //   cfsetispeed(&tty, B460800);
      //   break;
      default:
        throw std::invalid_argument("unsupported baud rate");
    }

    // Data bits
    tty.c_cflag &= ~CSIZE;
    switch (data.as_int()) {
      case 5:
        tty.c_cflag |= CS5;
        break;
      case 6:
        tty.c_cflag |= CS6;
        break;
      case 7:
        tty.c_cflag |= CS7;
        break;
      case 8:
        tty.c_cflag |= CS8;
        break;
    }

    // Stop bits
    if (stop.as_int() == 1) {
      tty.c_cflag &= ~CSTOPB;
    } else {
      tty.c_cflag |= CSTOPB;
    }

    // Parity
    // TODO(clairbee): handle this as a dictionary to support odd parity
    if (parity.as_bool()) {
      tty.c_cflag |= PARENB;
      tty.c_cflag &= ~PARODD;
    } else {
      tty.c_cflag &= ~PARENB;
    }

    // Flow control
    if (flow_control.as_bool()) {
      tty.c_cflag |= CRTSCTS;  // Turn on h/w flow control
    } else {
      tty.c_cflag &= ~CRTSCTS;
    }
    if (sw_flow_control.as_bool()) {
      tty.c_iflag |= (IXON | IXOFF | IXANY);  // Turn on s/w flow ctrl
    } else {
      tty.c_iflag &= ~(IXON | IXOFF | IXANY);  // Turn off s/w flow ctrl
    }

    tty.c_cflag |= CREAD | CLOCAL;
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    // Setting both to 0 will give a non-blocking read
    tty.c_cc[VTIME] = 0;
    tty.c_cc[VMIN] = 0;
    tty.c_lflag &= ~ICANON;  // Turn off canonical input
    tty.c_lflag &= ~(ECHO);  // Turn off echo
    tty.c_lflag &= ~ECHOE;   // Disable erasure
    tty.c_lflag &= ~ECHONL;  // Disable new-line echo
    tty.c_lflag &= ~ISIG;  // Disables recognition of INTR (interrupt), QUIT and
                           // SUSP (suspend) characters
    tty.c_oflag &= ~OPOST;  // Prevent special interpretation of output bytes
                            // (e.g. newline chars)
    tty.c_oflag &=
        ~ONLCR;  // Prevent conversion of newline to carriage return/line feed

    // Set the terminal attributes
    ::tcflush(fd, TCIFLUSH);
    if (::tcsetattr(fd, TCSANOW, &tty) != 0) {
      ::close(fd);
      throw std::invalid_argument("failed to set the device attributes");
    }
  }

  // Get the file descriptor attributes
#if defined(F_NOCACHE)
  ::fcntl(fd, F_NOCACHE, 1);
#endif
  int flags = ::fcntl(fd, F_GETFL, 0);
  if (flags == -1) {
    ::close(fd);
    throw std::invalid_argument("failed to get the file descriptor attributes");
  }

  flags |= O_NONBLOCK;

  // Set the file attributes
  if (::fcntl(fd, F_SETFL, flags)) {
    ::close(fd);
    throw std::invalid_argument("failed to set the file descriptor attributes");
  }

  return fd;
}

}  // namespace remote_serial
