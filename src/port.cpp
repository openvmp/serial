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

#define _BSD_SOURCE
#include "serial/port.hpp"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

namespace serial {

int PortSettings::setup() {
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
      tty.c_cflag |= CRTSCTS;
    } else {
      tty.c_cflag &= ~CRTSCTS;
    }

    tty.c_cflag |= CREAD | CLOCAL;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);  // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    // Setting both to 0 will give a non-blocking read
    tty.c_cc[VTIME] = 0;
    tty.c_cc[VMIN] = 0;
    tty.c_lflag &= ~ICANON;  // Turn off canonical input
    tty.c_lflag &= ~(ECHO);  // Turn off echo
    tty.c_lflag &= ~ISIG;  // Disables recognition of INTR (interrupt), QUIT and
                           // SUSP (suspend) characters

    // Set the terminal attributes
    ::tcflush(fd, TCIFLUSH);
    if (::tcsetattr(fd, TCSANOW, &tty) != 0) {
      ::close(fd);
      throw std::invalid_argument("failed to set the device attributes");
    }
  }

  // Get the file descriptor attributes
  ::fcntl(fd, F_NOCACHE, 1);
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

}  // namespace serial