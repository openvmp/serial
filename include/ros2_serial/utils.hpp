/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-09-24
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_SERIAL_UTILS_H
#define OPENVMP_SERIAL_UTILS_H

#include <sstream>
#include <string>

namespace ros2_serial {

namespace utils {

static inline std::string bin2hex(const std::string &bin) {
  std::stringstream ss;

  ss << std::hex;

  for (size_t i = 0; i < bin.size(); i++) {
    ss << std::setfill('0') << std::setw(2) << (int)(uint8_t)((int)bin[i])
       << " ";
  }
  return ss.str();
}

}  // namespace utils

}  // namespace ros2_serial

#endif  // OPENVMP_SERIAL_UTILS_H
