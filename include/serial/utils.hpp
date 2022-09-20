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

#ifndef OPENVMP_SERIAL_UTILS_H
#define OPENVMP_SERIAL_UTILS_H

#include <sstream>
#include <string>

namespace serial {

namespace utils {

static inline std::string bin2hex(const std::string &bin) {
  std::stringstream ss;

  ss << std::hex << std::setfill('0') << std::setw(2);

  for (const auto &item : bin) {
    ss << int(item) << " ";
  }
  return ss.str();
}

}  // namespace utils

}  // namespace serial

#endif  // OPENVMP_SERIAL_UTILS_H