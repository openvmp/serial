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

#ifndef OPENVMP_SERIAL_INTERFACE_NATIVE_H
#define OPENVMP_SERIAL_INTERFACE_NATIVE_H

#include <memory>
#include <string>

namespace serial {

class InterfaceNative {
 public:
  virtual void write(const std::string &) = 0;

  // having pure pointers would improve performance here
  // but it would be against the religion of so many
  virtual void register_read_cb(void (*)(const std::string &msg,
                                         void *user_data),
                                void *user_data) = 0;

  virtual void inject_read(const std::string &) = 0;
};

}  // namespace serial

#endif  // OPENVMP_SERIAL_INTERFACE_NATIVE_H
