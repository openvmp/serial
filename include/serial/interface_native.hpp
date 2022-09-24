/*
 * OpenVMP, 2022
 *
 * Author: Roman Kuzmenko
 * Created: 2022-09-24
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_SERIAL_INTERFACE_NATIVE_H
#define OPENVMP_SERIAL_INTERFACE_NATIVE_H

#include <memory>
#include <string>

namespace serial {

class InterfaceNative {
 public:
  virtual void output(const std::string &) = 0;

  // having pure pointers would improve performance here
  // but it would be against the religion of so many
  virtual void register_input_cb(void (*)(const std::string &msg,
                                          void *user_data),
                                 void *user_data) = 0;

  virtual void inject_input(const std::string &) = 0;
};

}  // namespace serial

#endif  // OPENVMP_SERIAL_INTERFACE_NATIVE_H
