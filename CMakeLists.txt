cmake_minimum_required(VERSION 3.8)
project(serial)

# Compiler settings
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
  include
)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Logic shared by all targets
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/InjectInput.srv"
  "srv/InjectOutput.srv"
)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

set(serial_SOURCE_FILES
  src/node.cpp
  src/worker.cpp
  src/port.cpp
  src/interface_ros.cpp
)

set(serial_DEPENDENCIES
  rclcpp
  std_msgs
)

# Executable target
add_executable(serial_standalone src/main.cpp ${serial_SOURCE_FILES})
target_link_libraries(serial_standalone ${rclcpp_LIBRARIES} "${cpp_typesupport_target}")
ament_target_dependencies(serial_standalone ${serial_DEPENDENCIES})
install(TARGETS
  serial_standalone
  DESTINATION lib/${PROJECT_NAME}
)

# Library target
add_library(serial_native ${serial_SOURCE_FILES})
target_link_libraries(serial_native ${rclcpp_LIBRARIES} "${cpp_typesupport_target}")
ament_target_dependencies(serial_native ${serial_DEPENDENCIES})
ament_export_dependencies(rclcpp)
install(TARGETS
  serial_native
  DESTINATION lib/${PROJECT_NAME}
)
# TODO(clairbee): ament_export_targets(export_serial HAS_LIBRARY_TARGET)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
