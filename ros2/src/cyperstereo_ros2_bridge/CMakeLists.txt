cmake_minimum_required(VERSION 3.8)
project(cyperstereo_ros2_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

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

if (CMAKE_SYSTEM_NAME STREQUAL "Windows")
  message("WIN OS.")
  add_library(Cyperlib STATIC
    ../../../src/usb/uvc/uvc.h
    ../../../src/usb/uvc/types.h
    ../../../src/usb/uvc/win/uvc-wmf.cc)
elseif (CMAKE_SYSTEM_NAME STREQUAL "Darwin")
  message("MAC OS.")
elseif (CMAKE_SYSTEM_NAME STREQUAL "Linux")
  message("LINUX OS.")
  add_library(Cyperlib STATIC
    ../../../src/usb/uvc/uvc.h
    ../../../src/usb/uvc/types.h
    ../../../src/usb/uvc/linux/uvc-v4l2.cc)
else()
  message(FATAL_ERROR "Unsupported OS.")
endif()

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(cyperstereo_ros2_bridge src/capture_image_imu.cpp)
ament_target_dependencies(cyperstereo_ros2_bridge rclcpp cv_bridge image_transport sensor_msgs)
target_link_libraries(cyperstereo_ros2_bridge Cyperlib ${OpenCV_LIBRARIES})

install(
  TARGETS cyperstereo_ros2_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

