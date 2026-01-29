cmake_minimum_required(VERSION 3.8)
project(flycapture_ros2_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

include_directories(/usr/include/flycapture)

ament_auto_add_executable(flycapture_camera_driver_node
  src/flycapture_camera_driver_core.cpp
  src/flycapture_camera_driver_node.cpp
)
target_link_libraries(flycapture_camera_driver_node
  /usr/lib/libflycapture.so
  pthread
  m
)

ament_auto_package(INSTALL_TO_SHARE
    config
    launch
)