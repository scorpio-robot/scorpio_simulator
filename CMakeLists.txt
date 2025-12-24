cmake_minimum_required(VERSION 3.8)
project(scorpio_simulator)

## Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################

find_package(ament_cmake_auto REQUIRED)
find_package(ignition-msgs8 REQUIRED)
set(IGN_MSGS_VER 8)
find_package(ignition-transport11 REQUIRED)
set(IGN_TRANSPORT_VER 11)
find_package(ignition-gazebo6 REQUIRED)
set(IGN_GAZEBO_VER 6)
ament_auto_find_build_dependencies()

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/gazebo.dsv.in")

###########
## Build ##
###########

# Build GlobalOdometryPublisher plugin
ament_auto_add_library(GlobalOdometryPublisher SHARED
  src/global_odometry_publisher.cpp
)

target_link_libraries(GlobalOdometryPublisher
  ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER}
  ignition-transport${IGN_TRANSPORT_VER}::ignition-transport${IGN_TRANSPORT_VER}
  ignition-msgs${IGN_MSGS_VER}::ignition-msgs${IGN_MSGS_VER}
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  set(ament_cmake_clang_format_CONFIG_FILE "${CMAKE_SOURCE_DIR}/.clang-format")
  set(ament_cmake_clang_tidy_CONFIG_FILE "${CMAKE_SOURCE_DIR}/.clang-tidy")
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

#############
## Install ##
#############

ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
  config
  launch
  rviz
  resource
)
