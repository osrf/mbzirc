cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(mbzirc_naive_3d_scanning_radar)

find_package(ament_cmake REQUIRED)
find_package(ignition-math6 REQUIRED)
set(IGN_MATH_VER ${ignition-math6_VERSION_MAJOR})
find_package(ignition-msgs8 REQUIRED)
set(IGN_MSGS_VER ${ignition-msgs8_VERSION_MAJOR})
find_package(ignition-transport11 REQUIRED)
set(IGN_TRANSPORT_VER ${ignition-transport11_VERSION_MAJOR})
find_package(ignition-plugin1 REQUIRED COMPONENTS loader register)
set(IGN_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})
find_package(ignition-sensors6 REQUIRED)
set(IGN_SENSORS_VER ${ignition-sensors6_VERSION_MAJOR})
find_package(ignition-gazebo6 REQUIRED)
set(IGN_GAZEBO_VER ${ignition-gazebo6_VERSION_MAJOR})
find_package(radar_msgs REQUIRED)
find_package(ros_ign_bridge REQUIRED)
find_package(rclcpp REQUIRED)

# Hooks
ament_environment_hooks("hooks/resource_paths.dsv.in")
ament_environment_hooks("hooks/resource_paths.sh")

# build the ign-gazebo system
add_library(Naive3dScanningRadar SHARED
    src/Naive3dScanningRadar.cc
)
target_link_libraries(Naive3dScanningRadar PUBLIC
  ignition-gazebo${IGN_GAZEBO_VER}::core
  ignition-msgs${IGN_MSGS_VER}::ignition-msgs${IGN_MSGS_VER}
  ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
  ignition-transport${IGN_TRANSPORT_VER}::ignition-transport${IGN_TRANSPORT_VER}
  ignition-sensors${IGN_SENSORS_VER}::ignition-sensors${IGN_SENSORS_VER}
  ignition-math${IGN_MATH_VER}::ignition-math${IGN_MATH_VER}
)

# Uncomment the install call below to install the example naive spinning radar
# model
install(DIRECTORY
  launch
  models
  DESTINATION share/${PROJECT_NAME})
install(
  TARGETS Naive3dScanningRadar
  DESTINATION lib)

ament_package()
