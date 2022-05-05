cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(mbzirc_naive_radar)

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
add_library(NaiveRadar SHARED
    src/NaiveRadar.cc
)
target_link_libraries(NaiveRadar PUBLIC
  ignition-gazebo${IGN_GAZEBO_VER}::core
  ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
  ignition-transport${IGN_TRANSPORT_VER}::ignition-transport${IGN_TRANSPORT_VER}
  ignition-sensors${IGN_SENSORS_VER}::ignition-sensors${IGN_SENSORS_VER}
  ignition-math${IGN_MATH_VER}::ignition-math${IGN_MATH_VER}
)

# build the bridge process
add_executable(naive_radar_bridge src/naive_radar_bridge.cc)
ament_target_dependencies(naive_radar_bridge
  PUBLIC
  rclcpp
  radar_msgs
  ros_ign_bridge
)
target_link_libraries(naive_radar_bridge PUBLIC
  ignition-msgs${IGN_MSGS_VER}::ignition-msgs${IGN_MSGS_VER}
  ignition-transport${IGN_TRANSPORT_VER}::ignition-transport${IGN_TRANSPORT_VER}
)

# Uncomment the install calls below to install the example naive radar model
# install(
#   TARGETS NaiveRadar
#   DESTINATION lib)
# install(DIRECTORY
#   models
#   launch
#   DESTINATION share/${PROJECT_NAME})
# install(TARGETS
#   naive_radar_bridge
#   DESTINATION lib/${PROJECT_NAME})

ament_package()
