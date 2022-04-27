find_package(radar_msgs REQUIRED)
find_package(ros_ign_bridge REQUIRED)
find_package(ignition-msgs8 REQUIRED)
find_package(ignition-transport11 REQUIRED)
set(IGN_TRANSPORT_VER ${ignition-transport11_VERSION_MAJOR})
find_package(rclcpp REQUIRED)

add_executable(naive_radar_bridge src/naive_radar_bridge.cc)
ament_target_dependencies(naive_radar_bridge
  PUBLIC
  rclcpp
  radar_msgs
  ros_ign_bridge
)
target_link_libraries(naive_radar_bridge PUBLIC
  ignition-msgs8
  ignition-transport11
)

# Resources
install(TARGETS
  naive_radar_bridge
  DESTINATION lib/${PROJECT_NAME})
