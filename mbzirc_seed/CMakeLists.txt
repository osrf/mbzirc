cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(mbzirc_seed)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ros_ign_interfaces REQUIRED)

find_package(ignition-math6 REQUIRED)
set(IGN_MATH_VER ${ignition-math6_VERSION_MAJOR})

add_library(mbzirc_seed
  SHARED
  src/ReadRfRange.cpp
  src/UavController.cpp
  src/UsvController.cpp
)

ament_target_dependencies(mbzirc_seed
  rclcpp
  rclcpp_components
  geometry_msgs
  sensor_msgs
  ros_ign_interfaces
)

target_include_directories(mbzirc_seed
  PUBLIC include
)

target_link_libraries(mbzirc_seed
  ignition-math${IGN_MATH_VER}::ignition-math${IGN_MATH_VER})

rclcpp_components_register_node(mbzirc_seed
  PLUGIN mbzirc_seed::UavController
  EXECUTABLE uav_controller)

rclcpp_components_register_node(mbzirc_seed
  PLUGIN mbzirc_seed::UsvController
  EXECUTABLE usv_controller)

rclcpp_components_register_node(mbzirc_seed
  PLUGIN mbzirc_seed::ReadRfRange
  EXECUTABLE read_rf_range)

install(TARGETS mbzirc_seed
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(mbzirc_seed)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(
  rclcpp
)

ament_package()
