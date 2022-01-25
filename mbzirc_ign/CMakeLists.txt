cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(mbzirc_ign)

find_package(ament_cmake REQUIRED)

find_package(ignition-gazebo6 REQUIRED)
set(IGN_GAZEBO_VER ${ignition-gazebo6_VERSION_MAJOR})

find_package(ignition-common4 REQUIRED COMPONENTS graphics)
find_package(ignition-fuel_tools7 REQUIRED)
find_package(ignition-math6 REQUIRED)
find_package(ignition-msgs8 REQUIRED)
find_package(ignition-transport11 REQUIRED)
find_package(ignition-plugin1 REQUIRED COMPONENTS loader register)
set(IGN_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})
find_package(sdformat12 REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

#============================================================================
# Hooks
configure_file(
  "hooks/hook.dsv.in"
  "${CMAKE_CURRENT_BINARY_DIR}/hooks/hook.dsv" @ONLY
)


#============================================================================
# Plugins
list(APPEND MBZIRC_IGN_PLUGINS
  SimpleHydrodynamics
  Surface
)

foreach(PLUGIN ${MBZIRC_IGN_PLUGINS})
  add_library(${PLUGIN} SHARED src/${PLUGIN}.cc)
  target_link_libraries(${PLUGIN} PUBLIC
     ignition-gazebo${IGN_GAZEBO_VER}::core
     ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
  )
endforeach()

install(
  TARGETS ${MBZIRC_IGN_PLUGINS}
  DESTINATION lib)

#============================================================================
# ROS Bridge
add_executable(MavBridge src/MavBridge.cc)
ament_target_dependencies(MavBridge
    rclcpp
    std_msgs
    ignition-transport11
    ignition-msgs8
  )
install(
  TARGETS MavBridge
  DESTINATION lib/${PROJECT_NAME})

#============================================================================
# Resources
install(DIRECTORY
  worlds
  models
  launch
  ${CMAKE_CURRENT_BINARY_DIR}/hooks
  DESTINATION share/${PROJECT_NAME})


ament_package()
