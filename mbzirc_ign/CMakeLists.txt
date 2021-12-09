cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(mbzirc_ign)

find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-common3 REQUIRED COMPONENTS graphics)
find_package(ignition-fuel_tools5 REQUIRED)
find_package(ignition-math6 REQUIRED)
find_package(ignition-msgs6 REQUIRED)
find_package(ignition-transport9 REQUIRED)
find_package(ignition-plugin1 REQUIRED COMPONENTS loader register)
find_package(sdformat10 REQUIRED)

#============================================================================
# Hooks
configure_file(
  "hooks/hook.dsv.in"
  "${CMAKE_CURRENT_BINARY_DIR}/hooks/hook.dsv" @ONLY
)

#============================================================================
# Resources
install(DIRECTORY
  worlds
  models
  ${CMAKE_CURRENT_BINARY_DIR}/hooks
  DESTINATION share/${PROJECT_NAME})

