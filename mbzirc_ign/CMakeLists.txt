cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(mbzirc_ign)

find_package(ament_cmake REQUIRED)

find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-common4 REQUIRED COMPONENTS graphics)
find_package(ignition-fuel_tools7 REQUIRED)
find_package(ignition-math6 REQUIRED)
find_package(ignition-msgs8 REQUIRED)
find_package(ignition-transport11 REQUIRED)
find_package(ignition-plugin1 REQUIRED COMPONENTS loader register)
find_package(sdformat12 REQUIRED)

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
  launch
  ${CMAKE_CURRENT_BINARY_DIR}/hooks
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Build-time constants
  set("PROJECT_BINARY_PATH" ${CMAKE_CURRENT_BINARY_DIR})
  set("PROJECT_SOURCE_PATH" ${CMAKE_CURRENT_SOURCE_DIR})
  configure_file(test/helper/TestConstants.hh.in TestConstants.hh @ONLY)

  foreach(TEST_TARGET
    test_spawn_uav_battery)

    ament_add_gtest(
      ${TEST_TARGET}
      test/${TEST_TARGET}.cc
    )
    target_include_directories(${TEST_TARGET} 
      PRIVATE ${CMAKE_BINARY_DIR} ${CMAKE_BINARY_DIR}/proto )
    target_link_libraries(${TEST_TARGET} ignition-gazebo6::ignition-gazebo6)
  endforeach()
endif()

ament_package()
