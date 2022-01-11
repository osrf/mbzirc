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

  # Fetch and configure GTest
  include(FetchContent)
  FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/609281088cfefc76f9d0ce82e1ff6c30cc3591e5.zip
  )
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)

  enable_testing()
  include(Dart)

  # Build-time constants
  set("PROJECT_BINARY_PATH" ${CMAKE_CURRENT_BINARY_DIR})
  set("PROJECT_SOURCE_PATH" ${CMAKE_CURRENT_SOURCE_DIR})
  configure_file(test/helper/TestConstants.hh.in TestConstants.hh @ONLY)

  include(GoogleTest)

  foreach(TEST_TARGET
    test_spawn_uav_battery)

    add_executable(
      ${TEST_TARGET}
      test/${TEST_TARGET}.cc
    )
    target_include_directories(${TEST_TARGET}
      PUBLIC ${CMAKE_BINARY_DIR}/proto)
    target_link_libraries(${TEST_TARGET}
      PUBLIC gtest_main
      PRIVATE ignition-gazebo6::ignition-gazebo6
    )
    include_directories(${CMAKE_CURRENT_BINARY_DIR})
    gtest_discover_tests(${TEST_TARGET})
  endforeach()
endif()


ament_package()
