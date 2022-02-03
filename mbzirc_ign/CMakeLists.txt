cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(mbzirc_ign)

find_package(ament_cmake REQUIRED)

find_package(ignition-gazebo6 REQUIRED)
set(IGN_GAZEBO_VER ${ignition-gazebo6_VERSION_MAJOR})

find_package(ignition-common4 REQUIRED COMPONENTS graphics)
set(IGN_COMMON_VER ${ignition-common4_VERSION_MAJOR})
find_package(ignition-fuel_tools7 REQUIRED)
find_package(ignition-math6 REQUIRED)
set(IGN_MATH_VER ${ignition-math6_VERSION_MAJOR})
find_package(ignition-msgs8 REQUIRED)
find_package(ignition-transport11 REQUIRED)
find_package(ignition-plugin1 REQUIRED COMPONENTS loader register)
set(IGN_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})
find_package(sdformat12 REQUIRED)

#============================================================================
# Hooks
configure_file(
  "hooks/hook.dsv.in"
  "${CMAKE_CURRENT_BINARY_DIR}/hooks/hook.dsv" @ONLY
)


#============================================================================

# Waves
add_library(Waves SHARED
  src/Wavefield.cc
)
target_link_libraries(Waves PUBLIC
  ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
  ignition-gazebo${IGN_GAZEBO_VER}::core
  ignition-math${IGN_MATH_VER}
)
install(
  TARGETS Waves
  DESTINATION lib)

# Plugins
list(APPEND MBZIRC_IGN_PLUGINS
  GameLogicPlugin
  SimpleHydrodynamics
  Surface
)

foreach(PLUGIN ${MBZIRC_IGN_PLUGINS})
  add_library(${PLUGIN} SHARED src/${PLUGIN}.cc)
  target_link_libraries(${PLUGIN} PUBLIC
     ignition-gazebo${IGN_GAZEBO_VER}::core
     ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
     Waves
  )
endforeach()

install(
  TARGETS ${MBZIRC_IGN_PLUGINS}
  DESTINATION lib)

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
    test_spawn_uav_battery test_game_logic)
    ament_add_gtest(
      ${TEST_TARGET}
      test/${TEST_TARGET}.cc
    )
    target_include_directories(${TEST_TARGET}
      PRIVATE ${CMAKE_BINARY_DIR} ${CMAKE_BINARY_DIR}/proto )
    target_link_libraries(${TEST_TARGET} ignition-gazebo6::ignition-gazebo6)
    set_tests_properties(${TEST_TARGET} PROPERTIES TIMEOUT 120)
  endforeach()
endif()

ament_package()
