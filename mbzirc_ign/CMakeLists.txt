cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(mbzirc_ign)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(ignition-gazebo6 REQUIRED)
set(IGN_GAZEBO_VER ${ignition-gazebo6_VERSION_MAJOR})

find_package(ignition-common4 REQUIRED COMPONENTS graphics)
set(IGN_COMMON_VER ${ignition-common4_VERSION_MAJOR})
find_package(ignition-fuel_tools7 REQUIRED)
find_package(ignition-math6 REQUIRED)
set(IGN_MATH_VER ${ignition-math6_VERSION_MAJOR})
find_package(ignition-msgs8 REQUIRED)
find_package(ignition-transport11 REQUIRED)
set(IGN_TRANSPORT_VER ${ignition-transport11_VERSION_MAJOR})
find_package(ignition-plugin1 REQUIRED COMPONENTS loader register)
set(IGN_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})
find_package(ignition-rendering6 REQUIRED)
set(IGN_RENDERING_VER ${ignition-rendering6_VERSION_MAJOR})
find_package(sdformat12 REQUIRED)

find_package(std_msgs REQUIRED)

#============================================================================
# Hooks
ament_environment_hooks("hooks/resource_paths.dsv.in")
ament_environment_hooks("hooks/resource_paths.sh")

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
  BaseStation
  FixedWingController
  GameLogicPlugin
  EntityDetector
  RFRange
  SimpleHydrodynamics
  SuctionGripper
  Surface
  WaveVisual
)

foreach(PLUGIN ${MBZIRC_IGN_PLUGINS})
  add_library(${PLUGIN} SHARED src/${PLUGIN}.cc)
  target_link_libraries(${PLUGIN} PUBLIC
    ignition-gazebo${IGN_GAZEBO_VER}::core
    ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
    ignition-rendering${IGN_RENDERING_VER}::ignition-rendering${IGN_RENDERING_VER}
    Waves
  )
endforeach()

# copy of multicoptor control from ign-gazebo with custom modifications
add_library(MulticopterControl SHARED
    src/multicopter_control/MulticopterVelocityControl.cc
    src/multicopter_control/LeeVelocityController.cc
    src/multicopter_control/Common.cc
)
target_link_libraries(MulticopterControl PUBLIC
  ignition-gazebo${IGN_GAZEBO_VER}::core
  ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
  ignition-transport${IGN_TRANSPORT_VER}::ignition-transport${IGN_TRANSPORT_VER}
  ignition-math${IGN_MATH_VER}::ignition-math${IGN_MATH_VER}
)


install(
  TARGETS ${MBZIRC_IGN_PLUGINS} MulticopterControl
  DESTINATION lib)

ament_python_install_package(
  mbzirc_ign
  PACKAGE_DIR src/mbzirc_ign
)

#============================================================================
# Scripts
install(PROGRAMS
  examples/multi_uav_demo.py
  DESTINATION lib/${PROJECT_NAME}
)

#============================================================================
# Resources
install(DIRECTORY
  config
  worlds
  models
  launch
  DESTINATION share/${PROJECT_NAME})

#============================================================================

if(BUILD_TESTING)
  find_package(python_cmake_module REQUIRED)
  find_package(PythonExtra MODULE REQUIRED)

  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  find_package(ament_cmake_flake8 REQUIRED)
  ament_flake8()

  find_package(ament_cmake_pycodestyle REQUIRED)
  ament_pycodestyle()

  # Build-time constants
  set("PROJECT_BINARY_PATH" ${CMAKE_CURRENT_BINARY_DIR})
  set("PROJECT_SOURCE_PATH" ${CMAKE_CURRENT_SOURCE_DIR})
  configure_file(test/helper/TestConstants.hh.in TestConstants.hh @ONLY)

  foreach(TEST_TARGET
    test_spawn_uav_battery
    test_game_logic
    test_spawn_usv_maxspeed
    test_fixed_wing
    test_gripper
    test_spawn_usv_arm
    )
    ament_add_gtest(
      ${TEST_TARGET}
      test/${TEST_TARGET}.cc
    )
    target_include_directories(${TEST_TARGET}
      PRIVATE ${CMAKE_BINARY_DIR} ${CMAKE_BINARY_DIR}/proto)
    target_link_libraries(${TEST_TARGET} ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER})
    set_tests_properties(${TEST_TARGET} PROPERTIES TIMEOUT 300)
  endforeach()

  set (_pytest_tests
    src/mbzirc_ign/test_model.py
    src/mbzirc_ign/test_bridges.py
  )
  foreach(_test_path ${_pytest_tests})
    get_filename_component(_test_name ${_test_path} NAME_WE)
    ament_add_pytest_test(${_test_name} ${_test_path}
      PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE}"
      APPEND_ENV AMENT_PREFIX_PATH=${ament_index_build_path}
        PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
      TIMEOUT 120
      WERROR ON
    )
  endforeach()
endif()

ament_package()
