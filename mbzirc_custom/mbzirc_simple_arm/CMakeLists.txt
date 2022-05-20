cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(mbzirc_simple_arm)

find_package(ament_cmake REQUIRED)

# Hooks
ament_environment_hooks("hooks/resource_paths.dsv.in")
ament_environment_hooks("hooks/resource_paths.sh")

# Uncomment the install call below to install the example simple arm model
# install(DIRECTORY
#   models
#   launch
#   DESTINATION share/${PROJECT_NAME})

ament_package()
