cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(mbzirc_naive_spinning_radar)

find_package(ament_cmake REQUIRED)

# Hooks
ament_environment_hooks("hooks/resource_paths.dsv.in")
ament_environment_hooks("hooks/resource_paths.sh")

# Uncomment the install call below to install the example naive spinning radar
# model
# install(DIRECTORY
#   models
#   DESTINATION share/${PROJECT_NAME})

ament_package()
