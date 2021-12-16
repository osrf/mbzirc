cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(mbzirc_ros)

find_package(ament_cmake REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)


add_executable(pose_tf_broadcaster src/pose_tf_broadcaster.cc)
ament_target_dependencies(pose_tf_broadcaster
   geometry_msgs
   rclcpp
   tf2
   tf2_ros
   tf2_msgs
)

# Resources
install(TARGETS
  pose_tf_broadcaster
  DESTINATION lib/${PROJECT_NAME})


ament_package()
