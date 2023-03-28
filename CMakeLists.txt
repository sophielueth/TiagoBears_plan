cmake_minimum_required(VERSION 2.8.3)
project(TiagoBears_plan)
find_package(catkin REQUIRED COMPONENTS
  control_msgs
  geometry_msgs
  trajectory_msgs
  std_msgs
  nav_msgs
  moveit_msgs
  rospy
  tf
  moveit_commander
  actionlib

  gazebo_ros
  TiagoBears_grasp
  TiagoBears_PoseEstimation
  TiagoBears_ColorDetection
)
catkin_package(
  CATKIN_DEPENDS
    control_msgs
    geometry_msgs
    trajectory_msgs
    std_msgs
    nav_msgs
    moveit_msgs
    rospy
    tf
    moveit_commander
    actionlib
    TiagoBears_grasp
    TiagoBears_PoseEstimation
    TiagoBears_ColorDetection
)

catkin_install_python(PROGRAMS scripts/test_node.py
			       scripts/test_node_pose_est.py
             scripts/TiagoBears_stack.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_python_setup()
