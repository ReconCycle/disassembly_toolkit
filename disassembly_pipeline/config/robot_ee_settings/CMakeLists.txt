cmake_minimum_required(VERSION 2.8.3)
project(robot_ee_settings)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package()

install(FILES
  PATTERN ".json" INCLUDE
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


