cmake_minimum_required(VERSION 2.8.3)
project(poses)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package()

install(FILES
  PATTERN ".json" INCLUDE
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

catkin_install_python(PROGRAMS
  store_pose.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)