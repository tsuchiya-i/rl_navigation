cmake_minimum_required(VERSION 3.0.2)
project(rl_navigation)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

find_package(PythonLibs REQUIRED) #追加

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  scripts/input_publisher.py
  scripts/ddpg_navi.py
  scripts/controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

