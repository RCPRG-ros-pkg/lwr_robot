cmake_minimum_required(VERSION 2.8.6)

project(lwr_msgs)

find_package(catkin REQUIRED
  COMPONENTS message_generation std_msgs geometry_msgs)

add_message_files(FILES
  FriRobotState.msg
  FriIntfState.msg
  FriIntfStatistics.msg
  LwrCmd.msg
  LwrStatus.msg
  )
  
generate_messages(DEPENDENCIES
  std_msgs geometry_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs geometry_msgs)
