cmake_minimum_required(VERSION 2.8.6)

project(lwr_msgs)

find_package(catkin REQUIRED
  COMPONENTS message_generation)

add_message_files(FILES
  FriRobotState.msg
  FriIntfState.msg
  FriIntfStatistics.msg
  )
  
generate_messages()

catkin_package(CATKIN_DEPENDS message_runtime)
