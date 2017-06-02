cmake_minimum_required(VERSION 2.8.3)
project(rtt_lwr_msgs)

find_package(catkin REQUIRED COMPONENTS rtt_roscomm lwr_msgs rtt_std_msgs rtt_geometry_msgs)

ros_generate_rtt_typekit(lwr_msgs)

orocos_generate_package(
  DEPENDS lwr_msgs
  DEPENDS_TARGETS rtt_roscomm rtt_std_msgs rtt_geometry_msgs

)
