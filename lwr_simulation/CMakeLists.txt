# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(lwr_simulation)

find_package(catkin REQUIRED COMPONENTS kdl_parser kuka_lwr_fri lwr_fri rtt_gazebo_plugin)

find_package(gazebo REQUIRED)
find_package(OROCOS-RTT REQUIRED COMPONENTS rtt-scripting rtt-transport-corba)
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )
find_package(orocos_kdl REQUIRED)

# http://stackoverflow.com/questions/7172670/best-shortest-way-to-join-a-list-in-cmake
function(JOIN VALUES GLUE OUTPUT)
  string (REGEX REPLACE "([^\\]|^);" "\\1${GLUE}" _TMP_STR "${VALUES}")
  string (REGEX REPLACE "[\\](.)" "\\1" _TMP_STR "${_TMP_STR}") #fixes escaping
  set (${OUTPUT} "${_TMP_STR}" PARENT_SCOPE)
endfunction()
JOIN("${GAZEBO_LDFLAGS}" " " LD_FLAGS)

message(STATUS "lwr_simulation: using LD_FLAGS = \"${LD_FLAGS}\"")

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${orocos_kdl_LIBRARY_DIRS} 
  )

include_directories(
  include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
)

link_directories(${orocos_kdl_LIBRARY_DIRS})

orocos_component(lwr_controller src/lwr_controller.cpp)
set_target_properties(lwr_controller PROPERTIES LINK_FLAGS "${LD_FLAGS}")
target_link_libraries(lwr_controller
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

catkin_package(
  CATKIN_DEPENDS
    gazebo_ros
  LIBRARIES
    lwr_controller
)

# Install targets
install(TARGETS
  lwr_controller
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )


install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  )

install(DIRECTORY worlds/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds
  )
