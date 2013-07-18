# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(lwr_simulation)

find_package(catkin REQUIRED COMPONENTS orocos_kdl kuka_lwr_fri)

find_package(Boost REQUIRED COMPONENTS thread)
find_package(GAZEBO REQUIRED)
find_package(SDF REQUIRED)
find_package(Eigen REQUIRED)

# http://stackoverflow.com/questions/7172670/best-shortest-way-to-join-a-list-in-cmake
function(JOIN VALUES GLUE OUTPUT)
  string (REGEX REPLACE "([^\\]|^);" "\\1${GLUE}" _TMP_STR "${VALUES}")
  string (REGEX REPLACE "[\\](.)" "\\1" _TMP_STR "${_TMP_STR}") #fixes escaping
  set (${OUTPUT} "${_TMP_STR}" PARENT_SCOPE)
endfunction()
JOIN("${GAZEBO_LDFLAGS}" " " LD_FLAGS)

message(STATUS "lwr_simulation: using LD_FLAGS = \"${LD_FLAGS}\"")

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${SDFormat_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

link_directories(${orocos_kdl_LIBRARY_DIRS})

add_library(lwr_controller src/lwr_controller.cpp)
set_target_properties(lwr_controller PROPERTIES LINK_FLAGS "${LD_FLAGS}")
target_link_libraries(lwr_controller
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${SDFormat_LIBRARIES}
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
