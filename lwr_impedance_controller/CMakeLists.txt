cmake_minimum_required(VERSION 2.8.3)
project(lwr_impedance_controller)

find_package(catkin REQUIRED COMPONENTS rtt_ros geometry_msgs std_msgs message_generation kdl_conversions lwr_fri xeno_clock)

add_message_files(DIRECTORY msg)
generate_messages(DEPENDENCIES geometry_msgs std_msgs)

catkin_package(
  DEPENDS rtt
  CATKIN_DEPENDS rtt_ros geometry_msgs std_msgs message_runtime
)

find_package(OROCOS-RTT REQUIRED)
find_package(Eigen REQUIRED)
find_package(orocos_kdl REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})
link_directories(${orocos_kdl_LIBRARY_DIRS})

ros_generate_rtt_typekit(lwr_impedance_controller)

# Defines the orocos_* cmake macros. See that file for additional
# documentation.
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)
add_definitions(-DRTT_COMPONENT)


orocos_component(cartesianimpedance src/CartesianImpedance.cpp)
target_link_libraries(cartesianimpedance ${orocos_kdl_LIBRARIES} ${kdl_conversions_LIBRARIES})
add_dependencies(cartesianimpedance ${PROJECT_NAME}_gencpp)

orocos_component(nullspaceprojection src/NullSpaceProjection.cpp)
target_link_libraries(nullspaceprojection ${orocos_kdl_LIBRARIES} ${kdl_conversions_LIBRARIES})

orocos_component(jointlimitsavoidance src/JointLimitsAvoidance.cpp)

orocos_component(singularityavoidance src/SingularityAvoidance.cpp)

orocos_component(impedancetrajectorygenerator src/ImpedanceTrajectoryGenerator.cpp)
target_link_libraries(impedancetrajectorygenerator ${orocos_kdl_LIBRARIES} ${kdl_conversions_LIBRARIES})
add_dependencies(impedancetrajectorygenerator ${PROJECT_NAME}_gencpp)

orocos_component(sumvectors2 src/SumVectors2.cpp)

orocos_generate_package()

