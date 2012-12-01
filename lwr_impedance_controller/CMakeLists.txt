cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()
rosbuild_include(rtt_rosnode GenerateRTTtypekit )

rosbuild_find_ros_package(rtt )
find_package(Orocos-RTT HINTS ${rtt_PACKAGE_PATH}/../install )

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)
 
orocos_component(cartesianimpedance src/CartesianImpedance.cpp)
orocos_component(nullspaceprojection src/NullSpaceProjection.cpp)
orocos_component(jointlimitsavoidance src/JointLimitsAvoidance.cpp)
orocos_component(singularityavoidance src/SingularityAvoidance.cpp)
orocos_component(impedancetrajectorygenerator src/ImpedanceTrajectoryGenerator.cpp)
orocos_component(sumvectors2 src/SumVectors2.cpp)

#Start of user code user_build
ros_generate_rtt_typekit(lwr_impedance_controller)
#End of user code

