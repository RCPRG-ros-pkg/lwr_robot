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

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_find_ros_package(xacro)
include(${xacro_PACKAGE_PATH}/cmake/xacro.cmake)

set(config_files "")

file(GLOB config_xacro_files ${PROJECT_SOURCE_DIR}/configs/*.xml.xacro) 

foreach(it ${config_xacro_files})
  get_filename_component(basepath ${it} PATH)
  get_filename_component(basename ${it} NAME_WE)
  
  message(" processing world file :",${basepath}," : ",${basename})
  
  set(expanded_file "${basepath}/${basename}.xml")
  message("    produce expanded file :",${expanded_file})
  
  add_custom_target(media_files ALL "${xacro_PACKAGE_PATH}/xacro.py" "${it}" "-o" "${expanded_file}" )
endforeach(it)


