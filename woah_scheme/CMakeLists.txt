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

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)
pkg_check_modules(GUILE REQUIRED guile-2.0)

set(CMAKE_CXX_FLAGS "-I/usr/local/include/guile/2.0 -L/usr/local/lib -DPACKAGE_PATH=\\\"${PROJECT_SOURCE_DIR}\\\"")

rosbuild_add_executable(woah_scheme src/woah_scheme.cc src/transform.cc)
target_link_libraries(woah_scheme "guile-2.0")
target_link_libraries(woah_scheme "gc")
