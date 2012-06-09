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
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_library(qc_serial src/qc/qc_serial.cc)

rosbuild_add_executable(motor_driver src/motor/motor_driver.cc src/motor/MotorDriver.cc src/motor/MotorController.cc)
target_link_libraries(motor_driver qc_serial)
rosbuild_add_executable(motor_test src/motor/motor_test.cc src/motor/MotorDriver.cc src/motor/MotorController.cc)
target_link_libraries(motor_test qc_serial)
rosbuild_add_executable(encoder_test src/motor/encoder_test.cc src/motor/MotorDriver.cc src/motor/MotorController.cc)
target_link_libraries(encoder_test qc_serial)
rosbuild_add_executable(light_test src/motor/light_test.cc src/motor/MotorDriver.cc src/motor/MotorController.cc)
target_link_libraries(light_test qc_serial)

rosbuild_add_executable(gps_test src/pose/gps_test.cc src/pose/CalcUTM.cc src/pose/llutm.cc src/pose/GpsProxy.cc)
target_link_libraries(gps_test qc_serial pcrecpp gps)

rosbuild_add_executable(pose_driver src/pose/CalcUTM.cc src/pose/pose_driver.cc src/pose/GpsProxy.cc src/pose/llutm.cc src/pose/robot_ekf.cpp)
target_link_libraries(pose_driver qc_serial pcrecpp gps kalman)

#rosbuild_add_executable(calibrate src/compass/calibrate.cc  src/compass/PhidgetCompass.cc)
#target_link_libraries(calibrate phidget21)
#rosbuild_add_executable(compass_node src/compass/compass_node.cc src/compass/PhidgetCompass.cc)
#target_link_libraries(compass_node phidget21)
#rosbuild_add_executable(test_compass src/compass/test_compass.cc src/compass/PhidgetCompass.cc)
#target_link_libraries(test_compass phidget21)
