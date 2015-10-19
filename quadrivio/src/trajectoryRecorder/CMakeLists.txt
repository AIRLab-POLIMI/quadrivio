cmake_minimum_required(VERSION 2.8.3)
project(trajectoryRecorder)

find_package(catkin REQUIRED COMPONENTS roscpp tf)

set(CMAKE_CXX_FLAGS "-g ${OTHERS_DEFINE}")

add_executable(trajectoryRecorder trajectoryRecorder.cpp)

target_link_libraries(trajectoryRecorder ${catkin_LIBRARIES})

add_dependencies(trajectoryRecorder quadrivio_msgs_generate_messages_cpp)
