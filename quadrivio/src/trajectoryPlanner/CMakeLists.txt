cmake_minimum_required(VERSION 2.8.3)
project(trajectoryPlanner)

find_package(catkin REQUIRED COMPONENTS roscpp)
#find_package(sbpl REQUIRED)

set(CMAKE_CXX_FLAGS "-g ${OTHERS_DEFINE} -std=c++11")

include_directories( . )

add_executable(trajectoryPlanner trajectoryPlanner.cpp)

target_link_libraries(trajectoryPlanner ${catkin_LIBRARIES} sbpl)

add_dependencies(trajectoryPlanner quadrivio_msgs_generate_messages_cpp)
