cmake_minimum_required(VERSION 2.8.3)
project(trajectoryGenerator)

find_package(catkin REQUIRED COMPONENTS roscpp tf)

set(CMAKE_CXX_FLAGS "-g ${OTHERS_DEFINE}")

add_executable(trajectoryGenerator trajectoryGenerator.cpp)

target_link_libraries(trajectoryGenerator ${catkin_LIBRARIES})

add_dependencies(trajectoryGenerator quadrivio_msgs_generate_messages_cpp)
