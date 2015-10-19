cmake_minimum_required(VERSION 2.8.3)
project(utils)

find_package(catkin REQUIRED COMPONENTS roscpp heartbeat tf)

set(CMAKE_CXX_FLAGS "-g ${OTHERS_DEFINE}")

include_directories(${catkin_INCLUDE_DIRS})

add_executable(pathDisplay pathDisplay.cpp )
target_link_libraries(pathDisplay ${catkin_LIBRARIES})

add_executable(joy_emu joy_emu.cpp)
target_link_libraries(joy_emu ${catkin_LIBRARIES})

add_dependencies(pathDisplay quadrivio_msgs_generate_messages_cpp)
