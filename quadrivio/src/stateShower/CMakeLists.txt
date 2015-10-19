cmake_minimum_required(VERSION 2.8.3)
project(stateShower)

find_package(catkin REQUIRED COMPONENTS roscpp heartbeat)

set(CMAKE_CXX_FLAGS "-g ${OTHERS_DEFINE}")

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(stateShower stateShower.cpp )

target_link_libraries(stateShower
  ${catkin_LIBRARIES}
)

add_dependencies(stateShower quadrivio_msgs_generate_messages_cpp)
