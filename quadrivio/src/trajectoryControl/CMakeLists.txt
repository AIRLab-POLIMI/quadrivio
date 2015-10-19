cmake_minimum_required(VERSION 2.8.3)
project(trajectoryControl)

find_package(catkin REQUIRED COMPONENTS roscpp nav_msgs heartbeat tf)

set(CMAKE_CXX_FLAGS "-g ${OTHERS_DEFINE}")

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(trajectoryControl main.cpp TrajectoryControl.cpp)

target_link_libraries(trajectoryControl m gsl gslcblas
  ${catkin_LIBRARIES}
)
add_dependencies(trajectoryControl quadrivio_msgs_generate_messages_cpp)
