cmake_minimum_required(VERSION 2.8.3)
project(fastPredictor)

find_package(catkin REQUIRED COMPONENTS roscpp tf)

set(CMAKE_CXX_FLAGS "-g -std=c++11 ${OTHERS_DEFINE}")

add_executable(fastPredictor fastPredictor.cpp Interpolator.cpp)

target_link_libraries(fastPredictor gsl gslcblas ${catkin_LIBRARIES})

add_dependencies(fastPredictor quadrivio_msgs_generate_messages_cpp)
