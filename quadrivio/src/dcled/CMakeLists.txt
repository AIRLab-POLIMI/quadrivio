cmake_minimum_required(VERSION 2.8.3)
project(dcled)

find_package(catkin REQUIRED COMPONENTS roscpp )

set(CMAKE_CXX_FLAGS "-g ${OTHERS_DEFINE}")

add_executable(dcled dcled.cpp )

target_link_libraries(dcled
  ${catkin_LIBRARIES}
)

add_dependencies(dcled quadrivio_msgs_generate_messages_cpp)
