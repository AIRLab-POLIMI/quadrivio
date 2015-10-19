cmake_minimum_required(VERSION 2.8.3)
project(joypad)

find_package(catkin REQUIRED COMPONENTS roscpp heartbeat)

set(CMAKE_CXX_FLAGS "-g ${OTHERS_DEFINE}")

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(joypad_translate joypad_translate.cpp )

target_link_libraries(joypad_translate
  ${catkin_LIBRARIES}
)

add_dependencies(joypad_translate quadrivio_msgs_generate_messages_cpp)
