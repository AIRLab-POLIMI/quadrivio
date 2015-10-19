cmake_minimum_required(VERSION 2.8.3)
project(quadrivio_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs geometry_msgs nav_msgs)

FILE(GLOB_RECURSE quadrivio_message_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}/msg *.msg)

message("found the following message files: " ${quadrivio_message_files})

add_message_files(
  FILES
  ${quadrivio_message_files}
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs nav_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs geometry_msgs nav_msgs)
