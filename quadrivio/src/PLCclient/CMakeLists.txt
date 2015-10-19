cmake_minimum_required(VERSION 2.8.3)
project(PLCclient)

find_package(catkin REQUIRED COMPONENTS roscpp heartbeat)

set(CMAKE_CXX_FLAGS "-g -std=c++11 ${OTHERS_DEFINE}")

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(PLCclient CharCircularBuffer.cpp 
                         StringUtils.cpp 
                         MsgFromQuad.cpp 
                         MsgToQuad.cpp 
                         SerialCommunication.cpp
                         tcp_ip_client.cpp
                         PLCclient.cpp )

target_link_libraries(PLCclient
  ${catkin_LIBRARIES}
)

add_dependencies(PLCclient quadrivio_msgs_generate_messages_cpp)
