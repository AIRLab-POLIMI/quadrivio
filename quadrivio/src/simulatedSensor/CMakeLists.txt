cmake_minimum_required(VERSION 2.8.3)
project(simulatedSensor)

find_package(catkin REQUIRED COMPONENTS roscpp tf rosgraph_msgs)

set(CMAKE_CXX_FLAGS "-g ${OTHERS_DEFINE} -std=c++11")

add_executable(simulatedGPS simulatedGPS.cpp)
add_executable(simulatedMagnetometer simulatedMagnetometer.cpp)
add_executable(simulatedImu simulatedImu.cpp)
add_executable(simulatedInputs simulatedInputs.cpp)
add_executable(realPose realPose.cpp)
add_executable(clock clock.cpp)

target_link_libraries(simulatedGPS ${catkin_LIBRARIES} )
target_link_libraries(simulatedMagnetometer ${catkin_LIBRARIES} )
target_link_libraries(simulatedImu ${catkin_LIBRARIES} )
target_link_libraries(simulatedInputs ${catkin_LIBRARIES} )
target_link_libraries(realPose ${catkin_LIBRARIES} )
target_link_libraries(clock ${catkin_LIBRARIES} )

add_dependencies(simulatedInputs quadrivio_msgs_generate_messages_cpp)
