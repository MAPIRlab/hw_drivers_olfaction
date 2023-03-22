cmake_minimum_required(VERSION 2.8.3)
project(mapir_nose)
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    olfaction_msgs
    #message_generation
    )

## Declare ROS messages and services
#add_message_files(FILES Sensor.msg SensorArray.msg)

## Generate added messages and services
#generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package(
	INCLUDE_DIRS include
)


###########
## Build ##
###########
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(mapir_nose src/mapir_nose.cpp)
## Necessary to avoid error at compilation
add_dependencies(mapir_nose ${olfaction_msgs_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(mapir_nose ${catkin_LIBRARIES})
#add_dependencies(mapir_nose mapir_nose_generate_messages_cpp)

