cmake_minimum_required(VERSION 3.11)
project(falcon_tdlas)


##################
set(CMAKE_BUILD_TYPE "None") 
set(CMAKE_CXX_FLAGS "-std=c++17 -g -O2 ${CMAKE_CXX_FLAGS}")



##################
find_package(catkin REQUIRED COMPONENTS
  olfaction_msgs
  roscpp
  serial
)

catkin_package()


##################
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${serial_INCLUDE_DIRS}
)

set(SOURCES_FILES 
	src/falcon.cpp
)


##################
add_executable(falcon_tdlas  ${SOURCES_FILES})
add_dependencies(falcon_tdlas ${olfaction_msgs_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(falcon_tdlas 
  ${catkin_LIBRARIES}
)