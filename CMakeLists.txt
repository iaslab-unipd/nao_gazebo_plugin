cmake_minimum_required(VERSION 2.8.3)
project(nao_gazebo_plugin)

set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_BUILD_TYPE Release)


find_package(catkin REQUIRED COMPONENTS roscpp gazebo_ros std_msgs trajectory_msgs genmsg message_generation tf urdf)

#build using LINUX SOCKETS libraries
add_definitions(-DLINUXSOCKETS=1)  

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})


#add Gazebo's plugin as dynamic library 
add_library(${PROJECT_NAME} SHARED 
	src/NaoGazeboPlugin.cpp 
	src/RobotMovementPID.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_libraries})


catkin_package(DEPENDS roscpp gazebo_ros message_runtime std_msgs trajectory_msgs genmsg)
