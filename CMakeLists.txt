#
# Software License Agreement (BSD License)
#
#  Copyright (c) 2013-, Stefano Michieletto
#
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   # Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   # Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   # Neither the name of the copyright holder(s) nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#
# CMakeLists.txt
# Authors:	Stefano Michieletto [stefano.michieletto@dei.unipd.it]
#			Andrea Bisson [andreabisson@hotmail.it]
#
#

cmake_minimum_required(VERSION 2.8.3)
project(nao_gazebo_plugin)

set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_BUILD_TYPE Release)


find_package(catkin REQUIRED COMPONENTS roscpp gazebo_ros std_msgs trajectory_msgs genmsg message_generation simple_message tf urdf)

#build using LINUX SOCKETS libraries
add_definitions(-DLINUXSOCKETS=1)  

# Generate added messages and services
generate_messages(DEPENDENCIES trajectory_msgs std_msgs)

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
