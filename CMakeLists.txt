cmake_minimum_required(VERSION 2.8.3)
project(interpreter_xyz_rpy)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)


catkin_package(
    CATKIN_DEPENDS
    roscpp
    roslib
    std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}

)
set (MyProject_src
          src/pub_interp.cpp
    )
add_executable(interpreter_xyz_rpy  ${MyProject_src} )

target_link_libraries(interpreter_xyz_rpy
                         ${catkin_LIBRARIES}
                         ${Boost_LIBRARIES}

 )

add_definitions(-std=c++11
                )



