cmake_minimum_required(VERSION 3.0.2)
project(bspline)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++14)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
roscpp
rospy
std_msgs
visualization_msgs
# plan_env
cv_bridge
)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES bspline
 CATKIN_DEPENDS
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  SYSTEM 
  include 
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
)


# set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS} -O3 -Wall")

add_library( bspline
    src/uniform_bspline.cpp 
    )
target_link_libraries( bspline
    ${catkin_LIBRARIES} 
    ${Eigen3_LIBRARIES} 
    ${PCL_LIBRARIES}
    )  


  
    


