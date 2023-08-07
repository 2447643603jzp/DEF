cmake_minimum_required(VERSION 3.0.2)
project(bspline_opt)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(GLPK QUIET)
# set(OpenCV_DIR "~/opencv-4.5.1/build")
find_package(OpenCV 4.2 REQUIRED)
find_package(InferenceEngine REQUIRED)
find_package(ngraph REQUIRED)
# find_package(CGAL REQUIRED)
# include(${CGAL_USE_FILE})
# find_package(decomp_util REQUIRED)
# include_directories(${DECOMP_UTIL_INCLUDE_DIRS})

find_package(NLopt REQUIRED)
set(NLopt_INCLUDE_DIRS ${NLOPT_INCLUDE_DIR})

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  # nlopt
  plan_env
  roscpp
  rospy
  std_msgs
  visualization_msgs
  detector
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES bspline_opt
  CATKIN_DEPENDS cv_bridge  plan_env roscpp rospy std_msgs visualization_msgs
  DEPENDS system_lib
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
  ${GLPK_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${NLOPT_INCLUDE_DIR}
)

set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS} -O3 -Wall")

add_library( bspline_opt 
    src/bspline_optimizer.cpp 
    )
target_link_libraries( bspline_opt
    ${catkin_LIBRARIES} 
    ${Eigen3_LIBRARIES} 
    ${PCL_LIBRARIES}
    ${InferenceEngine_LIBRARIES}
    ${NGRAPH_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${GLPK_LIBRARIES}
    ${NLOPT_LIBRARIES}
    # /usr/local/lib/libnlopt.so
    )


