cmake_minimum_required(VERSION 3.0.2)
project(plan_search)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++14)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")



find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(catkin REQUIRED)
find_package(GLPK QUIET)

# set(OpenCV_DIR $INSTALL_PATH/install/share/OpenCV)
# set(OpenCV_DIR "~/opencv-4.5.1/build")
find_package(OpenCV 4.2 REQUIRED)
find_package(InferenceEngine REQUIRED)
find_package(ngraph REQUIRED)
# find_package(CGAL REQUIRED)
# include(${CGAL_USE_FILE})
# find_package(decomp_util REQUIRED)
# include_directories(${DECOMP_UTIL_INCLUDE_DIRS})
# find_package(GLPK QUIET)


find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  rospy
  std_msgs
  visualization_msgs
  plan_env
  detector
  # decomp_util
  # decomp_ros_utils
)

# For convenience (link to static library)
# set(GLPK_INCLUDE_DIRS ${GLPK_DIR}/install/include)
# set(GLPK_LIBS -L${GLPK_DIR}/install/lib;libglpk.a)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES plan_search
 CATKIN_DEPENDS cv_bridge roscpp rospy std_msgs visualization_msgs plan_env
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  SYSTEM 
  include 
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
  ${GLPK_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS} -O3 -Wall")

		

add_library( plan_search
    # src/rrtSearch.cpp
    src/rsastar.cpp
    src/separator_glpk.cpp
    )
target_link_libraries( plan_search
    ${catkin_LIBRARIES} 
    ${GLPK_LIBRARIES}
    ${PCL_LIBRARIES}
    ${InferenceEngine_LIBRARIES}
    ${NGRAPH_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${GLPK_LIBRARIES}
    )  

