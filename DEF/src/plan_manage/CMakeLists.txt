cmake_minimum_required(VERSION 3.0.2)
project(plan_manage)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

# set(OpenCV_DIR "~/opencv-4.5.1/build")
find_package(OpenCV 4.2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(InferenceEngine REQUIRED)
find_package(ngraph REQUIRED)
# find_package(CGAL REQUIRED)
# include(${CGAL_USE_FILE})
# find_package(decomp_util REQUIRED)
# include_directories(${DECOMP_UTIL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
find_package(GLPK QUIET)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  bspline
  bspline_opt
  cv_bridge
  detector
  geometry_msgs
  message_generation
  nav_msgs
  plan_env
  plan_search
  roscpp
  roslib
  rospy
  std_msgs
  traj_utils
)



# Generate messages in the 'msg' folder
add_message_files(
  FILES
  Bspline.msg
  )

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES plan_manage
 CATKIN_DEPENDS bspline bspline_opt cv_bridge detector geometry_msgs message_generation nav_msgs plan_env plan_search roscpp roslib rospy std_msgs 
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories( 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${GLPK_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})


add_executable(def_planner_node
  src/def_planner_node.cpp 
  src/RSAStar_replan_fsm.cpp
  src/planner_manager.cpp
  )
target_link_libraries(def_planner_node 
${catkin_LIBRARIES} 
${PCL_LIBRARIES}
${InferenceEngine_LIBRARIES}
${NGRAPH_LIBRARIES}
${OpenCV_LIBRARIES}
${GLPK_LIBRARIES}
-lglpk

)

add_executable(traj_server src/traj_server.cpp)
target_link_libraries(traj_server ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(traj_server ${${PROJECT_NAME}_EXPORTED_TARGETS} )
## Specify additional locations of header files
## Your package locations should be listed before other locations


