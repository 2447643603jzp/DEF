cmake_minimum_required(VERSION 3.0.2)
project(detector)

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

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  message_filters
  roscpp
  rospy
  std_msgs
  geometry_msgs
  visualization_msgs
#   decomp_util
#   decomp_ros_utils
  roslib
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES detector
 CATKIN_DEPENDS cv_bridge message_filters roscpp rospy std_msgs roslib geometry_msgs visualization_msgs 
 DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories( 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

add_library( detector 
    src/Boxdetector.cpp 
    src/Hungarian.cpp        # 好像他这里就是建造了一个库
    src/nanodet_openvino.cpp
    src/cgal_utils.cpp
    )
target_link_libraries( detector
    ${catkin_LIBRARIES} 
    ${PCL_LIBRARIES}
    ${InferenceEngine_LIBRARIES}
    ${NGRAPH_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${CGAL_LIBRARIES}
    ${Eigen3_LIBRARIES}
    ) 


add_executable(box_detector   
    src/test/detector_main_node.cpp  
    src/Boxdetector.cpp 
    src/Hungarian.cpp       
    src/nanodet_openvino.cpp
    src/cgal_utils.cpp)
target_link_libraries(box_detector 
    ${catkin_LIBRARIES} 
    ${PCL_LIBRARIES}
    ${InferenceEngine_LIBRARIES}
    ${NGRAPH_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${CGAL_LIBRARIES}
    ${Eigen3_LIBRARIES}
    )
    



