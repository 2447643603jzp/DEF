cmake_minimum_required(VERSION 2.8.3)
project(plan_env)


set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
add_compile_options(-std=c++14)
# # set(OpenCV_DIR "~/opencv-4.5.1/build")
# find_package(OpenCV 4.2 REQUIRED)
## Compile as C++14, supported in ROS Kinetic and newer
# add_compile_options(-std=c++14)

# set(OpenCV_DIR "~/opencv-4.5.1/build")
find_package(OpenCV 4.2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL  REQUIRED)
find_package(InferenceEngine REQUIRED)
find_package(ngraph REQUIRED)
# find_package(CGAL REQUIRED)
# include(${CGAL_USE_FILE})
# find_package(decomp_util REQUIRED)
# include_directories(${DECOMP_UTIL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
find_package(GLPK QUIET)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  cv_bridge
  message_filters
  detector
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES plan_env
 CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${GLPK_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_library( plan_env 
    src/sdf_map.cpp 
    src/obj_predictor.cpp 
    src/edt_environment.cpp 
    src/raycast.cpp
    )
target_link_libraries( plan_env
    ${catkin_LIBRARIES} 
    ${PCL_LIBRARIES}
    ${InferenceEngine_LIBRARIES}
    ${NGRAPH_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${CGAL_LIBRARIES}
    ${Eigen3_LIBRARIES}
    )  

add_executable(obj_generator
    src/obj_generator.cpp 
    )
target_link_libraries(obj_generator 
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    )



add_executable(plan_env_test  
src/test/plan_env_test.cpp 
src/sdf_map.cpp 
src/obj_predictor.cpp 
src/edt_environment.cpp 
src/raycast.cpp)
target_link_libraries(plan_env_test 
${catkin_LIBRARIES} 
${PCL_LIBRARIES}
${InferenceEngine_LIBRARIES}
${NGRAPH_LIBRARIES}
${OpenCV_LIBRARIES}
${CGAL_LIBRARIES}
${Eigen3_LIBRARIES}
)
