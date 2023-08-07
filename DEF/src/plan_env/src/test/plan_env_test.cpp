#include <ros/ros.h>
#include <plan_env/edt_environment.h>
#include <plan_env/linear_obj_model.hpp>
#include <plan_env/obj_predictor.h>
#include <plan_env/raycast.h>
#include <plan_env/sdf_map.h>
#include <plan_env/polynomial_traj.hpp>
int main(int argc, char** argv){
	ros::init(argc, argv, "sdf_node");
	ros::NodeHandle nh("~");
    
	// std::cout<<"start1"<<std::endl;
	SDFMap m;
	// std::cout<<"start2"<<std::endl;
	m.initMap(nh);
	// std::cout<<"start3"<<std::endl;

    ros::Duration(1.0).sleep();
	ros::spin();

	return 0;
}