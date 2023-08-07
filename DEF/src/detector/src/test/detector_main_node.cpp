#include <ros/ros.h>
#include <detector/Boxdetector.h>
int main(int argc, char** argv){
	ros::init(argc, argv, "dyanmic_map_node");
	ros::NodeHandle nh("~");;
    
	// std::cout<<"start1"<<std::endl;
	BoxDetector m;
	// std::cout<<"start2"<<std::endl;
	m.init(nh);
	// std::cout<<"start3"<<std::endl;

    ros::Duration(1.0).sleep();
	ros::spin();

	return 0;
}