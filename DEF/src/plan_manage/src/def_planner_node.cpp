
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <plan_manage/RSAStar_replan_fsm.h>

using namespace def_planner;

int main(int argc, char** argv) {
  ros::init(argc, argv, "def_planner_node");
  ros::NodeHandle nh("~");

  RSAStarReplanFSM RSAStar_replan;
  RSAStar_replan.init(nh);

  ros::Duration(1.0).sleep();
  ros::spin();
  // ros::MultiThreadedSpinner spinner(8);
  // spinner.spin();

  return 0;
}
