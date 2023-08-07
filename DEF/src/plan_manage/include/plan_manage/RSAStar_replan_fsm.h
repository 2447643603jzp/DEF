
#ifndef _ASTAR_REPLAN_FSM_H_
#define _ASTAR_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <vector>
#include <visualization_msgs/Marker.h>

#include <bspline_opt/bspline_optimizer.h>
#include <plan_search/rsastar.h>
// #include <plan_search/rrtSearch.h>

#include <plan_env/edt_environment.h>
#include <plan_env/obj_predictor.h>
#include <plan_env/sdf_map.h>
#include <plan_manage/Bspline.h>
#include <plan_manage/planner_manager.h>
#include <traj_utils/planning_visualization.h>
#include <plan_search/separator.hpp>
#include <glpk.h>

using std::vector;

namespace def_planner {

class RSAStarReplanFSM {

private:
  /* ---------- flag ---------- */
  enum FSM_EXEC_STATE { INIT, WAIT_TARGET, GEN_NEW_TRAJ, REPLAN_TRAJ, EXEC_TRAJ, REPLAN_NEW };
  // 只有手动设置目标点.

  /* planning utils */
  DEFPlannerManager::Ptr planner_manager_;
  PlanningVisualization::Ptr visualization_;

  /* planning data */
  bool trigger_, have_target_, have_odom_;
  FSM_EXEC_STATE exec_state_;

  Eigen::Vector3d odom_pos_, odom_vel_;  // odometry state
  Eigen::Quaterniond odom_orient_;

  Eigen::Vector3d start_pt_, start_vel_, start_acc_, start_yaw_;  // start state
  Eigen::Vector3d end_pt_, end_vel_;                              // target state

  /* ROS utils */
  ros::NodeHandle node_;
  ros::Timer exec_timer_, safety_timer_, vis_timer_, test_something_timer_;
  ros::Subscriber goal_sub_, odom_sub_;
  ros::Publisher replan_pub_, new_pub_, bspline_pub_;

  /*for continue*/
  Eigen::Matrix<double, 3, 3> init_control;   // 一行一个xyz

  double no_replan_thresh_, replan_thresh_;

  double ts;
  double time_start;
  int collision_index = -1;
  int trajectory_num = -1;
  bool now_position = false;
  int replan_count = 0;
  ros::Time time_replan;

  /* helper functions */

public:
  RSAStarReplanFSM(/* args */) {
  }
  ~RSAStarReplanFSM() {
  }
                                         // optimization; 1: new, 2: replan
  void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
  void printFSMExecState();

  /* ROS functions */
  void execFSMCallback(const ros::TimerEvent& e);
  void checkCollisionCallback(const ros::TimerEvent& e);
  void odometryCallback(const nav_msgs::OdometryConstPtr& msg);

  bool callRSAStardynamicReplan();
  void goalpointCallback(const geometry_msgs::PoseStampedPtr &msg);
  void init(ros::NodeHandle& nh);
  Eigen::Matrix<double, 3, 3> SolveInitPoint(Eigen::Vector3d p,Eigen::Vector3d v,Eigen::Vector3d a);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace def_planner

#endif