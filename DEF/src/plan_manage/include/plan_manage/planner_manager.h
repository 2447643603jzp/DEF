#ifndef _PLANNER_MANAGER_H_
#define _PLANNER_MANAGER_H_

#include <bspline_opt/bspline_optimizer.h>
#include <bspline/uniform_bspline.h>
// #include <plan_search/rrtSearch.h>
#include <plan_search/rsastar.h>
#include <plan_env/edt_environment.h>
#include <plan_manage/plan_container.hpp>
#include <ros/ros.h>
#include <plan_search/separator.hpp>
#include <glpk.h>
#include <ros/package.h>
#include <detector/Boxdetector.h>
#include <chrono>

using namespace fast_planner;
namespace def_planner {

class DEFPlannerManager {
  // SECTION stable
public:
  DEFPlannerManager();
  ~DEFPlannerManager();

  /* main planning interface */

  PlanParameters pp_;
  LocalTrajData local_data_;
  GlobalTrajData global_data_;
  MidPlanData plan_data_;
  EDTEnvironment::Ptr edt_environment_;
  BoxDetector::Ptr box_detector_;

private:
  /* main planning algorithms & modules */
  static constexpr std::size_t N = 3;
  SDFMap::Ptr sdf_map_;
  // BoxDetector::Ptr box_detector_;

  double time_start_;

  double ts;
  bool use_rsastar_path, use_optimization;
  std::unique_ptr<RSAstar> rsastar_path_finder_;
  // std::unique_ptr<rrtSearch<N>> rrt_path_finder_;
  BsplineOptimizer::Ptr bspline_optimizer_;
  // std::unique_ptr<BsplineOptimizer> bspline_optimizers_;
  separator::Separator* separator_solver_;
  
public:

  void updateTrajInfo();
  void initPlanModules(ros::NodeHandle& nh);

  // heading planning
  void calcNextYaw(const double& last_yaw, double& yaw);
  std::pair<bool,int> checkTrajCollision();
  void planYaw(const Eigen::Vector3d& start_yaw);
  bool RSAStardynamicReplan(Eigen::Matrix<double, 3, 3> init_control,Eigen::Vector3d start_pt,Eigen::Vector3d end_pt,double time_start,Eigen::Vector3d odom_pos);

  typedef unique_ptr<DEFPlannerManager> Ptr;
  
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // !SECTION
};
}  // namespace fast_planner

#endif
