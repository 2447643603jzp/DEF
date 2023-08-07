

// #include <fstream>
#include <plan_manage/planner_manager.h>
#include <thread>
using namespace fast_planner;
namespace def_planner {

// SECTION interfaces for setup and query

DEFPlannerManager::DEFPlannerManager() {}

DEFPlannerManager::~DEFPlannerManager() { std::cout << "des manager" << std::endl; }

void DEFPlannerManager::initPlanModules(ros::NodeHandle& nh) {
  /* read algorithm parameters */

  nh.param("manager/max_vel", pp_.max_vel_, -1.0);
  nh.param("manager/max_acc", pp_.max_acc_, -1.0);
  nh.param("manager/max_jerk", pp_.max_jerk_, -1.0);
  std::cout<<"pp_.max_vel_:"<<pp_.max_vel_<<std::endl;
  nh.param("manager/ts", this->ts, -1.0);

  nh.param("manager/use_rsastar_path", use_rsastar_path, false);
  nh.param("manager/use_optimization", use_optimization, false);


  box_detector_.reset(new BoxDetector);
  box_detector_->init(nh);

  local_data_.traj_id_ = 0;             // 当然还有很关键的本地轨迹
  // std::cout<<"a"<<std::endl;
  sdf_map_.reset(new SDFMap);
  sdf_map_->initMap(nh);
  edt_environment_.reset(new EDTEnvironment);
  edt_environment_->setMap(sdf_map_);
  edt_environment_->sdf_map_->setDetect(box_detector_);
  // this->edt_environment_->sdf_map_-
  // std::cout<<"b"<<std::endl;/
// 
  // cv::Mat image = cv::imread("/home/jzp/Pictures/1.png");
  // cv::namedWindow("1",cv::WINDOW_AUTOSIZE);
  // cv::imshow("1",image);
  // cv::waitKey(0);
  // std::cout<<"c"<<std::endl;

  if (use_rsastar_path) {
    // std::cout<<"d"<<std::endl;
    // rrt_path_finder_.reset(new rrtSearch<N>);
    rsastar_path_finder_.reset(new RSAstar);
    rsastar_path_finder_->setParam(nh);
    rsastar_path_finder_->setEnvironment(edt_environment_);   // 优化的值
    rsastar_path_finder_->init();    // 初始化设置考试的值.
    rsastar_path_finder_->setBoxDetector(box_detector_);
    // std::cout<<"e"<<std::endl;
  }

  if (use_optimization) {
    // std::cout<<"k"<<std::endl;
    bspline_optimizer_.reset(new BsplineOptimizer);     // b样条曲线优化
    std::cout<<"k1"<<std::endl;
    bspline_optimizer_->setParam(nh);
    std::cout<<"k2"<<std::endl;
    bspline_optimizer_->setEnvironment(edt_environment_);
    std::cout<<"k4"<<std::endl;
    // bspline_optimizer_->setBoxDetector(box_detector_);
  }

  separator_solver_ = new separator::Separator();
  std::cout<<"g"<<std::endl;

}


std::pair<bool,int> DEFPlannerManager::checkTrajCollision() {

  LocalTrajData *info = &this->local_data_;
  constexpr double time_step = 0.01;
  double t_cur = (ros::Time::now() - info->start_time_).toSec();
  double t_2_3 = info->duration_ * 0.95;
  int index = -1;
  for (double t = t_cur+0.3; t < info->duration_; t += time_step)
  {
      if (t >= t_2_3) // If t_cur < t_2_3, only the first 2/3 partition of the trajectory is considered valid and will get checked.
        //  return std::make_pair(false,index);
        break;
      if (this->edt_environment_->sdf_map_->getInflateOccupancy(info->position_traj_.evaluateDeBoorT(t)) == 1)
      {
        index = floor(t/0.3);
        return std::make_pair(true,index);
      }

  }
  // std::cout<<"check3:"<<std::endl;

  if(box_detector_->getallnumtrack()==0)
  {
    return std::make_pair(false,index);
  }


  Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
  // std::cout<<"check4:"<<pos_pts.cols()<<std::endl;
  // std::cout<<"box_detector_->getallnumtrack():"<<box_detector_->getallnumtrack()<<std::endl;
  Eigen::Vector3d n_i;
  double d_i;
  // for (double t = t_cur+0.3; t < info->duration_; t += 0.15)
  for (double t = t_cur+0.3; t <t_cur+0.9 ; t += 0.25)
  {
      if(t> info->duration_)
      {
        continue;
      }
      if ( t >= t_2_3) // If t_cur < t_2_3, only the first 2/3 partition of the trajectory is considered valid and will get checked.
         return std::make_pair(false,index);
      // std::cout<<"t:"<<t<<std::endl;
      // std::cout<<"t:"<<info->start_time_.toSec()<<std::endl;
      // t_cur < t_2_3 && 
      int temp_index = floor(t/0.3);
      if((temp_index+3)>=pos_pts.cols())
         return std::make_pair(false,index);
      // std::cout<<"index:"<<temp_index<<std::endl;

      Eigen::Matrix<double, 3, 4> controlpoint;
      // std::cout<<"check4:"<<std::endl;
      controlpoint.col(0) = pos_pts.col(temp_index);
      controlpoint.col(1) = pos_pts.col(temp_index+1);
      controlpoint.col(2) = pos_pts.col(temp_index+2);
      controlpoint.col(3) = pos_pts.col(temp_index+3);
      // std::cout<<"check5:"<<std::endl;
      // bool replan_now = box_detector_->checkReplan1(controlpoint.transpose());
      // if(replan_now == true)
      // {
      //   return std::make_pair(true,temp_index);
      // }
      bool col = false;
      for(int k=0;k<1;k++)
      {
        // ConvexHullsOfCurve_Std hull_curve =  box_detector_->getIntervalHull(1,ros::Time::now().toSec(),this->ts);
        ConvexHullsOfCurve_Std hull_curve =  box_detector_->getIntervalHull(1,info->start_time_.toSec()+(temp_index+k)*this->ts,this->ts);
        for(int j =0;j<hull_curve.size();j++)
        {
          bool satisfies_LP = separator_solver_->solveModel(n_i, d_i, controlpoint, hull_curve[j]); // 第几个障碍物的这一段
          if(satisfies_LP)
          {
            col =true;
            // index = temp_index;
            // std::cout<<"unsatified:"<<std::endl;
            // return std::make_pair(true,index);
          }
        }
      }
      if(col==false)
      {
        index = temp_index;
        std::cout<<"unsatified:"<<std::endl;
        return std::make_pair(true,index);
      }
  }

  return std::make_pair(false,-1);
}

bool DEFPlannerManager::RSAStardynamicReplan(Eigen::Matrix<double, 3, 3> init_control,Eigen::Vector3d start_pt, 
Eigen::Vector3d end_pt,double time_start,Eigen::Vector3d odom_pos) {

  std::cout << "[rsastar replan]: -----------------------" << std::endl;
  cout <<  end_pt.transpose() << ", " <<  endl;

  if ((start_pt - end_pt).norm() < 0.2) {
    cout << "Close goal" << endl;
    return false;
  }  // 已经接近了控制点的值.
  std::cout<<"time_start:"<<time_start<<std::endl;

  // this->time_start_ = time_start;
  this->time_start_ = ros::Time::now().toSec()+0.4;

  std::cout<<"33"<<std::endl;
  /*路径规划算法*/
  rsastar_path_finder_->reset(odom_pos);
  std::cout<<"55"<<std::endl;

  auto start_time = std::chrono::high_resolution_clock::now();
  int status = rsastar_path_finder_->search(init_control, end_pt,this->time_start_); // 得到了估计的这一个方面的值。
  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
  std::cout << "执行时间search: " << duration.count() << " 毫秒" << std::endl;
  // std::cout<<status<<":status"<<std::endl;
  // std::cout<<"66"<<std::endl;
  if (status == RSAstar::NO_PATH) {
    // cout << "[kino replan]: rsastar path searching fail!" << endl;
    // // retry searching with discontinuous initial state
    // rrt_path_finder_->reset();
    // status = rrt_path_finder_->search( init_control, end_pt,time_start); // 再重新尝试一遍。

    // if (status == RSAstar::NO_PATH) {
    //   cout << "[kino replan]: Can't find path." << endl;
    //   return false;
    // } else {
    //   cout << "[kino replan]: retry search success." << endl;
    // }
    std::cout<<"no_path"<<std::endl;
    return false;

  } 
  else 
  {
    cout << "[kino replan]: rsastar path searching success." << endl;
  }  
  // std::cout<<"status:"<<status<<std::endl;
  // std::cout<<"77"<<std::endl;
  Eigen::MatrixXd path_pos = rsastar_path_finder_->getctrlpoint();
  // std::cout<<"88"<<std::endl;
  Eigen::MatrixXd ctrl_pts = path_pos;
  Eigen::MatrixXd ctrl_pts1 = ctrl_pts;
  std::cout<<"sfaaadada:"<<path_pos.cols()<<std::endl;

  // for(int i =0 ;i<path_pos.size();i++)
  // {
  //   ctrl_pts.col(i) = path_pos[i];
  // }

  std::vector<double>  nd_result =  rsastar_path_finder_->getndresult();
  std::vector<std::vector<Eigen::Vector3d>> positionss = rsastar_path_finder_->getpositionss();
  // std::cout<<"99"<<std::endl;
  // std::cout<<use_optimization<<std::endl;
  if(use_optimization)
  {
    // std::cout<<"991"<<std::endl;
    // std::cout<<"sfaaadada:2"<<std::endl;
    ConvexHullsOfCurves_Std hulls_curve = rsastar_path_finder_->gethulls();
    // std::cout<<"992"<<std::endl;
    // bspline trajectory optimization
    Eigen::MatrixXd tem_ctrl_pts;
    // std::cout<<"993"<<std::endl;
    auto start_time = std::chrono::high_resolution_clock::now();
    tem_ctrl_pts = bspline_optimizer_->BsplineOptimizeTraj(ctrl_pts.transpose(), ts, hulls_curve, 1, 1,this->time_start_,nd_result,positionss);  //得到优化后的控制点。优化10个控制值
    // std::cout<<"994"<<std::endl;
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    std::cout << "执行时间opt: " << duration.count() << " 毫秒" << std::endl;

    ctrl_pts = tem_ctrl_pts.transpose();
    // std::cout<<"sfaaadadaaaa:"<<ctrl_pts.cols()<<std::endl;
    // for(int i =0;i<ctrl_pts.cols();i++)
    // {
    //   std::cout<<"ctrl_pts:"<<ctrl_pts.col(i)<<std::endl;
    // }
    // std::cout<<"995"<<std::endl;
  }
  rsastar_path_finder_->clearNodeMap();
  // std::cout<<"1010"<<std::endl;
  local_data_.start_time_.fromSec(time_start_-1.0);
  // local_data_.start_time_ = ros::Time::now();

  UniformBspline pos = UniformBspline(ctrl_pts, 3, this->ts);
  UniformBspline pos1 = UniformBspline(ctrl_pts1, 3, this->ts);
  double to = pos.getTimeSum();
  // std::cout<<"1111"<<std::endl;
  pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_,0.05);
  // std::cout<<"1212"<<std::endl;
  // bool feasible = pos.checkFeasibility(1.1, false);

  // t_opt = (ros::Time::now() - t1).toSec();
  // // iterative time adjustment  // 时间调整。
  // t1                    = ros::Time::now(); 
  local_data_.position_traj_ = pos;  
  local_data_.position_traj_1 = pos1;
  // std::cout<<"1313"<<std::endl;
  updateTrajInfo();
  // std::cout<<"1414"<<std::endl;
  return true;

}


void DEFPlannerManager::updateTrajInfo() {
  // std::cout<<"1515"<<std::endl;
  local_data_.velocity_traj_     = local_data_.position_traj_.getDerivative();
  // std::cout<<"1616"<<std::endl;
  local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();
  // std::cout<<"1717"<<std::endl;
  // Eigen::VectorXd pt_tem         = local_data_.position_traj_.evaluateDeBoorT(0.0); 
  // std::cout<<"pt_tem:"<<local_data_.position_traj_.evaluateDeBoorT(0.0) <<std::endl; 
  // Eigen::Vector3d pos            = Eigen::Vector3d(pt_tem(0),pt_tem(1),pt_tem(2)); 
  local_data_.start_pos_         = local_data_.position_traj_.evaluateDeBoorT(0.0);
  // std::cout<<"1818"<<std::endl;
  local_data_.duration_          = local_data_.position_traj_.getTimeSum();
  // std::cout<<"1919"<<std::endl;
  local_data_.traj_id_ += 1;
}

void DEFPlannerManager::planYaw(const Eigen::Vector3d& start_yaw) {
  ROS_INFO("plan yaw");
  auto t1 = ros::Time::now();
  // calculate waypoints of heading

  auto&  pos      = local_data_.position_traj_; 
  double duration = pos.getTimeSum();

  double dt_yaw  = 0.3;
  int    seg_num = ceil(duration / dt_yaw);
  dt_yaw         = duration / seg_num;

  const double            forward_t = 2.0;
  double                  last_yaw  = start_yaw(0);
  vector<Eigen::Vector3d> waypts;
  vector<int>             waypt_idx;

  // seg_num -> seg_num - 1 points for constraint excluding the boundary states

  for (int i = 0; i < seg_num; ++i) {
    double          tc = i * dt_yaw;
    Eigen::Vector3d pc = pos.evaluateDeBoorT(tc);
    double          tf = min(duration, tc + forward_t);
    Eigen::Vector3d pf = pos.evaluateDeBoorT(tf);
    Eigen::Vector3d pd = pf - pc;

    Eigen::Vector3d waypt;
    if (pd.norm() > 1e-6) {
      waypt(0) = atan2(pd(1), pd(0));
      waypt(1) = waypt(2) = 0.0;
      calcNextYaw(last_yaw, waypt(0));
    } else {
      waypt = waypts.back();
    }
    waypts.push_back(waypt);
    waypt_idx.push_back(i);
  }

  // calculate initial control points with boundary state constraints

  Eigen::MatrixXd yaw(seg_num + 3, 1);
  yaw.setZero();

  Eigen::Matrix3d states2pts;
  states2pts << 1.0, -dt_yaw, (1 / 3.0) * dt_yaw * dt_yaw, 1.0, 0.0, -(1 / 6.0) * dt_yaw * dt_yaw, 1.0,
      dt_yaw, (1 / 3.0) * dt_yaw * dt_yaw;
  yaw.block(0, 0, 3, 1) = states2pts * start_yaw;

  Eigen::Vector3d end_v = local_data_.velocity_traj_.evaluateDeBoorT(duration - 0.1);
  Eigen::Vector3d end_yaw(atan2(end_v(1), end_v(0)), 0, 0);
  calcNextYaw(last_yaw, end_yaw(0));
  yaw.block(seg_num, 0, 3, 1) = states2pts * end_yaw;
  // std::cout<<"endnn"<<std::endl;
  // // solve
  // bspline_optimizer_[1]->setWaypoints(waypts, waypt_idx);
  // int cost_func = BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::WAYPOINTS;
  // yaw           = bspline_optimizer_[1]->BsplineOptimizeTraj(yaw, dt_yaw, cost_func, 1, 1);

  // update traj info
  Eigen::MatrixXd yaw_tem = yaw.transpose();
  local_data_.yaw_traj_.setUniformBspline(yaw_tem, 3, dt_yaw); //
  // std::cout<<"endnn2"<<std::endl;
  local_data_.yawdot_traj_    = local_data_.yaw_traj_.getDerivative();
  // std::cout<<"endn33"<<std::endl;
  local_data_.yawdotdot_traj_ = local_data_.yawdot_traj_.getDerivative();
  // std::cout<<"endnn1"<<std::endl;

  vector<double> path_yaw;
  for (int i = 0; i < waypts.size(); ++i) path_yaw.push_back(waypts[i][0]);
  plan_data_.path_yaw_    = path_yaw;
  plan_data_.dt_yaw_      = dt_yaw;
  plan_data_.dt_yaw_path_ = dt_yaw;

  std::cout << "plan heading: " << (ros::Time::now() - t1).toSec() << std::endl;
}

void DEFPlannerManager::calcNextYaw(const double& last_yaw, double& yaw) {
  // round yaw to [-PI, PI]

  double round_last = last_yaw;

  while (round_last < -M_PI) {
    round_last += 2 * M_PI;
  }
  while (round_last > M_PI) {
    round_last -= 2 * M_PI;
  }

  double diff = yaw - round_last;

  if (fabs(diff) <= M_PI) {
    yaw = last_yaw + diff;
  } else if (diff > M_PI) {
    yaw = last_yaw + diff - 2 * M_PI;
  } else if (diff < -M_PI) {
    yaw = last_yaw + diff + 2 * M_PI;
  }
}
}

