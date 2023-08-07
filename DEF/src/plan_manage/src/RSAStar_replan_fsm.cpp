
#include <plan_manage/RSAStar_replan_fsm.h>

namespace def_planner {

void RSAStarReplanFSM::init(ros::NodeHandle& nh) {

  exec_state_  = FSM_EXEC_STATE::INIT;
  have_target_ = false;
  have_odom_   = false;

  /*  fsm param  */
  nh.param("fsm/thresh_replan", replan_thresh_, -1.0);
  nh.param("fsm/thresh_no_replan", no_replan_thresh_, -1.0);
  nh.param("manager/ts", this->ts, -1.0);

  std::cout<<"fsm/thresh_replan:"<<replan_thresh_<<std::endl;
  std::cout<<"no_replan_thresh_:"<<no_replan_thresh_<<std::endl;
  std::cout<<"manager/ts:"<<this->ts<<std::endl;

  /* initialize main modules */
  std::cout<<1<<std::endl;
  planner_manager_.reset(new DEFPlannerManager);
  planner_manager_->initPlanModules(nh);
  visualization_.reset(new PlanningVisualization(nh));
  time_replan = ros::Time::now();

  /* callback */
  exec_timer_   = nh.createTimer(ros::Duration(0.01), &RSAStarReplanFSM::execFSMCallback, this);
  safety_timer_ = nh.createTimer(ros::Duration(0.01), &RSAStarReplanFSM::checkCollisionCallback, this);

  goal_sub_= nh.subscribe("/move_base_simple/goal", 1, &RSAStarReplanFSM::goalpointCallback, this);
  odom_sub_ = nh.subscribe("/odom_world", 1, &RSAStarReplanFSM::odometryCallback, this);

  replan_pub_  = nh.advertise<std_msgs::Empty>("/planning/replan", 10);
  new_pub_     = nh.advertise<std_msgs::Empty>("/planning/new", 10);
  bspline_pub_ = nh.advertise<plan_manage::Bspline>("/planning/bspline", 10);
}

void RSAStarReplanFSM::goalpointCallback(const geometry_msgs::PoseStampedPtr &msg) {
  if (msg->pose.position.z < -0.1) return;

  cout << "Triggered!" << endl;
  trigger_ = true;
  have_target_ = true;
  // 设置目标点
  end_pt_ << msg->pose.position.x, msg->pose.position.y, 1;

  visualization_->drawGoal(end_pt_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));
  end_vel_.setZero();

  if (exec_state_ == WAIT_TARGET)
    changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
  else if (exec_state_ == EXEC_TRAJ)
    changeFSMExecState(REPLAN_TRAJ, "TRIG");    // 这两个判断也是很好的
}

void RSAStarReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr& msg) {
  odom_pos_(0) = msg->pose.pose.position.x;
  odom_pos_(1) = msg->pose.pose.position.y;
  odom_pos_(2) = msg->pose.pose.position.z;

  odom_vel_(0) = msg->twist.twist.linear.x;
  odom_vel_(1) = msg->twist.twist.linear.y;
  odom_vel_(2) = msg->twist.twist.linear.z;

  odom_orient_.w() = msg->pose.pose.orientation.w;
  odom_orient_.x() = msg->pose.pose.orientation.x;
  odom_orient_.y() = msg->pose.pose.orientation.y;
  odom_orient_.z() = msg->pose.pose.orientation.z;

  have_odom_ = true;
}

void RSAStarReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call) {
  string state_str[5] = { "INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ" };
  int    pre_s        = int(exec_state_);
  exec_state_         = new_state;
  cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
}

void RSAStarReplanFSM::printFSMExecState() {
  string state_str[5] = { "INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ" };

  cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
}

void RSAStarReplanFSM::execFSMCallback(const ros::TimerEvent& e) {
  static int fsm_num = 0;
  fsm_num++;
  if (fsm_num == 100) {
    printFSMExecState();
    if (!have_odom_) cout << "no odom." << endl;
    if (!trigger_) cout << "wait for goal." << endl;
    fsm_num = 0;
  }    

  switch (exec_state_) {
    case INIT: {
      if (!have_odom_) {
        return;
      }
      if (!trigger_) {
        return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET: {
      if (!have_target_)
        return;
      else {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case GEN_NEW_TRAJ: {             
      start_pt_  = odom_pos_;        
      start_vel_ = odom_vel_;        
      start_acc_.setZero();
      
      if((start_pt_ - end_pt_).norm() < 0.2)
      {
        std::cout<<"331bb"<<std::endl;
        have_target_ = false;
        changeFSMExecState(WAIT_TARGET, "FSM");
      }
      
      init_control.setZero();
      for (int i=0;i<3;i++)
      {
        
        init_control.row(i) = start_pt_; 
      }  
      // std::cout<<"331ll"<<std::endl;
      this->time_start = ros::Time::now().toSec();

      bool success = callRSAStardynamicReplan();
      std::cout<<"success:"<<success<<std::endl;
      if (success) {
        changeFSMExecState(EXEC_TRAJ, "FSM");
      } else {
        // have_target_ = false;
        // changeFSMExecState(WAIT_TARGET, "FSM");
        std::cout<<"here1"<<std::endl;
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case EXEC_TRAJ: {
      /* determine if need to replan */
      // std::cout<<"sdaada1"<<std::endl;
      LocalTrajData* info     = &planner_manager_->local_data_;
      ros::Time      time_now = ros::Time::now();
      double         t_cur    = (time_now - info->start_time_).toSec();

      t_cur                   = min(info->duration_, t_cur);
      
      // std::cout<<"331114"<<std::endl;
      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);
      // std::cout<<"331111"<<std::endl;
      /* && (end_pt_ - pos).norm() < 0.5 */
      if (t_cur > info->duration_ - 1e-2) { 
        have_target_ = false;
        changeFSMExecState(WAIT_TARGET, "FSM");
        return;

      } else if ((end_pt_ - pos).norm() < no_replan_thresh_) {
        // cout << "near end" << endl;
        return;

      } else if ((info->start_pos_ - pos).norm() < replan_thresh_) {
        // cout << "near start" << endl;
        return;

      } else {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
      break;
    }

    case REPLAN_TRAJ: {
      LocalTrajData* info     = &planner_manager_->local_data_;
      ros::Time      time_now = ros::Time::now();
      double         t_cur    = (time_now - info->start_time_).toSec(); 

      // if((time_now-time_replan).toSec()<0.5)  
      // {
      //   changeFSMExecState(EXEC_TRAJ, "FSM");return;}
      // else
      // {time_replan = time_now;}

      start_pt_  = odom_pos_;        
      start_vel_ = odom_vel_;       
      start_acc_.setZero();

      Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
      int index = floor(t_cur/ts);
      init_control.setZero();

      std::cout<<"33"<<std::endl;

      bool get_pose =false;

      if(collision_index == -1)
      {
        std::cout<<"oo3"<<std::endl;
        this->init_control = SolveInitPoint(odom_pos_,odom_vel_,start_acc_);  
      }
      else
      {
         if(pos_pts.cols()<(index+5))
         {
            for (int i=0;i<3;i++)
            {
              std::cout<<"pp"<<std::endl;
              init_control.row(i) = odom_pos_ ; 
              // get_pose = true;
            }
            std::cout<<"oo2"<<std::endl;
            // this->init_control = SolveInitPoint(start_pt_,start_vel_,start_acc_);
         }


        // if(collision_index - index<=4)
        // {
        //   this->init_control = SolveInitPoint(start_pt_,start_vel_,start_acc_);  
        // }
        else
        {
          if((odom_pos_-pos_pts.col(index)).norm()<0.5)
          {
            for (int i=0;i<3;i++)
            {
              std::cout<<"oo1"<<std::endl;
              init_control.row(i) = pos_pts.col(i+index+2).transpose(); 
              // get_pose = true;
            }
          }
          else
          {
            for (int i=0;i<3;i++)
            {
              std::cout<<"pppppp"<<std::endl;
              this->init_control = SolveInitPoint(odom_pos_,odom_vel_,start_acc_);
              // get_pose = true;
            }

          }
        }
         
      }

      // if(now_position == true)
      // {
      //    for (int i=0;i<3;i++)
      //   {
      //          std::cout<<"pp12"<<std::endl;
      //         init_control.row(i) = odom_pos_; // 开始的三个控制点就在同一个位置. // 开始执行的时候的三个控制点的值。
      //         get_pose = true;
      //    }
      //    now_position = false;
      // }
      // else
      // {
      // if(collision_index == -1)
      // {
      //    if(pos_pts.cols()<(index+4))
      //    {
      //       for (int i=0;i<3;i++)
      //       {
      //         std::cout<<"pp"<<std::endl;
      //         init_control.row(i) = odom_pos_; // 开始的三个控制点就在同一个位置. // 开始执行的时候的三个控制点的值。
      //         get_pose = true;
      //       }
      //    }
      //    else{
      //     if((odom_pos_-pos_pts.col(index+1)).norm()<0.5)
      //     {
      //       for (int i=0;i<3;i++)
      //       {
      //         // std::cout<<"oo"<<std::endl;
      //         init_control.row(i) = pos_pts.col(i+index+1).transpose(); // 开始的三个控制点就在同一个位置. // 开始执行的时候的三个控制点的值。
      //         get_pose = true;
      //         // std::cout<<"init_control.row(i):"<<init_control.row(i)<<std::endl;
      //       }
      //     }
      //     else
      //     {
      //       for (int i=0;i<3;i++)
      //       {
      //         std::cout<<"pppp"<<std::endl;
      //         init_control.row(i) = odom_pos_; // 开始的三个控制点就在同一个位置. // 开始执行的时候的三个控制点的值。
      //         get_pose = true;
      //       }

      //     }
      //    }
      // }
      // else{
      //   if(collision_index - index<=2)
      //   {
      //     std::cout<<"o2o"<<std::endl;
      //     changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      //     return;
      //   }
      //   else{
      //     if((odom_pos_-pos_pts.col(index+1)).norm()<0.5)
      //     {
      //       for (int i=0;i<3;i++)
      //       {
      //         // std::cout<<"oo1"<<std::endl;
      //         init_control.row(i) = pos_pts.col(i+index+1).transpose(); // 开始的三个控制点就在同一个位置. // 开始执行的时候的三个控制点的值。
      //         get_pose = true;
      //       }
      //     }
      //     else
      //     {
      //       for (int i=0;i<3;i++)
      //       {
      //         std::cout<<"pppppp"<<std::endl;
      //         init_control.row(i) = odom_pos_; // 开始的三个控制点就在同一个位置. // 开始执行的时候的三个控制点的值。
      //         get_pose = true;
      //       }

      //     }
      //   }
      // }
      // }
      // if(get_pose == false)
      // {
      //   for (int i=0;i<3;i++)
      //   {
      //     init_control.row(i) = odom_pos_;
      //   }
      // }

      this->time_start = ros::Time::now().toSec();  // 后退路径算法

      std_msgs::Empty replan_msg;
      replan_pub_.publish(replan_msg);

      bool success = callRSAStardynamicReplan();
      if (success) {
        changeFSMExecState(EXEC_TRAJ, "FSM");
      } else {
        std::cout<<"here"<<std::endl;
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }
  }
}

void RSAStarReplanFSM::checkCollisionCallback(const ros::TimerEvent& e) {
  LocalTrajData* info = &planner_manager_->local_data_;
  
  // cout << "check collision" << endl;

  // if (have_target_) {
  //   auto edt_env = planner_manager_->edt_environment_;

  //   double dist = edt_env->evaluateCoarseEDT(end_pt_, -1.0); // obstacle time
  //   // std::cout<<"dist____"<<dist<<std::endl;

  //   if (dist <= 0.3) {          // 更新终点
  //     /* try to find a max distance goal around */
  //     bool            new_goal = false;
  //     const double    dr = 0.5, dtheta = 30, dz = 0.3;
  //     double          new_x, new_y, new_z, max_dist = -1.0;
  //     Eigen::Vector3d goal;

  //     for (double r = dr; r <= 5 * dr + 1e-3; r += dr) {
  //       for (double theta = -90; theta <= 270; theta += dtheta) {
  //         for (double nz = 1 * dz; nz >= -1 * dz; nz -= dz) {

  //           new_x = end_pt_(0) + r * cos(theta / 57.3);
  //           new_y = end_pt_(1) + r * sin(theta / 57.3);
  //           new_z = end_pt_(2) + nz;

  //           Eigen::Vector3d new_pt(new_x, new_y, new_z);
  //           dist = planner_manager_->pp_.dynamic_ ?
  //               edt_env->evaluateCoarseEDT(new_pt, /* time to program start+ */ info->duration_) :
  //               edt_env->evaluateCoarseEDT(new_pt, -1.0);

  //           if (dist > max_dist) {
  //             /* reset end_pt_ */
  //             goal(0)  = new_x;
  //             goal(1)  = new_y;
  //             goal(2)  = new_z;
  //             max_dist = dist;
  //           }
  //         }
  //       }
  //     }

  //     if (max_dist > 0.3) {
  //       cout << "change goal, replan." << endl;
  //       end_pt_      = goal;
  //       have_target_ = true;
  //       end_vel_.setZero();

  //       if (exec_state_ == EXEC_TRAJ) {
  //         changeFSMExecState(REPLAN_TRAJ, "SAFETY");
  //       }

  //       visualization_->drawGoal(end_pt_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));
  //     } else {
  //       // have_target_ = false;
  //       // cout << "Goal near collision, stop." << endl;
  //       // changeFSMExecState(WAIT_TARGET, "SAFETY");
  //       cout << "goal near collision, keep retry" << endl;
  //       changeFSMExecState(REPLAN_TRAJ, "FSM");

  //       std_msgs::Empty emt;
  //       replan_pub_.publish(emt);
  //     }
  //   }
  // }
  
  auto box_detect = planner_manager_->box_detector_;
  // if(exec_state_ == FSM_EXEC_STATE::EXEC_TRAJ)
  // {
  //    bool replan_should = box_detect->checkReplan(this->odom_pos_);
  //    if(replan_should&&replan_count==0)
  //    {
  //      changeFSMExecState(REPLAN_TRAJ, "closetopeople");
  //      now_position = true;
  //      replan_count = 1;
  //      return;
  //    }
  // }

  // if(exec_state_ == FSM_EXEC_STATE::EXEC_TRAJ)
  // {
  //   int num = box_detect->getallnumtrack();
  //   if(trajectory_num == -1 )
  //   {
  //     this->trajectory_num = num;
  //   }
  //   else
  //   {
  //     if(this->trajectory_num != num)
  //     {
  //       changeFSMExecState(REPLAN_TRAJ, "trajectory");
  //       std::cout<<"num:"<<num<<std::endl;
  //       this->trajectory_num = num;
  //       return;
  //     }
  //   }
  // }
  
  /* ---------- check trajectory ---------- */

  if (exec_state_ == FSM_EXEC_STATE::EXEC_TRAJ) {
    // std::cout<<"check:"<<std::endl;
    // cout << "check collision1" << endl;
    std::pair<bool,int>  collision = planner_manager_->checkTrajCollision();
    // std::cout<<"check2:"<<std::endl;

    if (collision.first) {
      ROS_WARN("current traj in collision.");
      collision_index = collision.second;
      if(collision_index>2)//collision_index<=10&&collision_index>=2)
      {
        changeFSMExecState(REPLAN_TRAJ, "SAFETY1");
        std::cout<<"collision_index:"<<collision_index<<std::endl;
      }
      return;
    }
    else{
      collision_index = -1;
    }
  }
}

bool RSAStarReplanFSM::callRSAStardynamicReplan() {

  auto start_time = std::chrono::high_resolution_clock::now();
  bool plan_success = planner_manager_->RSAStardynamicReplan(this->init_control, this->odom_pos_,this->end_pt_,this->time_start, this->odom_pos_); // 开始和结束的点
  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
  std::cout << "执行时间fsm: " << duration.count() << " 毫秒" << std::endl;

  // start_pt 那个点结束的位置.
  trigger_ = false;
  // cout << "final_plan_success=" << plan_success << endl;

  if (plan_success) {

    // planner_manager_->planYaw(start_yaw_);  // some problems there;

    auto info = &planner_manager_->local_data_;

    /* publish traj */
    plan_manage::Bspline bspline;
    bspline.order      = 3;
    bspline.start_time = info->start_time_; // 开始时间
    bspline.traj_id    = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();

    for (int i = 0; i < pos_pts.cols(); ++i) {
      geometry_msgs::Point pt;
      pt.x = pos_pts(0, i);
      pt.y = pos_pts(1, i);
      pt.z = pos_pts(2, i);
      bspline.pos_pts.push_back(pt);
    }
    // std::cout<<"llll"<<std::endl;

    Eigen::VectorXd knots = info->position_traj_.getKnot();  // 得到节点
    for (int i = 0; i < knots.rows(); ++i) {
      bspline.knots.push_back(knots(i));
    }

    // std::cout<<"llll1"<<std::endl;

    // Eigen::MatrixXd yaw_pts = info->yaw_traj_.getControlPoint();
    // for (int i = 0; i < yaw_pts.cols(); ++i) {
    //   double yaw = yaw_pts(0,i);
    //   bspline.yaw_pts.push_back(yaw);
    // }
    // std::cout<<"llll2"<<std::endl;
    // bspline.yaw_dt = info->yaw_traj_.getInterval();

    bspline_pub_.publish(bspline);

    /* visulization */
    // auto plan_data = &planner_manager_->plan_data_;
    // visualization_->drawGeometricPath(plan_data->kino_path_, 0.075, Eigen::Vector4d(1, 1, 0, 0.4));
    visualization_->drawBspline(info->position_traj_, 0.08, Eigen::Vector4d(1.0, 0, 0.0, 1), true, 0.02,
                                Eigen::Vector4d(1, 0, 0, 1));
    visualization_->drawBspline(info->position_traj_1, 0.08, Eigen::Vector4d(0, 0, 1, 1), true, 0.02,
                                Eigen::Vector4d(0, 0, 1, 1),20,20,1);
    std::cout<<"llll3"<<std::endl;

    return true;

  } else {
    // cout << "generate new traj fail." << endl;
    return false;
  }

}

Eigen::Matrix<double, 3, 3> RSAStarReplanFSM::SolveInitPoint(Eigen::Vector3d p,Eigen::Vector3d v,Eigen::Vector3d a)
{
  Eigen::Matrix<double, 3, 3> A,B,C;
  A<< 0.1667,0.6667,0.1667,
      -0.500,0,0.500,
      1,-2,1;
  C.row(0)=p;
  C.row(1)=v;
  C.row(2)=a;
  
  B = A.inverse() * C;
  return B;    
}

  // bool EGOReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
  // {

  //   getLocalTarget();

  //   bool plan_success =
  //       planner_manager_->reboundReplan(start_pt_, start_vel_, start_acc_, local_target_pt_, local_target_vel_, (have_new_target_ || flag_use_poly_init), flag_randomPolyTraj);
  //   have_new_target_ = false;

  //   cout << "final_plan_success=" << plan_success << endl;

  //   if (plan_success)
  //   {

  //     auto info = &planner_manager_->local_data_;

  //     /* publish traj */
  //     ego_planner::Bspline bspline;
  //     bspline.order = 3;
  //     bspline.start_time = info->start_time_;
  //     bspline.traj_id = info->traj_id_;

  //     Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
  //     bspline.pos_pts.reserve(pos_pts.cols());
  //     for (int i = 0; i < pos_pts.cols(); ++i)
  //     {
  //       geometry_msgs::Point pt;
  //       pt.x = pos_pts(0, i);
  //       pt.y = pos_pts(1, i);
  //       pt.z = pos_pts(2, i);
  //       bspline.pos_pts.push_back(pt);
  //     }

  //     Eigen::VectorXd knots = info->position_traj_.getKnot();
  //     bspline.knots.reserve(knots.rows());
  //     for (int i = 0; i < knots.rows(); ++i)
  //     {
  //       bspline.knots.push_back(knots(i));
  //     }

  //     bspline_pub_.publish(bspline);

  //     visualization_->displayOptimalList(info->position_traj_.get_control_points(), 0);
  //   }

  //   return plan_success;
  // }

// RSAStarReplanFSM::
}  // namespace fast_planner
