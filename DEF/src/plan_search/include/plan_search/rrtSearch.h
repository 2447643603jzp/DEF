/*
*	File: rrtSearch.h
*	---------------
*/
#ifndef RRTG_H
#define RRTG_H
#include <ros/ros.h>
#include <plan_search/KDTree.h>
#include <limits>
#include <visualization_msgs/MarkerArray.h>
#include <thread>
#include <mutex>
#include <random>
#include <ros/console.h>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <boost/functional/hash.hpp>
#include <iostream>
#include <map>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>
#include <plan_env/edt_environment.h>
#include <plan_env/sdf_map.h>
#include <plan_search/separator.hpp>
#include <plan_search/timer.hpp>
#include <detector/Boxdetector.h>
#include <plan_search/utils.h>
#include <sstream>

using namespace std;
using namespace Eigen;

using namespace fast_planner;

namespace def_planner{
	// std::random_device rd;
	// std::mt19937 mt(rd());
	// Helper Function: Random Number


	template <std::size_t N>
	class rrtSearch {
	private:
	    /*random*/
		// std::random_device rd;
	    // std::mt19937 mt(rd());

		std::mt19937 mt;
        ros::NodeHandle nh_;
		std::vector<KDTree::Point<N>> start_;
		KDTree::Point<N> start_use;
		KDTree::Point<N> goal_,temp_goal;
		KDTree::Point<N> emptyToken_;
		KDTree::KDTree<N, int> ktree_; // KDTree
		std::unordered_map<KDTree::Point<N>, KDTree::Point<N>, KDTree::PointHasher> parent_,temp_parent_; // for backtracking
		std::unordered_map<KDTree::Point<N>, KDTree::Point<N>, KDTree::PointHasher> parent_end; // for backtracking
		std::vector<double> collisionBox_; // (lx, ly, lz)
		double collisionBox_x,collisionBox_y,collisionBox_z;
		std::vector<double> envBox_; // value of min max of x y and z
		double connectGoalRatio_;
		double timeout_;
		double dR_,plan_horizon;

		ros::Publisher RRTVisPub_;
		ros::Publisher pathVisPub_;

		double mapRes_;
		double envLimit_[6];
		double sampleRegion_[6];
		
		visualization_msgs::MarkerArray RRTVisMsg_;
		visualization_msgs::MarkerArray pathVisMsg_;
		std::vector<visualization_msgs::Marker> RRTVisvec_; // we update this
		std::vector<visualization_msgs::Marker> pathVisVec_; // update this
		bool visRRT_;
		bool visPath_;

		/*map*/
		double resolution_, inv_resolution_;
        Eigen::Vector3d origin_, map_size_3d_;
		Eigen::Vector3d min_id, max_id;
        int x_size,y_size,z_size;

		/*bspline*/
		const static int P_ = 3;
		typedef Eigen::Matrix<double, 1, P_ + 1> VectorNT;
        typedef Eigen::Matrix<double, P_ + 1, 1> VectorN;
        typedef Eigen::Matrix<double, P_ + 1, P_ + 1> MatrixN;
        typedef std::array<MatrixN, 4> MatrixNArray;

		MatrixN bspline_basis_matrix;
        MatrixN bezier_basis_matrix;
        MatrixN bspline2bezier_matrix;
        // MatrixN bspline2bezier_diff1_matrix;
        // MatrixN bspline2bezier_diff2_matrix;
        Eigen::MatrixXd bspline_diff1_matrix;
        Eigen::MatrixXd bspline_diff2_matrix;
        MatrixN diff_matrix;

		/*param*/
		std::vector<double> collisionBox, envBox;
		double delQ_, dR, connectGoalRatio, mapRes;

		double bspline_dist;
		double ts;

		Eigen::Matrix<double, P_ + 1, 3> local_control_points;

		/* ---------- record data ---------- */
        EDTEnvironment::Ptr edt_environment_;
        BoxDetector::Ptr box_detector_;
        separator::Separator* separator_solver_;

		/*start time*/
		double time_start;
		double max_vel_,max_acc_;

		/*result*/
		Eigen::MatrixXd temp_result_;
		Eigen::MatrixXd result_;
		std::vector<Eigen::Vector3d> n_result;
        std::vector<double> d_result; //这个值真的很重要的，最后求解的向量

		ConvexHullsOfCurves_Std hulls_curve;

		// ellipse  // 椭圆的采样点和采样的空间。
		double Cmin,Cmax;
		double center_x,center_y;
		bool use_ellipse =false;
		double tanta;  // 椭圆的角度。
		double b_ellipse,a_ellipse;

		double max_jerk;
		double max_jerk_last = 0;

	

	public:
		std::thread RRTVisWorker_;
		std::thread pathVisWorker_;
		enum { REACH_HORIZON = 1, REACH_END = 2, NO_PATH = 3, RUNTIME_REACHED = 4 };
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		// default constructor
		rrtSearch();

		// reset
		void reset();

		// get result
		Eigen::MatrixXd getctrlpoint();

		// env
		void setEnvironment(const EDTEnvironment::Ptr& env);

		// box
		void setBoxDetector(const BoxDetector::Ptr &detector_);

		// update samplemap	
		void updateSampleRegion(Eigen::Vector3d min_sample,Eigen::Vector3d max_sample);// helper function for update sample region
		
		// collision checking function based on map and collision box: TRUE => Collision
		bool checkCollision(Eigen::Vector3d p);

		// shortcut path
		void shortcutWaypointPaths(const std::vector<KDTree::Point<N>>& plan, std::vector<KDTree::Point<N>>& planSc);

		// random sample in valid space (based on current map)
		void randomConfig(KDTree::Point<N>& qRand);

		// *** Core function: make plan based on all input ***
		int search(Eigen::Matrix<double, 3, 3> init_control, Eigen::Vector3d end_pt,double time_start);

		// Visualization
		void startVisModule();
		void publishRRTVisMsg();
		void publishPathVisMsg();
		void updateRRTVisVec(const KDTree::Point<N>& qNew, const KDTree::Point<N>& qNear, int id);
		void updatePathVisVec(const std::vector<KDTree::Point<N>> &plan);


		double getMapRes();

		// add the new vertex to the RRT: add to KDTree
		void addVertex(const KDTree::Point<N>& qNew);

		// Find the nearest vertex (node) in the tree
		void nearestVertex(const KDTree::Point<N>& qKey, KDTree::Point<N>& qNear);

		// Steer function: basd on delta
		void newConfig(const KDTree::Point<N>& qNear, const KDTree::Point<N>& qRand, KDTree::Point<N>& qNew);

		// add the new edge to RRT: add to rrt 
		void addEdge(const KDTree::Point<N>& qNear, const KDTree::Point<N>& qNew);

		bool isReach(const KDTree::Point<N>& q);

		void backTrace(const KDTree::Point<N>& qGoal, std::vector<KDTree::Point<N>>& plan);

		void setParam(ros::NodeHandle& nh);

		void init();

		const MatrixN computeBasisMatrix_Bspline(int diff_degree = 0);
        const MatrixN computeBasisMatrix_Bezier(int diff_degree = 0);

		void KdPoint2Eigen(KDTree::Point<N>& q,Eigen::Vector3d &point);
		void Eigen2KdPoint(KDTree::Point<N>& q,Eigen::Vector3d &point);
        
		// control point 
		bool SetLocalControlPoints(KDTree::Point<N>  qNew, KDTree::Point<N>  qNear);

		std::vector<double> getCollisionBox();

	    // return goal conenct ratio
	    double getConnectGoalRatio();

	    // return timeout
	    double getTimeout();

        void initstart();

		bool getpath(std::vector<KDTree::Point<N>>& plan,std::vector<std::pair<int,int>> index_opt);

		uint64_t C_n_k(uint64_t n, uint64_t k);

		bool checkTrajDynamics();
		// bool checkTrajDynamics1(Eigen::Matrix<double,4,3> Q); 

		bool collidesWithObstaclesGivenVertexes(const Eigen::Matrix<double, 3, 4>& last4Cps,int num);

		bool CheckDynamicObstacle(int num);


		bool checkCollisionAll(const KDTree::Point<N> qNear,int num);
		bool checkdynamicCollision(Eigen::Matrix<double,4,3> Q,double t_start);

		int getNumPoint(const KDTree::Point<N> qNear);
		std::pair<int,int> getSearchSegment(std::vector<KDTree::Point<N>> controlpt,int segment_start);

		bool CheckStaticObstacle();
		ConvexHullsOfCurves_Std gethulls();
		std::vector<double> getndresult();
		bool checkFeasAndFillND(std::vector<Eigen::Vector3d> &q, std::vector<Eigen::Vector3d>& n,
                                       std::vector<double>& d,std::vector<std::pair<int,int>> index_opt);
		void getellipse(std::vector<KDTree::Point<N>>& plan);
		void parameterizeToBspline(const double &ts, const vector<Eigen::Vector3d> &point_set,
                                             const vector<Eigen::Vector3d> &start_end_derivative,
                                             Eigen::MatrixXd &ctrl_pts);

        double randomNumber(double min, double max){
		    std::uniform_real_distribution<double> distribution(min, max);
		    return distribution(mt);
	    }

	};


	// ===========================Function Definition=======================================
	template <std::size_t N>
	rrtSearch<N>::rrtSearch(){}

	template <std::size_t N>
    void rrtSearch<N>::setParam(ros::NodeHandle& nh)
    {
      std::cout<<"aa"<<std::endl;
      nh.param("search/max_vel", this->max_vel_, -1.0);
      nh.param("search/max_acc", this->max_acc_, -1.0);
      nh.param("search/horizon", this->plan_horizon, -1.0);
      nh.param("search/resolution", this->resolution_, -1.0);
      nh.param("search/vis_RRT", this->visRRT_, false);
      nh.param("search/vis_Path", this->visPath_, false);
      nh.param("search/max_runtime", this->timeout_, -1.0);
	  nh.param("search/dist2end", this->dR_, -1.0);
	  nh.param("search/probability2end", this->connectGoalRatio_, -1.0);
      nh.param("search/collision_box_x", this->collisionBox_x, -1.0);
      nh.param("search/collision_box_y", this->collisionBox_y, -1.0);
      nh.param("search/collision_box_z", this->collisionBox_z, -1.0);
      nh.param("manager/ts", this->ts, -1.0);
	  nh.param("manager/max_jerk", this->max_jerk, -1.0);
      this->collisionBox_.push_back(this->collisionBox_x);
      this->collisionBox_.push_back(this->collisionBox_y);
      this->collisionBox_.push_back(this->collisionBox_z);
      this->nh_ = nh;
	  std::cout<<"connectGoalRatio_:"<<connectGoalRatio_<<std::endl;
    }

	template <std::size_t N>
	void rrtSearch<N>::backTrace(const KDTree::Point<N>& qGoal, std::vector<KDTree::Point<N>>& plan){
		KDTree::Point<N> ptr = qGoal;
		while (ptr != this->emptyToken_){
			plan.push_back(ptr);
			ptr = this->parent_[ptr];
		}
		std::reverse(plan.begin(), plan.end());
	}

    template <std::size_t N>
	bool rrtSearch<N>::checkFeasAndFillND(std::vector<Eigen::Vector3d> &q, std::vector<Eigen::Vector3d>& n,
                                       std::vector<double>& d,std::vector<std::pair<int,int>> index_opt)
    {
    //    if(box_detector_->getnumtrack() == 0)
    //    {
    //       return true;
    //    }
       
	//    int num_of_segments_ = 0;
	//    for(int i =0;i<index_opt.size();i++ )
	//    {
    //       int num_temp = index_opt[i].second - index_opt[i].first+1-P_;
	// 	  num_of_segments_ = num_of_segments_+num_temp;
	//    }

    
    //    int num = num_of_segments_*box_detector_->getnumtrack();
    //    n_result.resize(std::max(num, 0), Eigen::Vector3d::Zero());
    //    d_result.resize(std::max(num, 0), 0.0);

	//    for(int i =0;i<index_opt.size();i++ )
	//    {
	// 	int num_temp = index_opt[i].second - index_opt[i].first+1-P_;
	// 	double time_segment = time_start+(index_opt[i].first-P_)*this->ts;
    //     this->hulls_curve = box_detector_-> getAllIntervalnHull(time_segment,this->ts,num_temp);

    //     Eigen::Matrix<double, 3, 4> last4Cps;  // Each column contains a control point

    //     // Check obstacles constraints (and compute n and d)
    //     for (int index_interv = index_opt[i].first; index_interv < index_opt[i].second-P_; index_interv++)
    //     {
    //       last4Cps.col(0) = q[index_interv];
    //       last4Cps.col(1) = q[index_interv + 1];
    //       last4Cps.col(2) = q[index_interv + 2];
    //       last4Cps.col(3) = q[index_interv + 3];   //求解到每一个控制点的值 n0 求解到曲线值
    //       Eigen::Matrix<double, 3, 4> last4Cps_new_basis = last4Cps*bspline2bezier_matrix.transpose();
    //       for(int obst_index=0;obst_index< hulls_curve.size();obst_index++)
    //       {
    //         Eigen::Vector3d n_i;
    //         double d_i;
    //         bool solved = separator_solver_->solveModel(n_i, d_i, hulls_curve[obst_index][index_interv], last4Cps_new_basis); // 第几个障碍物的这一段
    //         if (solved == false)
    //         {
    //            return false;
    //         }
    //         n_result.push_back(n_i);  //第一段每一个障碍物，第二段每一个障碍物。
    //         d_result.push_back(d_i);
    //       }
    //     }
	//    }
        
    //     return true;
	   if(box_detector_->getnumtrack() == 0)
       {
          return true;
       }
	   int num_of_segments_ = q.size() - P_;
       int num = num_of_segments_*box_detector_->getnumtrack();
       n.resize(std::max(num, 0), Eigen::Vector3d::Zero());
       d.resize(std::max(num, 0), 0.0);
	   this->hulls_curve = box_detector_-> getAllIntervalnHull(time_start,ts,num_of_segments_);

       
       Eigen::Matrix<double, 3, 4> last4Cps;  // Each column contains a control point

       // Check obstacles constraints (and compute n and d)
       for (int index_interv = 0; index_interv < num_of_segments_; index_interv++)
       {
          last4Cps.col(0) = q[index_interv];
          last4Cps.col(1) = q[index_interv + 1];
          last4Cps.col(2) = q[index_interv + 2];
          last4Cps.col(3) = q[index_interv + 3];   //求解到每一个控制点的值 n0 求解到曲线值
          Eigen::Matrix<double, 3, 4> last4Cps_new_basis = last4Cps*bspline2bezier_matrix.transpose();
          for(int obst_index=0;obst_index< hulls_curve.size();obst_index++)
          {
            int ip = obst_index * num_of_segments_ + index_interv;
            Eigen::Vector3d n_i;
            double d_i;
			for(int jj=0;jj<hulls_curve[obst_index][index_interv].cols();jj++)
			{
				std::cout<<"cols:"<<hulls_curve[obst_index][index_interv].col(jj)<<std::endl;
			}
			std::cout<<last4Cps<<std::endl;
            bool solved = separator_solver_->solveModel(n_i, d_i, hulls_curve[obst_index][index_interv], last4Cps_new_basis); // 第几个障碍物的这一段
			std::cout<<"n_i:"<<n_i.transpose()<<std::endl;
			std::cout<<"d_i:"<<d_i<<std::endl;
            if (solved == false)
            {
               return false;
            }
            n_result[obst_index * num_of_segments_ + index_interv] = n_i;  //第一段每一个障碍物，第二段每一个障碍物。
            d_result[obst_index * num_of_segments_ + index_interv] = d_i;

          }
        }
        
        return true;
    }

	// template <std::size_t N>
	// bool rrtSearch<N>::checkTrajDynamics1(Eigen::Matrix<double,4,3> Q) {
    //     Eigen::Matrix<double, P_ + 1, 3> bspline_diff0_control_points;
    //     Eigen::Matrix<double, P_, 3> bspline_diff1_control_points;
    //     Eigen::Matrix<double, P_ - 1, 3> bspline_diff2_control_points;

    //     bspline_diff0_control_points = Q;
    //     bspline_diff1_control_points = (bspline_diff1_matrix * bspline_diff0_control_points);//.block(0, 0, P_, 3);
    //     bspline_diff2_control_points = (bspline_diff2_matrix * bspline_diff0_control_points);//.block(0, 0, P_ - 1, 3);
    //     if (bspline_diff1_control_points.maxCoeff() > this->max_vel_ ||   //返回矩阵中最大和最小值
    //         bspline_diff1_control_points.minCoeff() < -this->max_vel_ ||
    //         bspline_diff2_control_points.maxCoeff() > this->max_acc_ ||
    //         bspline_diff2_control_points.minCoeff() < -this->max_acc_
    //     ) {
    //        return true;//Not dynamically feasible
    //     }
    //     return false;//Dynamically feasible
    // }

	template <std::size_t N>
	bool rrtSearch<N>::isReach(const KDTree::Point<N>& q){
		return KDTree::Distance(q, this->temp_goal) <= this->dR_; // 是那个重要的直。
	}

	template <std::size_t N>
	void rrtSearch<N>::addEdge(const KDTree::Point<N>& qNear, const KDTree::Point<N>& qNew){
		this->parent_[qNew] = qNear;
	}

	template <std::size_t N>
	void rrtSearch<N>::addVertex(const KDTree::Point<N>& qNew){
	    this->ktree_.insert(qNew);
	}

	template <std::size_t N>
	void rrtSearch<N>::updateSampleRegion(Eigen::Vector3d min_sample,Eigen::Vector3d max_sample){
	    this->sampleRegion_[0] = min_sample(0);
		this->sampleRegion_[1] = max_sample(0);
		this->sampleRegion_[2] = min_sample(1);
		this->sampleRegion_[3] = max_sample(1);
		this->sampleRegion_[4] = 0;//this->min_id(2);
		this->sampleRegion_[5] = std::min(5.0,double(max_sample(2)));
	}

	template <std::size_t N>
	bool rrtSearch<N>::checkCollision(Eigen::Vector3d p){
		if(this->edt_environment_->sdf_map_->getInflateOccupancy(p) == 1)
		{
			return true;
		}
		else
		{
			return false;
		}
	}

	template <std::size_t N>
	bool rrtSearch<N>::checkdynamicCollision(Eigen::Matrix<double,4,3> Q,double t_start){

       if(box_detector_->getnumtrack()==0)
       {
          return false;
       } 
	   Eigen::Vector3d n_i;
       double d_i;
       Eigen::Matrix<double, 3, 4> last4Cps;  // Each column contains a control point
	   last4Cps = (this->bspline2bezier_matrix*Q).transpose();
	   std::vector<Eigen::Matrix<double, 3, Eigen::Dynamic>> obstacles = box_detector_->getIntervalHull(t_start,this->ts);
	    for (int obst_index = 0; obst_index < obstacles.size(); obst_index++)
        {
           bool satisfies_LP = separator_solver_->solveModel(n_i, d_i, obstacles[obst_index], last4Cps); // 第几个障碍物的这一段   
           if (satisfies_LP == false)
           {
              return true;
           }
        }
		return false;
	}


    template <std::size_t N>
	bool rrtSearch<N>::CheckDynamicObstacle(int num)
    {
       Eigen::Matrix<double, 3, 4> last4Cps;  // Each column contains a control point
       Eigen::Matrix<double, P_ + 1, 3> bspline_diff0_control_points;
       bspline_diff0_control_points = this->local_control_points;
       last4Cps = (this->bspline2bezier_matrix*bspline_diff0_control_points).transpose();
       // std::cout<<"now_collide"<<std::endl;
       bool collides = collidesWithObstaclesGivenVertexes(last4Cps,num);
       return collides;
    }    

    template <std::size_t N>
    bool rrtSearch<N>::collidesWithObstaclesGivenVertexes(const Eigen::Matrix<double, 3, 4>& last4Cps,int num)
    {
        Eigen::Vector3d n_i;
        double d_i;
        double t_init = this->time_start + this->ts*(num-1);
  
        if(box_detector_->getnumtrack()==0)
        {
           return false;
        }  

        std::vector<Eigen::Matrix<double, 3, Eigen::Dynamic>> obstacles = box_detector_->getIntervalHull(t_init,this->ts); 
        for (int obst_index = 0; obst_index < obstacles.size(); obst_index++)
        {
           bool satisfies_LP = separator_solver_->solveModel(n_i, d_i, obstacles[obst_index], last4Cps); // 第几个障碍物的这一段   
           if (satisfies_LP == false)
           {
              return true;
           }
        }
        return false;
    } 


	template <std::size_t N>
	void rrtSearch<N>::KdPoint2Eigen(KDTree::Point<N>& q,Eigen::Vector3d &point)
	{
        point = Eigen::Vector3d(q[0],q[1],q[2]);
	}

	template <std::size_t N>
	void rrtSearch<N>::Eigen2KdPoint(KDTree::Point<N>& q,Eigen::Vector3d &point)
	{
        q[0] = point(0);
		q[1] = point(1);
		q[2] = point(2);
	}

	template <std::size_t N>
	Eigen::MatrixXd rrtSearch<N>::getctrlpoint()
	{
      return this->result_;
	}


	template <std::size_t N>
	void rrtSearch<N>::newConfig(const KDTree::Point<N>& qNear, const KDTree::Point<N>& qRand, KDTree::Point<N>& qNew){
		// TODO: implement steer function
		double distance = Distance(qNear, qRand);
		KDTree::Point<N> direction = qRand - qNear;
		qNew = qNear + (this->delQ_/distance) * direction;
	}

	template <std::size_t N>
	void rrtSearch<N>::nearestVertex(const KDTree::Point<N>& qKey, KDTree::Point<N>& qNear){
		this->ktree_.nearestNeighbor(qKey, qNear);
	}

	template <std::size_t N>
	bool rrtSearch<N>::checkCollisionAll(const KDTree::Point<N> qNear,int num){
		// 把所有需要检查的都进行检查
		// std::cout<<"seg:"<<std::endl;
        int segment = getNumPoint(qNear) + num + 1 - P_;
        bool not_satified;
        // std::cout<<"segment:"<<segment<<std::endl;
		/*step1 checkTrajDynamics*/
        not_satified = checkTrajDynamics();
		if(not_satified)
		{
			return true;
		}
        // std::cout<<"2check:"<<std::endl;
		/*step1, check static env*/
		not_satified = CheckStaticObstacle();
		if(not_satified)
		{
			return true;
		}
        // std::cout<<"2check:"<<std::endl;
		/*step2 check dynamic env*/
		not_satified = CheckDynamicObstacle(segment);
		if(not_satified)
		{
			return true;
		}
        // std::cout<<"1check:"<<std::endl;
		return false;
	}

	template <std::size_t N>
	int rrtSearch<N>::getNumPoint(const KDTree::Point<N> qNear)
	{
		KDTree::Point<N> ptr = qNear;
		// std::cout<<"qNear:"<<qNear<<std::endl;
		int num =0;
		while (ptr != this->emptyToken_){
			num = num + 1;
			// std::cout<<"ptr:"<<ptr<<std::endl;
			ptr = this->parent_[ptr];
		}
		// std::cout<<"seg:"<<num<<std::endl;
		return num;
	}
    
	template <std::size_t N>
	bool rrtSearch<N>::CheckStaticObstacle() {
        Eigen::Matrix<double, P_ + 1, 3> Q;
        Q = this->local_control_points;

        for(double time=0;time<=1;time+=0.2)
        {
           Eigen::VectorXd t_pow(4);
           t_pow<< 1,pow(time,1),pow(time,2),pow(time,3);
           Eigen::Vector3d point =t_pow.transpose()*this->bspline_basis_matrix*Q;
      
           if(this->edt_environment_->sdf_map_->getInflateOccupancy(point) == 1)
           {
             return true;
           }
        }

        return false;
    }
    


    template <std::size_t N>
	bool rrtSearch<N>::checkTrajDynamics() {
        Eigen::Matrix<double, P_ + 1, 3> bspline_diff0_control_points;
        Eigen::Matrix<double, P_, 3> bspline_diff1_control_points;
        Eigen::Matrix<double, P_ - 1, 3> bspline_diff2_control_points;

        bspline_diff0_control_points = local_control_points;
        bspline_diff1_control_points = (bspline_diff1_matrix * bspline_diff0_control_points);//.block(0, 0, P_, 3);
        bspline_diff2_control_points = (bspline_diff2_matrix * bspline_diff0_control_points);//.block(0, 0, P_ - 1, 3);
        //  std::cout<<"coffe:"<<bezier_diff1_control_points.maxCoeff()<<" "<<bezier_diff1_control_points.minCoeff()<<" "<<bezier_diff2_control_points.maxCoeff()<<" "<<bezier_diff2_control_points.minCoeff()<<std::endl;

        if (bspline_diff1_control_points.maxCoeff() > this->max_vel_ ||   //返回矩阵中最大和最小值
            bspline_diff1_control_points.minCoeff() < -this->max_vel_ ||
            bspline_diff2_control_points.maxCoeff() > this->max_acc_ ||
            bspline_diff2_control_points.minCoeff() < -this->max_acc_
        ) {
           return true;//Not dynamically feasible
        }
        return false;//Dynamically feasible
    }


	template <std::size_t N>
	void rrtSearch<N>::shortcutWaypointPaths(const std::vector<KDTree::Point<N>>& plan, std::vector<KDTree::Point<N>>& planSc){
		
		int ptr1 = P_-1; int ptr2 = P_;
		// planSc.push_back(plan[ptr1]);
		// this->parent_end[plan[ptr2]] = plan[ptr1];
		KDTree::Point<N> ptr;
		std::cout<<"fff"<<std::endl;
		int numtemp =0;
		while (true and ros::ok()){
			numtemp =numtemp+1;
			std::cout<<"numtemp:"<<numtemp<<std::endl;
			std::cout<<"ptr2:"<<ptr2<<std::endl;
			KDTree::Point<N> p1 = plan[ptr1]; KDTree::Point<N> p2 = plan[ptr2];

			KDTree::Point<N> direction = p1 - p2;
			double dx = abs(direction[0]);
			double dy = abs(direction[1]);
	        double dz = abs(direction[2]);// x y z 
			bool is_sati = true;
			if(dx>0.8*bspline_dist||dy>0.8*bspline_dist||dz>0.8*bspline_dist)
			{
               is_sati = false;
			}
			if (is_sati and not checkCollisionAllEnd(p1, p2)){
			    if (ptr2 >= plan.size()-1-P_){
                       this->parent_end[plan[ptr2]] = plan[ptr1];
					   // planSc.push_back(p2);
                       ptr = plan[ptr2];
					   break;
				    }
				    ++ptr2;
				    std::cout<<"fff2"<<std::endl;
		    }
			else{
				this->parent_end[plan[ptr2]] = plan[ptr1];
				// planSc.push_back(plan[ptr2-1]);
				ptr1 = ptr2-1;
				// if (ptr2 >= plan.size()-1-P_){
				// 	break;
				// }
			}
		}


		std::cout<<"fff1"<<std::endl;
		ptr = plan[ptr2];
		while (ptr != this->emptyToken_){
			planSc.push_back(ptr);
			ptr = this->parent_end[ptr];
			std::cout<<"fff99"<<std::endl;
		}
		std::reverse(planSc.begin(), planSc.end());
		int length = plan.size()-1;
		planSc.push_back(plan[length-2]);
		planSc.push_back(plan[length-1]);
		planSc.push_back(plan[length]);

	}

	template <std::size_t N>
	void rrtSearch<N>::randomConfig(KDTree::Point<N>& qRand){
		if(!this->use_ellipse)
		{
			bool valid = false;
		    double x, y, z;
		    Eigen::Vector3d p;
		    while (not valid){
			   p.x() = randomNumber(this->sampleRegion_[0], this->sampleRegion_[1]);
			   p.y() = randomNumber(this->sampleRegion_[2], this->sampleRegion_[3]);
			   p.z() = randomNumber(this->sampleRegion_[4], this->sampleRegion_[5]);
			   valid = not this->checkCollision(p);
		    }
            qRand[0] = p.x(); qRand[1] = p.y(); qRand[2] = p.z();
		}

		else
		{
			bool valid = false;
		    double x, y, z;
		    Eigen::Vector3d p;
		    while (not valid){
				double angle = randomNumber(0.0, 2*3.141592654);
				double radius = randomNumber(0.0, 1.0);
				double x,y,x_2,y_2;
				x = radius * std::cos(angle);
                y = radius * std::sin(angle);
                // trans
				x_2 = (a_ellipse/2)*x+center_x;
				y_2 = (b_ellipse/2)*y+center_y;    //sqrt(pow(Cmax,2)-pow(Cmin,2));
                
			    p.x() = cos(tanta)*x_2-sin(tanta)*y_2;
			    p.y() = sin(tanta)*x_2+cos(tanta)*y_2;
			    p.z() = randomNumber(this->sampleRegion_[4], this->sampleRegion_[5]);
			    valid = not this->checkCollision(p);
		    }
            qRand[0] = p.x(); qRand[1] = p.y(); qRand[2] = p.z();

		}

	}

	template <std::size_t N>
	void rrtSearch<N>::startVisModule(){
		if (this->visRRT_){
			this->RRTVisPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>("/rrt_vis_array", 1);
			this->RRTVisWorker_ = std::thread(&rrtSearch<N>::publishRRTVisMsg, this);
		}

		if (this->visPath_){
			this->pathVisPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>("/rrt_planned_path", 10);
			this->pathVisWorker_ = std::thread(&rrtSearch<N>::publishPathVisMsg, this);
		}
	}

	template <std::size_t N>
	void rrtSearch<N>::publishRRTVisMsg(){
		ros::Rate rate(5);
		while (ros::ok()){
			this->RRTVisMsg_.markers = this->RRTVisvec_;
			this->RRTVisPub_.publish(this->RRTVisMsg_);
			rate.sleep();
		}
	}
	
	template <std::size_t N>
	void rrtSearch<N>::publishPathVisMsg(){
		ros::Rate rate(20);
		while (ros::ok()){
			this->pathVisPub_.publish(this->pathVisMsg_);
			rate.sleep();
		}
	}

	template <std::size_t N>
	void rrtSearch<N>::updateRRTVisVec(const KDTree::Point<N>& qNew, const KDTree::Point<N>& qNear, int id){
		visualization_msgs::Marker point;
		visualization_msgs::Marker line;
		geometry_msgs::Point p1, p2;
		std::vector<geometry_msgs::Point> lineVec;
		
		// point:
		point.header.frame_id = "world";
		point.ns = "RRT_point";
		point.id = id;
		point.type = visualization_msgs::Marker::SPHERE;
		point.pose.position.x = qNew[0];
		point.pose.position.y = qNew[1];
		point.pose.position.z = qNew[2];
		point.lifetime = ros::Duration(1);
		point.scale.x = 0.2;
		point.scale.y = 0.2;
		point.scale.z = 0.2;
		point.color.a = 0.8;
		point.color.r = 1;
		point.color.g = 0;
		point.color.b = 0;
		this->RRTVisvec_.push_back(point);

		// line:
		p1.x = qNew[0];
		p1.y = qNew[1];
		p1.z = qNew[2];
		p2.x = qNear[0];
		p2.y = qNear[1];
		p2.z = qNear[2]; 
		lineVec.push_back(p1);
		lineVec.push_back(p2);

		line.header.frame_id = "world";
		line.ns = "RRT_line";
		line.points = lineVec;
		line.id = id;
		line.type = visualization_msgs::Marker::LINE_LIST;
		line.lifetime = ros::Duration(1);
		line.scale.x = 0.05;
		line.scale.y = 0.05;
		line.scale.z = 0.05;
		line.color.a = 1.0;
		line.color.r = 1*0.5;
		line.color.g = 0;
		line.color.b = 1;
		this->RRTVisvec_.push_back(line);
	}

	template <std::size_t N>
	void rrtSearch<N>::updatePathVisVec(const std::vector<KDTree::Point<N>> &plan){
		this->pathVisVec_.clear();
		visualization_msgs::Marker waypoint;
		visualization_msgs::Marker line;
		geometry_msgs::Point p1, p2;
		std::vector<geometry_msgs::Point> lineVec;
		for (int i=0; i < plan.size(); ++i){
			KDTree::Point<N> currentPoint = plan[i];
			if (i != plan.size() - 1){
				KDTree::Point<N> nextPoint = plan[i+1];
				p1.x = currentPoint[0];
				p1.y = currentPoint[1];
				p1.z = currentPoint[2];
				p2.x = nextPoint[0];
				p2.y = nextPoint[1];
				p2.z = nextPoint[2]; 
				lineVec.push_back(p1);
				lineVec.push_back(p2);
			}
			// waypoint
			waypoint.header.frame_id = "world";
			waypoint.id = 1+i;
			waypoint.ns = "rrt_path";
			waypoint.type = visualization_msgs::Marker::SPHERE;
			waypoint.pose.position.x = currentPoint[0];
			waypoint.pose.position.y = currentPoint[1];
			waypoint.pose.position.z = currentPoint[2];
			waypoint.lifetime = ros::Duration(0.5);
			waypoint.scale.x = 0.2;
			waypoint.scale.y = 0.2;
			waypoint.scale.z = 0.2;
			waypoint.color.a = 0.8;
			waypoint.color.r = 0.3;
			waypoint.color.g = 1;
			waypoint.color.b = 0.5;
			this->pathVisVec_.push_back(waypoint);
		}
		line.header.frame_id = "world";
		line.points = lineVec;
		line.ns = "rrt_path";
		line.id = 0;
		line.type = visualization_msgs::Marker::LINE_LIST;
		line.lifetime = ros::Duration(0.5);
		line.scale.x = 0.05;
		line.scale.y = 0.05;
		line.scale.z = 0.05;
		line.color.a = 1.0;
		line.color.r = 0.5;
		line.color.g = 0.1;
		line.color.b = 1;
		this->pathVisVec_.push_back(line);
	}
	

	template <std::size_t N>
	double rrtSearch<N>::getMapRes(){
		return this->resolution_;
	}
	template <std::size_t N>
	std::vector<double> rrtSearch<N>::getCollisionBox(){
		return this->collisionBox_;
	}

	template <std::size_t N>
	double rrtSearch<N>::getConnectGoalRatio(){
		return this->connectGoalRatio_;
	}

	template <std::size_t N>
	double rrtSearch<N>::getTimeout(){
		return this->timeout_;
	}


	// =================Operator overload==============================
	template <std::size_t N>
	std::ostream &operator<<(std::ostream &os, rrtSearch<N> &rrtplanner){   // 进行了这个类的初始化
        os << "========================INFO========================\n";
        os << "[Planner INFO]: RRT planner \n";
        os << "[Connect Ratio]: " << rrtplanner.getConnectGoalRatio() << "\n";
        // os << "[Start/Goal]:  " <<  rrtplanner.getStart() << "=>" <<  rrtplanner.getGoal() << "\n";
        std::vector<double> collisionBox = rrtplanner.getCollisionBox();
        os << "[Collision Box]: " << collisionBox[0] << " " << collisionBox[1] << " " <<  collisionBox[2] << "\n";
        os << "[Map Res]: " << rrtplanner.getMapRes() << "\n";
        os << "[Timeout]: " << rrtplanner.getTimeout() << "\n";
        os << "====================================================";
        return os;
    }

	template <std::size_t N>
    void rrtSearch<N>::init()
    {
		/* ---------- map params ---------- */
		std::cout<<"cc"<<std::endl;
		this->inv_resolution_ = 1.0 / this->resolution_;
		edt_environment_->sdf_map_->getRegion(origin_, map_size_3d_);

		cout << "origin_: " << origin_.transpose() << endl;
		cout << "map size: " << map_size_3d_.transpose() << endl;  // get map

		diff_matrix = MatrixN::Zero();
		for (int i = 0; i < P_; i++)
		{
			diff_matrix(i, i + 1) = i + 1;
		}
		diff_matrix = diff_matrix /this->ts;
		bspline_basis_matrix = computeBasisMatrix_Bspline();
		bezier_basis_matrix = computeBasisMatrix_Bezier();
		bspline2bezier_matrix = bezier_basis_matrix.inverse() * bspline_basis_matrix;
		// bspline2bezier_diff1_matrix = computeBasisMatrix_Bezier(1).inverse() * diff_matrix * bspline_basis_matrix;
		// bspline2bezier_diff2_matrix = computeBasisMatrix_Bezier(2).inverse() * diff_matrix * diff_matrix * bspline_basis_matrix;

		separator_solver_ = new separator::Separator();

		Eigen::MatrixXd trans_matrix(3,4);
		trans_matrix<<-1,1,0,0,
						0 ,-1,1,0,
						0,0,-1,1;
		
		// Eigen::MatrixXd bspline_diff1_matrix1 = (computeBasisMatrix_Bezier(1).inverse()).block(0,0,3,3)*computeBasisMatrix_Bspline(1).block(0,0,3,3)*trans_matrix;
		Eigen::MatrixXd bspline_diff1_matrix1 = trans_matrix;
		this->bspline_diff1_matrix = bspline_diff1_matrix1/this->ts;

		Eigen::MatrixXd transacc_matrix(2,4);
		transacc_matrix<<1,-2,1,0,
						 0 ,1,-2,1;
		Eigen::MatrixXd bspline_diff2_matrix1 = transacc_matrix;//(computeBasisMatrix_Bezier(2).inverse()).block(0,0,2,2)*computeBasisMatrix_Bspline(1).block(0,0,2,2)*transacc_matrix;
		bspline_diff2_matrix = (bspline_diff2_matrix1/this->ts)/this->ts;



		// std::cout<<"computeBasisMatrix_Bspline():"<<computeBasisMatrix_Bspline(2)<<std::endl;
		// std::cout<<"computeBasisMatrix_Bezier(1):"<<computeBasisMatrix_Bezier(1)<<std::endl;
		// std::cout<<"diff_matrix:"<<diff_matrix<<std::endl;
		// std::cout<<"bspline2bezier_matrix:"<<bspline2bezier_matrix<<std::endl;
		// std::cout<<"bspline2bezier_diff1_matrix :"<<bspline2bezier_diff1_matrix <<std::endl;
		// std::cout<<"bspline2bezier_diff1_matrix1 :"<<bspline2bezier_diff1_matrix1 <<std::endl;
		// std::cout<<"bspline2bezier_diff2_matrix :"<<bspline2bezier_diff2_matrix<<std::endl;
		// std::cout<<"bspline2bezier_diff2_matrix1 :"<<bspline2bezier_diff2_matrix1<<std::endl;

		startVisModule();


		this->bspline_dist = this->ts*max_vel_;
		std::cout<<"bspline_dist"<<bspline_dist<<std::endl;
		std::random_device rd;
		this->mt = std::mt19937(rd());
	}	

	template <std::size_t N>
	void rrtSearch<N>::reset()
	{ 
		std::unordered_map<KDTree::Point<N>, KDTree::Point<N>, KDTree::PointHasher> parent_temp;
		this->parent_ = parent_temp;
		this->parent_end = parent_temp;
		KDTree::KDTree<N, int> ktree_temp;
		this->ktree_=ktree_temp;
		Eigen::MatrixXd result_temp; 
		this->result_ = result_temp;
		this->n_result.clear();
        this->d_result.clear();
	}

	template <std::size_t N>
	ConvexHullsOfCurves_Std rrtSearch<N>::gethulls()
	{
		return this->hulls_curve;
	}

	template <std::size_t N>
	std::vector<double> rrtSearch<N>::getndresult()
	{
		std::vector<double> ndresult;
		for(int i = 0;i<n_result.size();i++ )
	    {
			ndresult.push_back(n_result[i](0));
			ndresult.push_back(n_result[i](1));
			ndresult.push_back(n_result[i](2));
			// std::cout<<"n:"<<n_result[i]<<std::endl;
	    }
	    for(int i = 0;i<d_result.size();i++ )
	    {
		    ndresult.push_back(d_result[i]);
			// std::cout<<"d:"<<d_result[i]<<std::endl;
	    }
	    return ndresult;
	}

	template <std::size_t N>
	void rrtSearch<N>::setEnvironment(const EDTEnvironment::Ptr& env)
	{
	    this->edt_environment_ = env;
	}

	template <std::size_t N>
	void rrtSearch<N>::setBoxDetector(const BoxDetector::Ptr &detector_)
	{
	    this->box_detector_ = detector_;
	}

	template <std::size_t N>
	uint64_t rrtSearch<N>::C_n_k(uint64_t n, uint64_t k) {
	   if (k > n) {
		return 0;
	   }
	   uint64_t r = 1;
	   for (uint64_t d = 1; d <= k; ++d) {
		r *= n--;
		r /= d;
	   }
	    return r;
	}

	template <std::size_t N>
	const typename rrtSearch<N>::MatrixN rrtSearch<N>::computeBasisMatrix_Bspline(int diff_degree) {
		if (diff_degree > P_) {
			cerr << "rrtSearch<N>::MatrixN rrtSearch<N>::computeBasisMatrix_Bspline: diff_degree  = " << diff_degree
				<< endl;
			return rrtSearch<N>::MatrixN::Zero();
		}

		typename rrtSearch<N>::MatrixN m;
		m.setZero();
		for (int i = 0; i < P_ + 1 - diff_degree; ++i) {
			for (int j = 0; j < P_ + 1 - diff_degree; ++j) {
			double sum = 0;

			for (int s = j; s < P_ + 1 - diff_degree; ++s) {
				sum += std::pow(-1.0, s - j) * C_n_k(P_ + 1 - diff_degree, s - j)
					* std::pow(P_ + 1 - diff_degree - s - 1.0, P_ + 1 - diff_degree - 1.0 - i);
			}
			m(i, j) = C_n_k(P_ + 1 - diff_degree - 1, P_ + 1 - diff_degree - 1 - i) * sum;
			}
		}

		uint64_t factorial = 1.0;
		for (int i = 2; i < P_ + 1 - diff_degree; ++i) {
			factorial *= i;
		}
		m = m / factorial;
		if (diff_degree > 0) {
			for (int i = 0; i < diff_degree; i++)
			m(P_ - i, P_ - i) = 1.0;
		}
		return m.template cast<double>();

	}

	template <std::size_t N>
	const typename rrtSearch<N>::MatrixN rrtSearch<N>::computeBasisMatrix_Bezier(int diff_degree) {
		if (diff_degree > P_) {
			cerr << "MatrixN computeBasisMatrix_Bezier: diff_degree  = " << diff_degree
				<< endl;
			return rrtSearch<N>::MatrixN::Zero();
		}

		typename rrtSearch<N>::MatrixN m;
		m.setZero();

		for (int i = 0; i < P_ + 1 - diff_degree; ++i) {
			for (int j = 0; j < P_ + 1 - diff_degree; ++j) {
			if (i < j)
				m(i, j) = 0;
			else {
				m(i, j) = std::pow(-1.0, i - j) * C_n_k(P_ - diff_degree, j) * C_n_k(P_ - diff_degree - j, i - j);
			}
			}
		}

		if (diff_degree > 0) {
			for (int i = 0; i < diff_degree; i++)
			m(P_ - i, P_ - i) = 1.0;
		}

		return (m).template cast<double>();
	}

	template <std::size_t N>
	bool rrtSearch<N>::SetLocalControlPoints(KDTree::Point<N>  qNew, KDTree::Point<N>  qNear)
	{
		Eigen::Vector3d ptemp;
		KDTree::Point<N> ptr = qNear;
		for (int i = 0; i < P_; i++) 
		{
			KdPoint2Eigen(ptr,ptemp);
			this->local_control_points.row(P_ - 1 - i) = ptemp;
			if (ptr != this->emptyToken_)
			   ptr = this->parent_[ptr];     
			else
			{
			   std::cout<<"not enghou point"<<std::endl;
			}
		}
		KdPoint2Eigen(qNew,ptemp);
		this->local_control_points.row(P_) = ptemp;

		Eigen::Matrix<double, 1, 4> derivate;
		derivate << 0.0, 0.0, 0.0, 6.0;
		Eigen::Matrix<double, 1, 4> derivate_tmp = derivate*this->bspline_basis_matrix;
		Eigen::Vector3d cost = derivate_tmp*local_control_points;
		
		double maxVal = cost.cwiseAbs().maxCoeff();

		double change_cost = abs(maxVal - max_jerk_last);

        max_jerk_last = maxVal;
		// if(maxVal<0.1)
		// {
           std::cout<<"cost:"<<change_cost<<std::endl;
		// }
		
		if(change_cost>this->max_jerk)
		{
			return false;
		}
		// std::cout<<"local_control_points:"<<local_control_points<<std::endl;
		return true;

	}

	template <std::size_t N>
	bool rrtSearch<N>::getpath(std::vector<KDTree::Point<N>>& plan,std::vector<std::pair<int,int>> index_opt)
	{

		this->result_ = Eigen::MatrixXd(3,plan.size());
		for(int i=0;i<plan.size();i++)
		{
			Eigen::Vector3d pt;
			KdPoint2Eigen(plan[i],pt);
			this->result_.col(i) = pt;
		}

		std::vector<Eigen::Vector3d> q;
		for(int j =0;j<this->result_.cols();j++)
		{
			q.push_back(this->result_.col(j));
		}

        // std::vector<Eigen::Vector3d> point_set;
		// Eigen::Matrix<double,4,3> Q;
		// for(int i=0;i<q.size()-3;i++)
		// {
        //    Q.row(0) = q[i];
		//    Q.row(1) = q[i+1];
        //    Q.row(2) = q[i+2];
		//    Q.row(3) = q[i+3];

		//    Eigen::Vector3d point;
		//    Eigen::VectorXd t(4);
		//    t<<1,0,0,0;
		//    point = t.transpose()*this->bspline_basis_matrix*Q;
		//    point_set.push_back(point);
		// }
		// point_set.push_back(q.back());
        
        // std::vector<Eigen::Vector3d> start_end_derivative;
		// Eigen::Vector3d zero =Eigen::Vector3d(0,0,0);
		// start_end_derivative.push_back(zero);
		// start_end_derivative.push_back(zero);
        // start_end_derivative.push_back(zero);
        // start_end_derivative.push_back(zero); 
        // Eigen::MatrixXd ctrl_pts;
        // parameterizeToBspline(this->ts,point_set,start_end_derivative,ctrl_pts);
		// std::vector<Eigen::Vector3d> q_end;
		// for(int j =0;j<ctrl_pts.cols();j++)
		// {
		// 	q_end.push_back(ctrl_pts.col(j));
		// }

		bool is_fea = checkFeasAndFillND(q,this->n_result,this->d_result,index_opt);
		if(!is_fea)
		{
			return false;
		}

	    // visualization
		if (this->visPath_){
			this->updatePathVisVec(plan);
			this->pathVisMsg_.markers = this->pathVisVec_;
		}
		return true;

	}

    template <std::size_t N>
	void rrtSearch<N>::parameterizeToBspline(const double &ts, const vector<Eigen::Vector3d> &point_set,
                                             const vector<Eigen::Vector3d> &start_end_derivative,
                                             Eigen::MatrixXd &ctrl_pts)
    {
    if (ts <= 0)
    {
      cout << "[B-spline]:time step error." << endl;
      return;
    }

    if (point_set.size() <= 3)
    {
      cout << "[B-spline]:point set have only " << point_set.size() << " points." << endl;
      return;
    }

    if (start_end_derivative.size() != 4)
    {
      cout << "[B-spline]:derivatives error." << endl;
    }

    int K = point_set.size();

    // write A
    Eigen::Vector3d prow(3), vrow(3), arow(3);
    prow << 1, 4, 1;
    vrow << -1, 0, 1;
    arow << 1, -2, 1;

    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(K + 4, K + 2);

    for (int i = 0; i < K; ++i)
      A.block(i, i, 1, 3) = (1 / 6.0) * prow.transpose();

    A.block(K, 0, 1, 3) = (1 / 2.0 / ts) * vrow.transpose();
    A.block(K + 1, K - 1, 1, 3) = (1 / 2.0 / ts) * vrow.transpose();

    A.block(K + 2, 0, 1, 3) = (1 / ts / ts) * arow.transpose();
    A.block(K + 3, K - 1, 1, 3) = (1 / ts / ts) * arow.transpose();

    //cout << "A" << endl << A << endl << endl;

    // write b
    Eigen::VectorXd bx(K + 4), by(K + 4), bz(K + 4);
    for (int i = 0; i < K; ++i)
    {
      bx(i) = point_set[i](0);
      by(i) = point_set[i](1);
      bz(i) = point_set[i](2);
    }

    for (int i = 0; i < 4; ++i)
    {
      bx(K + i) = start_end_derivative[i](0);
      by(K + i) = start_end_derivative[i](1);
      bz(K + i) = start_end_derivative[i](2);
    }

    // solve Ax = b
    Eigen::VectorXd px = A.colPivHouseholderQr().solve(bx);
    Eigen::VectorXd py = A.colPivHouseholderQr().solve(by);
    Eigen::VectorXd pz = A.colPivHouseholderQr().solve(bz);

    // convert to control pts
    ctrl_pts.resize(3, K + 2);
    ctrl_pts.row(0) = px.transpose();
    ctrl_pts.row(1) = py.transpose();
    ctrl_pts.row(2) = pz.transpose();

    // cout << "[B-spline]: parameterization ok." << endl;
  }
    
	template <std::size_t N>
	void rrtSearch<N>::getellipse(std::vector<KDTree::Point<N>>& plan)
	{
		double length = 0;
		int num_plan = plan.size();
		for(int jj=0;jj<plan.size()-1;jj++)
		{
            length = length + KDTree::Distance(plan[jj],plan[jj+1]);
	    }
		this->Cmax = length;
		this->Cmin = KDTree::Distance(plan[0],plan[num_plan-1]);
		this->center_x =  (plan[0][0]+plan[num_plan-1][0])/2;
		this->center_y =  (plan[0][1]+plan[num_plan-1][1])/2;
		this->tanta = std::atan2((plan[0][1]-plan[num_plan-1][1]),(plan[0][0]+plan[num_plan-1][0]));

		this->b_ellipse = sqrt(pow(Cmax,2)-pow(Cmin,2));
		this->a_ellipse = sqrt(pow(Cmin/2,2)+pow(b_ellipse/2,2))*2;

		this->use_ellipse ==true;

		std::cout<<"have_yuan"<<std::endl;
      
	} 

	template <std::size_t N>
	void rrtSearch<N>::initstart()
	{
		this->emptyToken_[0] = -11311; 
		this->emptyToken_[1] = -11311;
		this->emptyToken_[2] = -11311;
		// for(int i =0;i<this->start_.size();i++)
		// {
		// 	std::cout<<"aa"<<this->start_[i]<<std::endl;
		// }
		
		this->parent_[start_[P_-3]] = this->emptyToken_; // set start parent to NULL
		this->parent_[start_[P_-2]] = start_[P_-3]; // set start parent to NULL
		this->parent_[start_[P_-1]] = start_[P_-2];

		this->parent_end[start_[P_-3]] = this->emptyToken_; // set start parent to NULL
		this->parent_end[start_[P_-2]] = start_[P_-3]; // set start parent to NULL
		this->parent_end[start_[P_-1]] = start_[P_-2];

		// for (auto it = parent_.begin(); it != parent_.end(); ++it) {
        //    const KDTree::Point<N>& key = it->first;
        //    const KDTree::Point<N>& value = it->second;
        //    std::cout << "key: " << key << ", value: " << value << std::endl;
        // }
		// this->temp_parent_ = this->parent_;

	}

	template <std::size_t N>
	std::pair<int,int> rrtSearch<N>::getSearchSegment(std::vector<KDTree::Point<N>> controlpt,int segment_start)
	{
       //    std::vector<std::pair(int,int)> segment;
	   std::pair<int,int>segment = std::make_pair(0,0);
	   Eigen::Matrix<double, P_ + 1, 3> now_control_points;
	   Eigen::Vector3d ptemp;
	   KDTree::Point<N> ptr;
	   int start_index;
	   int end_index;
	   bool write_segment = false;
	   for(int i=segment_start;i<controlpt.size()-P_;i++)
	   {
		    bool coll =false;
			KdPoint2Eigen(controlpt[i],ptemp);
            now_control_points.row(0) = ptemp;
			KdPoint2Eigen(controlpt[i+1],ptemp);
            now_control_points.row(1) = ptemp;
			KdPoint2Eigen(controlpt[i+2],ptemp);
            now_control_points.row(2) = ptemp;
			KdPoint2Eigen(controlpt[i+3],ptemp);
            now_control_points.row(3) = ptemp;
			for(double t=0;t<=1;t=t+0.2)
            {
				Eigen::VectorXd pow_time(4);
				pow_time<<1,pow(t,1),pow(t,2),pow(t,3);
				Eigen::Vector3d pt = pow_time.transpose()*this->bspline_basis_matrix*now_control_points;
				coll = checkCollision(pt); // static
				if(coll)
				{
					break;
				}
			}
			if(coll == false)
			{
              coll = checkdynamicCollision(now_control_points,i*this->ts);  // time is changed
			}
			
			if(coll&&!write_segment)
			{
				start_index = i-1;//i-1
				write_segment = true;
			}
			else if(!coll&&write_segment)
			{
               end_index = i;  // i-1
			   write_segment = false;
			   segment = std::make_pair(start_index,end_index);
			   break;
			}
	   }
	   return segment;

	}


	template <std::size_t N>
	int rrtSearch<N>::search(Eigen::Matrix<double, 3, 3> init_control, Eigen::Vector3d end_pt,double time_start){
		box_detector_-> StoreAllTrack();
		this->start_.clear();
		this->reset();
		std::vector<KDTree::Point<N>> plan;
		if (this->visRRT_){
			this->RRTVisvec_.clear();
			this->RRTVisMsg_.markers = this->RRTVisvec_;
		}

		if (this->visPath_){
			this->pathVisVec_.clear();
			this->pathVisMsg_.markers = this->pathVisVec_;
		}
        
        std::vector<KDTree::Point<N>> controlpt_end;

		KDTree::Point<N> kdgoal;
		Eigen2KdPoint(kdgoal,end_pt);
		this->goal_ = kdgoal;
		this->time_start = time_start;

        std::vector<KDTree::Point<N>> init_;
        KDTree::Point<N> kdtemp;
		for(int i =0;i<P_;i++)
		{
			Eigen::Vector3d temp;
			temp = init_control.row(i);
			Eigen2KdPoint(kdtemp,temp);
			init_.push_back(kdtemp);
		}

        KDTree::Point<N> start_init = init_[2];
        int goal_status;
		KDTree::Point<N> local_goal_;
		double distance_all = KDTree::Distance(start_init, this->goal_);
		if(distance_all<=this->plan_horizon)
		{
           local_goal_ = this->goal_;
		   goal_status = 0;
		}
		else
		{
		   KDTree::Point<N> direction = this->goal_ - start_init;
		   local_goal_ = start_init + (this->plan_horizon/distance_all) * direction;
		   goal_status = 1;
		}
		// std::cout<<"goal_status"<<goal_status<<std::endl;
		std::cout<<"goal_:"<<goal_<<std::endl;
        std::cout<<"local_goal_:"<<local_goal_<<std::endl;
		std::vector<KDTree::Point<N>> controlpt;

		controlpt.push_back(init_[0]);
		controlpt.push_back(init_[1]);

        double distance_all_local = KDTree::Distance(start_init, local_goal_);
		KDTree::Point<N> direction_all = local_goal_ - start_init;
        for(double dis = 0;dis<distance_all_local;dis+=0.7*this->bspline_dist)
		{
			KDTree::Point<N> temp = start_init + (dis/distance_all_local) * direction_all;
            controlpt.push_back(temp);
		}

		// if(goal_status ==0)
		// {
			controlpt.push_back(local_goal_);
			local_goal_[2] = local_goal_[2] + 0.00001;
			controlpt.push_back(local_goal_);
			local_goal_[2] = local_goal_[2] + 0.00001;
			controlpt.push_back(local_goal_);
		// }
		// else if(goal_status ==1)
		// {
        //     controlpt.push_back(local_goal_);
		// }

        int segment_start = 0;

        int count_falg = 0;
        std::vector<std::pair<int,int>> index_opt;

		// if collision
		while(true)
		{
			count_falg++;
			if(count_falg == 100)
			{
				return NO_PATH;
			}
			// std::cout<<"count_falg"<<count_falg<<std::endl;
		    std::pair<int,int> segment = getSearchSegment(controlpt,segment_start);
		    if(segment.first == 0&&segment.second==0)
		    {
			  // write segment;
              
			  bool is_satidied = getpath(controlpt,index_opt);
			  if(!is_satidied)
			  {
				// std::cout<<"count_falg1"<<std::endl;
				return NO_PATH;
			  }
			//   std::cout<<"count_falg2"<<std::endl;
			  return REACH_HORIZON;
		    }
		    else
		    {   
			  
			  int iteration = 2;
			  while(iteration--)
			  {
			    this->start_.clear();
                // std::unordered_map<KDTree::Point<N>, KDTree::Point<N>, KDTree::PointHasher> parent_temp;
		        // this->parent_ = parent_temp;
				this->reset();

                int start_index = segment.first;
				int end_index = segment.second;
				// if(start_index<2)
				// {
				// 	std::cout<<"count_falg3"<<std::endl;
				// 	return NO_PATH;// uav in obstacle
				// }

	
				while(start_index>=4)
				{
                   KDTree::Point<N> temp_dist;
				   temp_dist = controlpt[start_index];
				   Eigen::Vector3d temp_dist_eigen(temp_dist[0],temp_dist[0],temp_dist[0]);
				   double dist =this->edt_environment_->sdf_map_->getDistance(temp_dist_eigen);
				   if(dist<1)
				   {
					start_index --;
				   }
				   else{
					break;
				   }
				}

			    this->start_.push_back(controlpt[start_index-2]);

				if(controlpt[start_index-2]==controlpt[start_index-1])
				{
					KDTree::Point<N> kdtemp;
					kdtemp = controlpt[start_index-1];
					kdtemp[2] = kdtemp[2]+0.00001;
					this->start_.push_back(kdtemp);

				}
				else{this->start_.push_back(controlpt[start_index-1]);}
				if(controlpt[start_index]==controlpt[start_index-1])
				{
					KDTree::Point<N> kdtemp;
					kdtemp = controlpt[start_index];
					kdtemp[2] = kdtemp[2]+0.00002;
					this->start_.push_back(kdtemp);
				}
				else{this->start_.push_back(controlpt[start_index]);}
                initstart();

				// KDTree::Point<N> temp_goal;
				int end_index_use = end_index+3;
				for(int p=0;p<3;p++)
				{
                   if(end_index_use == controlpt.size())
				   {
					break;
				   }
				   else
				   {
					end_index_use++;
				   }
				}
				
				this->temp_goal = controlpt[end_index_use];  //
				int num_all =  start_index - 2 + 1;

                
				KDTree::Point<N> temp_start;
				temp_start = this->start_[2];

				double min_x = std::min(temp_start[0],temp_goal[0]);
                double min_y = std::min(temp_start[1],temp_goal[1]);
				double min_z = std::min(temp_start[2],temp_goal[2]);

				double max_x = std::max(temp_start[0],temp_goal[0]);
				double max_y = std::max(temp_start[1],temp_goal[1]);
				double max_z = std::max(temp_start[2],temp_goal[2]);

				Eigen::Vector3d min_sample(min_x-2,min_y-2,min_z-3);
				Eigen::Vector3d max_sample(max_x+2,max_y+2,max_z+3);

				updateSampleRegion(min_sample,max_sample);

                bool findPath = false;
		        bool timeout = false;
		        bool findhorizon = false;
		        ros::Time startTime = ros::Time::now();
		        double dT;
		        int sampleNum = 0;

		        KDTree::Point<N> qBack;

				this->addVertex(this->start_[2]);
				while (ros::ok() and not findPath and not timeout )
			    {

                    ros::Time currentTime = ros::Time::now();
			        dT = (currentTime - startTime).toSec();
			        if (dT >= this->timeout_){
			    	   timeout = true;
					}

                  	// 1. sample:
			        KDTree::Point<N> qRand;
			        double randomValue = randomNumber(0, 1);
			        // std::cout<<"randomValue:"<<randomValue<<std::endl;
			        if (randomValue >= this->connectGoalRatio_){ // random sample trick
				      this->randomConfig(qRand);
			        }   
			        else{
				      qRand = this->temp_goal;
				      // std::cout<<"goal"<<std::endl;
			        }

				    // 2. find nearest neighbor:
			        KDTree::Point<N> qNear;
			        this->nearestVertex(qRand, qNear); // 采样的点有点问题


                    // 3. new config by steering function:
				    KDTree::Point<N> qNew;
	
				    int num_seg =6;
				    double dist = KDTree::Distance(qRand,qNear);
				    KDTree::Point<N> direction = (qRand - qNear)*(1/dist);
				    // std::cout<<"direction:"<<direction<<std::endl;

				    double dx = abs(direction[0]);
				    double dy = abs(direction[1]);
				    double dz = abs(direction[2]);// x y z 
				    // std::cout<<"bb:"<<std::endl;
				    double dmax = max(max(dx, dy), dz);// 最小距离
				    double axiedist;
			        if(dmax==dx)
			        {
				        axiedist = (0.6*bspline_dist)/direction[0];
			        }
			        else if(dmax==dy){
	    		        axiedist = (0.6*bspline_dist)/direction[1]; 
			        }
			        else if(dmax==dz){
				        axiedist = (0.6*bspline_dist)/direction[2]; 
			        }
					// Eigen::Vector3d point_dist(qNear[0],qNear[1],qNear[2]);
					// double point_ds = this->edt_environment_->sdf_map_->getDistance(point_dist);
                    // std::cout<<"axiedist:"<<axiedist<<std::endl;
			        for(int i = num_seg;i > 0;i--)
			        {
					    this->delQ_ = abs((double(i)/double(num_seg))*axiedist);
						
						// if(this->delQ_>point_ds)
						// {
						// 	continue;
						// }
					    KDTree::Point<N> qNew;
					    this->newConfig(qNear, qRand, qNew);

                        Eigen::Vector3d point_dist(qNew[0],qNew[1],qNew[2]);
					    double point_ds = this->edt_environment_->sdf_map_->getDistance(point_dist);
						std::cout<<"point_ds:"<<point_ds<<std::endl;
						if(point_ds<0.2)
						{
							continue;
						}

					    // 4. Add new config to vertex and edge:
					    bool success_set = SetLocalControlPoints(qNew,qNear);
						if(!success_set)
						{
							continue;
						}
					    // std::cout<<"qNew:"<<qNew<<std::endl;
					    // std::cout<<"delQ_:"<<delQ_<<std::endl;
					    // std::cout<<"i:"<<i<<std::endl;
				        if (!this->checkCollisionAll(qNear,num_all))
				        {   // 检查的是localpoint
					       this->addVertex(qNew);
					       this->addEdge(qNear, qNew);
				           ++sampleNum;
					
				          //   std::cout<<sampleNum<<"sampleNum"<<std::endl;
				          // 5. check whether goal has been reached
				          findPath = this->isReach(qNew); // add two points
				          if (findPath)
					      {
					        qBack = qNew;              
			   	          }
					      // visualization:
					      if (this->visRRT_){
					    	 this->updateRRTVisVec(qNew, qNear, sampleNum);
					       }
					    
				        }
			        }
				}

				cout << "[Planner INFO]: Finish planning. with sample number: " << sampleNum << endl;

		        // if(timeout)
		        // {
			    //   std::cout<<"timeout"<<std::endl;
		        // }
		        // if(findPath)
		        // {
                //   std::cout<<"findPath"<<std::endl;
		        // }
		
                // final step: back trace using the last one
		        std::vector<KDTree::Point<N>> planRaw;
		        if(timeout)
		        { 
					if(iteration>0)
					   continue;
					else
					   return NO_PATH;
		        }
		        else if (findPath)
			    {
					// controlpt_end.clear();
			       this->backTrace(qBack, planRaw);
				   int count =0;
				   for(int nn=0;nn<start_index-2;nn++)
				   {
					controlpt_end.push_back(controlpt[nn]);
					count++;
				   }

				   for(int nn=0;nn<planRaw.size();nn++)
				   {
					 controlpt_end.push_back(planRaw[nn]);
					 count++;
				   }

				   if((end_index_use)==controlpt.size())
				   {
                      controlpt_end.push_back(controlpt[end_index_use]);
					  KDTree::Point<N> temp = controlpt[end_index_use]+(0.15*this->bspline_dist/distance_all_local)*direction_all;
					  controlpt_end.push_back(temp);
					  temp = controlpt[end_index_use]+(0.3*this->bspline_dist/distance_all_local)*direction_all;
                      controlpt_end.push_back(temp);  		
				   }
				   else
				   {
                        for(int nn=end_index_use;nn<controlpt.size();nn++)
				        {
                           controlpt_end.push_back(controlpt[nn]);
						}
				   }
				   controlpt = controlpt_end;
				   controlpt_end.clear();
				   segment_start = count;
				   index_opt.push_back(std::make_pair(start_index-2,segment_start));

				   break;
		        }
			  }



                
			}

		}


	}

					

}

#endif