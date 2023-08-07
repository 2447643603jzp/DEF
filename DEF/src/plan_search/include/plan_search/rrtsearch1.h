/*
*	File: rrtSearch1.h
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
		KDTree::Point<N> goal_;
		KDTree::Point<N> emptyToken_;
		KDTree::KDTree<N, int> ktree_; // KDTree
		std::unordered_map<KDTree::Point<N>, KDTree::Point<N>, KDTree::PointHasher> parent_; // for backtracking
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
        Eigen::MatrixXd bspline2bezier_diff1_matrix;
        Eigen::MatrixXd bspline2bezier_diff2_matrix;
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
		Eigen::MatrixXd result_;
		std::vector<Eigen::Vector3d> n_result;
        std::vector<double> d_result; //这个值真的很重要的，最后求解的向量

		ConvexHullsOfCurves_Std hulls_curve;

		// ellipse  // 椭圆的采样点和采样的空间。
		double Cmin,Cmax;
		double center;

	

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
		void updateSampleRegion();// helper function for update sample region
		
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
		void SetLocalControlPoints(KDTree::Point<N>  qNew, KDTree::Point<N>  qNear);

		std::vector<double> getCollisionBox();

	    // return goal conenct ratio
	    double getConnectGoalRatio();

	    // return timeout
	    double getTimeout();

        void initstart();

		bool getpath(std::vector<KDTree::Point<N>>& planRaw, std::vector<KDTree::Point<N>>& plan,int num =0);

		uint64_t C_n_k(uint64_t n, uint64_t k);

		bool checkTrajDynamics();
		bool checkTrajDynamics1(Eigen::Matrix<double,4,3> Q); 

		bool collidesWithObstaclesGivenVertexes(const Eigen::Matrix<double, 3, 4>& last4Cps,int num);

		bool CheckDynamicObstacle(int num);

		bool checkCollisionAllEnd( KDTree::Point<N> q1, KDTree::Point<N> q2);

		bool checkCollisionAll(const KDTree::Point<N> qNear);

		int getNumPoint(const KDTree::Point<N> qNear);

		bool CheckStaticObstacle(int num);
		ConvexHullsOfCurves_Std gethulls();
		std::vector<double> getndresult();
		bool checkFeasAndFillND(std::vector<Eigen::Vector3d> &q, std::vector<Eigen::Vector3d>& n,
                                       std::vector<double>& d);

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
                                       std::vector<double>& d)
    {
       int num_of_segments_ = q.size() - P_;
       int num = num_of_segments_*box_detector_->getnumtrack();
       n.resize(std::max(num, 0), Eigen::Vector3d::Zero());
       d.resize(std::max(num, 0), 0.0);
	   this->hulls_curve = box_detector_-> getAllIntervalnHull(time_start,ts,num_of_segments_);
       if(box_detector_->getnumtrack() == 0)
       {
          return true;
       }
       
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
            if(checkTrajDynamics1(last4Cps.transpose()))
            {
			   return false;
            }

          }
        }
        
        return true;
    }

	template <std::size_t N>
	bool rrtSearch<N>::checkTrajDynamics1(Eigen::Matrix<double,4,3> Q) {
        Eigen::Matrix<double, P_ + 1, 3> bspline_diff0_control_points;
        Eigen::Matrix<double, P_, 3> bezier_diff1_control_points;
        Eigen::Matrix<double, P_ - 1, 3> bezier_diff2_control_points;

        bspline_diff0_control_points = Q;
        bezier_diff1_control_points = (bspline2bezier_diff1_matrix * bspline_diff0_control_points);//.block(0, 0, P_, 3);
        bezier_diff2_control_points = (bspline2bezier_diff2_matrix * bspline_diff0_control_points);//.block(0, 0, P_ - 1, 3);
        if (bezier_diff1_control_points.maxCoeff() > this->max_vel_ ||   //返回矩阵中最大和最小值
            bezier_diff1_control_points.minCoeff() < -this->max_vel_ ||
            bezier_diff2_control_points.maxCoeff() > this->max_acc_ ||
            bezier_diff2_control_points.minCoeff() < -this->max_acc_
        ) {
           return true;//Not dynamically feasible
        }
        return false;//Dynamically feasible
    }

	template <std::size_t N>
	bool rrtSearch<N>::isReach(const KDTree::Point<N>& q){
		return KDTree::Distance(q, this->goal_) <= this->dR_; // 是那个重要的直。
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
	void rrtSearch<N>::updateSampleRegion(){
	    this->sampleRegion_[0] = this->min_id(0);
		this->sampleRegion_[1] = this->max_id(0);
		this->sampleRegion_[2] = this->min_id(1);
		this->sampleRegion_[3] = this->max_id(1);
		this->sampleRegion_[4] = 0;//this->min_id(2);
		this->sampleRegion_[5] = this->max_id(2);
		std::cout<<"vv"<<std::endl;
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
	bool rrtSearch<N>::checkCollisionAllEnd( KDTree::Point<N> q1,  KDTree::Point<N> q2)
	{
		// 把所有需要检查的都进行检查
		KDTree::Point<N> ptr = q1;
        Eigen::Vector3d ptemp;
		for (int i = 0; i < P_; i++) 
        {
          KdPoint2Eigen(ptr,ptemp);
          this->local_control_points.row(P_ - 1 - i) = ptemp;
          if (ptr != this->emptyToken_)
             ptr = this->parent_end[ptr];     
          else
          {
            std::cout<<"not enghou point"<<std::endl;
          }
        }
        KdPoint2Eigen(q2,ptemp);
        this->local_control_points.row(P_) = ptemp;
  
	
        int segment = getNumPoint(q1) - P_+1;
        bool not_satified;

		/*step1 checkTrajDynamics*/
        not_satified = checkTrajDynamics();
		if(not_satified)
		{
			return true;
		}

		/*step1, check static env*/
		not_satified = CheckStaticObstacle(segment);
		if(not_satified)
		{
			return true;
		}

		/*step2 check dynamic env*/
		not_satified = CheckDynamicObstacle(segment);
		if(not_satified)
		{
			return true;
		}

		return false;
      

	}

	template <std::size_t N>
	bool rrtSearch<N>::checkCollisionAll(const KDTree::Point<N> qNear){
		// 把所有需要检查的都进行检查
		// std::cout<<"seg:"<<std::endl;
        int segment = getNumPoint(qNear) - P_+1;
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
		not_satified = CheckStaticObstacle(segment);
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
	bool rrtSearch<N>::CheckStaticObstacle(int num) {
        Eigen::Matrix<double, P_ + 1, 3> Q;
        Q = this->local_control_points;

        double t_init = time_start + this->ts*(num-1);
        double t_end  = t_init + this->ts;
        int n=5;
        for(double time=t_init;time<=t_end;time=time+double(this->ts)/n)
        {
           double t_use = (time - t_init)/this->ts;
           Eigen::VectorXd t_pow(4);
           t_pow<< 1,pow(t_use,1),pow(t_use,2),pow(t_use,3);
           Eigen::Vector3d point =t_pow.transpose()*this->bspline_basis_matrix*Q;
      
           if(this->edt_environment_->sdf_map_->getInflateOccupancy(point) == 1)
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
       last4Cps = (bspline2bezier_matrix*bspline_diff0_control_points).transpose();
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
	bool rrtSearch<N>::checkTrajDynamics() {
        Eigen::Matrix<double, P_ + 1, 3> bspline_diff0_control_points;
        Eigen::Matrix<double, P_, 3> bezier_diff1_control_points;
        Eigen::Matrix<double, P_ - 1, 3> bezier_diff2_control_points;

        bspline_diff0_control_points = local_control_points;
        bezier_diff1_control_points = (bspline2bezier_diff1_matrix * bspline_diff0_control_points);//.block(0, 0, P_, 3);
        bezier_diff2_control_points = (bspline2bezier_diff2_matrix * bspline_diff0_control_points);//.block(0, 0, P_ - 1, 3);
        //  std::cout<<"coffe:"<<bezier_diff1_control_points.maxCoeff()<<" "<<bezier_diff1_control_points.minCoeff()<<" "<<bezier_diff2_control_points.maxCoeff()<<" "<<bezier_diff2_control_points.minCoeff()<<std::endl;

        if (bezier_diff1_control_points.maxCoeff() > this->max_vel_ ||   //返回矩阵中最大和最小值
            bezier_diff1_control_points.minCoeff() < -this->max_vel_ ||
            bezier_diff2_control_points.maxCoeff() > this->max_acc_ ||
            bezier_diff2_control_points.minCoeff() < -this->max_acc_
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
		Eigen::MatrixXd bspline2bezier_diff1_matrix1 = (computeBasisMatrix_Bezier(1).inverse()).block(0,0,3,3)*computeBasisMatrix_Bspline(1).block(0,0,3,3)*trans_matrix;
		bspline2bezier_diff1_matrix = bspline2bezier_diff1_matrix1/this->ts;
		Eigen::MatrixXd transacc_matrix(2,4);
		transacc_matrix<<1,-2,1,0,
						 0 ,1,-2,1;
		Eigen::MatrixXd bspline2bezier_diff2_matrix1 = transacc_matrix;//(computeBasisMatrix_Bezier(2).inverse()).block(0,0,2,2)*computeBasisMatrix_Bspline(1).block(0,0,2,2)*transacc_matrix;
		bspline2bezier_diff2_matrix = (bspline2bezier_diff2_matrix1/this->ts)/this->ts;
		std::cout<<"computeBasisMatrix_Bspline():"<<computeBasisMatrix_Bspline(2)<<std::endl;
		std::cout<<"computeBasisMatrix_Bezier(1):"<<computeBasisMatrix_Bezier(1)<<std::endl;
		std::cout<<"diff_matrix:"<<diff_matrix<<std::endl;
		std::cout<<"bspline2bezier_matrix:"<<bspline2bezier_matrix<<std::endl;
		std::cout<<"bspline2bezier_diff1_matrix :"<<bspline2bezier_diff1_matrix <<std::endl;
		std::cout<<"bspline2bezier_diff1_matrix1 :"<<bspline2bezier_diff1_matrix1 <<std::endl;
		std::cout<<"bspline2bezier_diff2_matrix :"<<bspline2bezier_diff2_matrix<<std::endl;
		std::cout<<"bspline2bezier_diff2_matrix1 :"<<bspline2bezier_diff2_matrix1<<std::endl;

		startVisModule();


		this->bspline_dist = this->ts*max_vel_;
		std::cout<<"bspline_dist"<<bspline_dist<<std::endl;
		std::random_device rd;
		this->mt = std::mt19937(rd());
	}	

	template <std::size_t N>
	void rrtSearch<N>::reset()
	{
		this->edt_environment_->sdf_map_->getMinMaxId(this->min_id, this->max_id);

		std::cout<<"oo"<<std::endl; 
		updateSampleRegion();
		std::cout<<"pp"<<std::endl;
		this->start_.clear();
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
	void rrtSearch<N>::SetLocalControlPoints(KDTree::Point<N>  qNew, KDTree::Point<N>  qNear)
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
		// std::cout<<"local_control_points:"<<local_control_points<<std::endl;
		return;

	}

	template <std::size_t N>
	bool rrtSearch<N>::getpath(std::vector<KDTree::Point<N>>& planRaw, std::vector<KDTree::Point<N>>& plan,int num )
	{
		std::cout<<"findho2"<<std::endl;
		std::cout<<"planRaw:"<<planRaw.size()<<std::endl;
		// this->shortcutWaypointPaths(planRaw, plan); // 这里还有一个操作，最后三个控制点 end_pt_的树脂。
		plan = planRaw;
        std::cout<<"findho1"<<std::endl;
		std::cout<<"planRaw:"<<planRaw.size()<<std::endl;
		this->result_ = Eigen::MatrixXd (3,plan.size());
		if(num ==1)
		{
			this->result_ = Eigen::MatrixXd (3,plan.size()+2);
		}
		std::cout<<"findho3"<<std::endl;
		int i;
		for(i=0;i<plan.size();i++)
		{
			Eigen::Vector3d pt;
			KdPoint2Eigen(plan[i],pt);
			this->result_.col(i) = pt;
		}
		std::cout<<"findho5"<<std::endl;
		if(num ==1)
		{
			Eigen::Vector3d pt;
			KdPoint2Eigen(plan[plan.size()-1],pt);
			std::cout<<"findho8"<<std::endl;
			pt(2)=pt(2)+0.0001;
			this->result_.col(i) = pt;
			pt(2)=pt(2)+0.0002;
			this->result_.col(i+1) = pt;
		}
        std::cout<<"findho4"<<std::endl;
		std::vector<Eigen::Vector3d> q;
		for(int j =0;j<this->result_.cols();j++)
		{
			q.push_back(this->result_.col(j));
		}
		bool is_fea = checkFeasAndFillND(q,this->n_result,this->d_result);
		std::cout<<"findh6"<<std::endl;
		if(!is_fea)
		{
			return false;
		}

	    // visualization
		std::cout<<"findho5"<<std::endl;
		if (this->visPath_){
			this->updatePathVisVec(plan);
			this->pathVisMsg_.markers = this->pathVisVec_;
		}
		return true;

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

		for (auto it = parent_.begin(); it != parent_.end(); ++it) {
           const KDTree::Point<N>& key = it->first;
           const KDTree::Point<N>& value = it->second;
           std::cout << "key: " << key << ", value: " << value << std::endl;
        }

	}


	template <std::size_t N>
	int rrtSearch<N>::search(Eigen::Matrix<double, 3, 3> init_control, Eigen::Vector3d end_pt,double time_start){
		box_detector_-> StoreAllTrack();
		std::vector<KDTree::Point<N>> plan;
		if (this->visRRT_){
			this->RRTVisvec_.clear();
			this->RRTVisMsg_.markers = this->RRTVisvec_;
		}

		if (this->visPath_){
			this->pathVisVec_.clear();
			this->pathVisMsg_.markers = this->pathVisVec_;
		}

		
		KDTree::Point<N> kdtemp;
		for(int i =0;i<P_;i++)
		{
			Eigen::Vector3d temp;
			temp = init_control.row(i);
			temp(2) = temp(2)+i*0.0001;
			Eigen2KdPoint(kdtemp,temp);
			this->start_.push_back(kdtemp);
			std::cout<<"start:"<<kdtemp<<std::endl;
		}
		Eigen2KdPoint(kdtemp,end_pt);
		this->goal_ = kdtemp;
		this->time_start = time_start;

		initstart();
		this->start_use = this->start_[P_-1]; 
		// return NO_PATH;

        std::cout<<"this->start_.size()"<<this->start_.size()<<std::endl;
		std::cout<<"this->parent.size()"<<this->parent_.size()<<std::endl;
		std::cout<<"startuse:"<<this->start_use<<std::endl;

		bool findPath = false;
		bool timeout = false;
		bool findhorizon = false;
		ros::Time startTime = ros::Time::now();
		double dT;
		int sampleNum = 0;
		KDTree::Point<N> qBack;

		cout << "[Planner INFO]: Start planning!" << endl;
		double nearestDistance = std::numeric_limits<double>::max();  // if cannot find path to goal, find nearest way to goal
		KDTree::Point<N> nearestPoint = this->start_use;
		double currentDistance = KDTree::Distance(nearestPoint, this->goal_);
		this->addVertex(this->start_use);              // 我可以设置一个迭代次数吗。
		while (ros::ok() and not findPath and not timeout and not findhorizon){	
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
				qRand = this->goal_;
				// std::cout<<"goal"<<std::endl;
			}
            // std::cout<<"qRand:"<<qRand<<std::endl;

			// 2. find nearest neighbor:
			KDTree::Point<N> qNear;
			this->nearestVertex(qRand, qNear); // 采样的点有点问题
			// std::cout<<"qRand:"<<qRand[0]<<qRand[1]<<qRand[2]<<std::endl;
            // std::cout<<"qNear:"<<qNear<<std::endl;
			// 3. new config by steering function:
			bool exit = false;
			KDTree::Point<N> qNew;
  
			int num_seg =10;
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
				axiedist = (0.5*bspline_dist)/direction[0];
			}
			else if(dmax==dy){
	    		axiedist = (0.5*bspline_dist)/direction[1]; 
			}
			else if(dmax==dz){
				axiedist = (0.5*bspline_dist)/direction[2]; 
			}
            // std::cout<<"axiedist:"<<axiedist<<std::endl;
			for(int i = num_seg;i > 0;i--)
			{
			    this->delQ_ = abs((double(i)/double(num_seg))*axiedist);
			    KDTree::Point<N> qNew;
			    this->newConfig(qNear, qRand, qNew);

                // std::cout<<"aa:"<<std::endl;   // 怎么限制他的这个采样的范围。
			    // 4. Add new config to vertex and edge:
			    SetLocalControlPoints(qNew,qNear);
			    // std::cout<<"qNew:"<<qNew<<std::endl;
			    // std::cout<<"delQ_:"<<delQ_<<std::endl;
		    	// std::cout<<"i:"<<i<<std::endl;
				if ( !this->checkCollisionAll(qNear))
				{   // 检查的是localpoint
					this->addVertex(qNew);
					this->addEdge(qNear, qNew);
				    ++sampleNum;
					
				    std::cout<<sampleNum<<"sampleNum"<<std::endl;
				    // 5. check whether goal has been reached
				    findPath = this->isReach(qNew);
				    if (findPath)
					{
					   qBack = qNew;              
			   	    }
				    else
					{
						currentDistance = KDTree::Distance(qNew, this->goal_);
						if (currentDistance < nearestDistance){
							nearestDistance = currentDistance;
							nearestPoint = qNew;
						}
                        double distance2start = KDTree::Distance(nearestPoint, this->start_use);
						if(distance2start > this->plan_horizon)
						{
						    findhorizon = true;
						}	
					}
					// visualization:
					if (this->visRRT_){
						this->updateRRTVisVec(qNew, qNear, sampleNum);
					}
					exit = true; 
				}
				if(exit)
				{
                   break;
				}
			}
		}
		cout << "[Planner INFO]: Finish planning. with sample number: " << sampleNum << endl;
         
		if(timeout)
		{
			std::cout<<"timeout"<<std::endl;
		}
		if(findhorizon)
		{
           std::cout<<"findhorizon"<<std::endl;
		}
		if(findPath)
		{
            std::cout<<"findPath"<<std::endl;
		}
		// final step: back trace using the last one
		std::vector<KDTree::Point<N>> planRaw;
		if(timeout)
		{
			this->backTrace(nearestPoint, planRaw);
			if (planRaw.size() <=6){
					plan = planRaw;
					cout << "[Planner INFO]: TIMEOUT! Start position might not be feasibl11e!!" << endl;
					return NO_PATH;
				}
				else{
			bool satified = getpath(planRaw,plan);
			if(!satified)
			{
				return NO_PATH;
			}
					cout << "[Global Planner INFO]: TIMEOUT1!"<< "(>" << this->timeout_ << "s)" << ", Return closest path. Distance: " << nearestDistance << " m." << endl;
				}
			return RUNTIME_REACHED;
		}
		else if(findhorizon)
		{
			this->backTrace(nearestPoint, planRaw);
			std::cout<<"findho"<<std::endl;
			    if (planRaw.size() <= 6){
					plan = planRaw;
					cout << "[Planner INFO]: distance! Start position might not be feasible!!" << endl;
					return NO_PATH;
				}
				else{
			       bool satified = getpath(planRaw,plan);
			       if(!satified)
			       {
				      return NO_PATH;
			       }
					cout << "[Planner INFO]: distance1!"<< "(>" << this->timeout_ << "s)" << ", Return closest path. Distance: " << nearestDistance << " m." << endl;
			      return REACH_HORIZON;
				}
			
		}
		else if (findPath){
			this->backTrace(qBack, planRaw);
		    bool satified = getpath(planRaw,plan,1);
		    if(!satified)
		    {
     		   return NO_PATH;
		    }
		    cout << "[Planner INFO]: path found! Time: " << dT << "s."<< endl;
		    return REACH_END;
		}

	}
}

#endif