

#ifndef _ASTAR_H_
#define _ASTAR_H_

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
#include "detector/Boxdetector.h"

using namespace fast_planner;
namespace def_planner 
{
#define IN_CLOSE_SET 'a'
#define IN_OPEN_SET 'b'
#define NOT_EXPAND 'c'
#define inf 1 >> 30    // 把这个视为正无穷

typedef MADER_timers::Timer MyTimer;

class PathNode {
 public:
  /* -------------------- */
  Eigen::Vector3i index;    // 在总地图中的位置.
  Eigen::Vector3d position;
  double g_score = 0;
  double h_score = 0 ;  // h是启发函数,g是基本的函数,到起点的值.
  double dist;              //距离地图里面的距离
  PathNode* parent;
  char node_state;
  double f_score()
  {
    return g_score+h_score;
  }
  PathNode() {
    parent = NULL;   //记录这个点的状态.
  }
  PathNode(Eigen::Vector3i _index) { 
    index = _index;
    node_state = NOT_EXPAND;
    parent = NULL;
  }
  ~PathNode(){};
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef PathNode* PathNodePtr;

class NodeComparator {
 public:
  bool operator()(PathNodePtr node1, PathNodePtr node2) {
    return node1->f_score() > node2->f_score();
  }
};

template <typename T>
struct matrix_hash : std::unary_function<T, size_t> {
  std::size_t operator()(T const& matrix) const {
    size_t seed = 0;
    for (size_t i = 0; i < matrix.size(); ++i) {
      auto elem = *(matrix.data() + i);
      seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) +
              (seed >> 2);
    }
    return seed;
  }
};

class NodeHashTable {
 public:
//  private:
  /* data */
  std::unordered_map<Eigen::Vector3i, PathNodePtr, matrix_hash<Eigen::Vector3i>>
      openlist;
  std::unordered_map<Eigen::Vector3i, PathNodePtr, matrix_hash<Eigen::Vector3i>>
      closelist;
 public:
  NodeHashTable(/* args */) {}
  ~NodeHashTable() {}
  void insertopenlist(Eigen::Vector3i idx, PathNodePtr node) {   // 哈希表
    openlist.insert(std::make_pair(idx, node));
  }
  void insertcloselist(Eigen::Vector3i idx, PathNodePtr node) {   // 哈希表
    closelist.insert(std::make_pair(idx, node));
  }
  void open2closelist(Eigen::Vector3i idx, PathNodePtr node) {   // 哈希表
    auto iter = openlist.find(idx);
    if (iter != openlist.end()) {
        openlist.erase(iter);
    }
    insertcloselist(idx,node); 
  }

  PathNodePtr findopen(Eigen::Vector3i idx) {
    auto iter = openlist.find(idx);
    return iter == openlist.end() ? NULL : iter->second;
  }
    PathNodePtr findclose(Eigen::Vector3i idx) {
    auto iter = closelist.find(idx);
    return iter == closelist.end() ? NULL : iter->second;
  }

  void clear() {
    closelist.clear();
    openlist.clear();
  }
};

class RSAstar {
 private:
  /*typedef*/
  const static int P_ = 3;
  typedef Eigen::Matrix<double, 1, P_ + 1> VectorNT;
  typedef Eigen::Matrix<double, P_ + 1, 1> VectorN;
  typedef Eigen::Matrix<double, P_ + 1, P_ + 1> MatrixN;
  typedef std::array<MatrixN, 4> MatrixNArray;

  /* ---------- main data structure ---------- */
  std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator> openList_;
  std::unordered_map<Eigen::Vector3i, bool, matrix_hash<Eigen::Vector3i>> neighborlist_;


  PathNodePtr ***BSplineGridNodeMap;

  
  std::vector<double> all_combinations_;
  std::vector<Eigen::Vector3d> vs;
  ConvexHullsOfCurves_Std hulls_curve;
  
  /* ---------- record data ---------- */
  EDTEnvironment::Ptr edt_environment_;
  BoxDetector::Ptr box_detector_;
  separator::Separator* separator_solver_;

  /* ---------- parameter ---------- */
  /* search */
  std::array<double, 2 * (P_ + 1)> pow_inv_dt_;
  std::array<double, P_ + 1> quadratic_cost_weight;
  MatrixN bspline_basis_matrix;
  MatrixN bezier_basis_matrix;
  MatrixN bspline2bezier_matrix;
  // MatrixN bspline2bezier_diff1_matrix;
  // MatrixN bspline2bezier_diff2_matrix;
  Eigen::MatrixXd bspline_diff1_matrix;
  Eigen::MatrixXd bspline_diff2_matrix;
  MatrixN diff_matrix;
  MatrixNArray quadratic_cost_jacobian;

  std::vector<PathNodePtr> start_nodes_;
  Eigen::Vector3d end_pt_;

  PathNodePtr end_node;
  PathNodePtr start_node;

  Eigen::Matrix<double, P_ + 1, 3> local_control_points;
  NodeHashTable expanded_nodes_;

  std::vector<PathNodePtr> path_nodes_;
  std::vector<Eigen::Vector3d> result_;
  std::vector<Eigen::Vector3d> n_result;
  std::vector<double> d_result; //这个值真的很重要的，最后求解的向量
  // double tentative_gScore;
  double Time_Obstacle;


  double max_vel_,max_acc_,horizon_,tie_breaker,tie_breaker1,tie_breaker2,tie_breaker3;
  double ts;
  double time_start;
  double max_runtime_;

  /* map */
  double resolution_, inv_resolution_;
  Eigen::Vector3d origin_, map_size_3d_;
  Eigen::Vector3i min_id, max_id;
  int x_size,y_size,z_size;

  double dist_bspline;
  std::vector<std::vector<Eigen::Vector3d>> positionss;
  
 public:
  RSAstar(){};
  ~RSAstar();
    /* helper */
  Eigen::MatrixXd getctrlpoint();
  bool checkTrajDynamics1(Eigen::Matrix<double, 4, 3> local_control_points);

  const MatrixN computeBasisMatrix_Bspline(int diff_degree = 0);
  const MatrixN computeBasisMatrix_Bezier(int diff_degree = 0);
  void  clearNodeMap();

  MatrixN computeBaseCoefficients();
  double getHeu(PathNodePtr node1, PathNodePtr node2,int num);
  double getDiagHeu(PathNodePtr node1, PathNodePtr node2);
  double getDiagDis(PathNodePtr node1, PathNodePtr node2) ;
  void   setinitPathNode(Eigen::Matrix<double, 3, 3> init_control);
  std::vector<Eigen::Vector3i> Neighbor(Eigen::Vector3i current_idx,double distance);
  bool   checkTrajDynamics(PathNodePtr neighbor);
  bool CheckStaticObstacle(PathNodePtr neighbor);
  int getNumControlNow(PathNodePtr neighbor);
  void   setLocalControlPoints(PathNodePtr current);
  void initGridNodeMap();
  bool collidesWithObstacles(PathNodePtr neighbor,int num);
  bool collidesWithObstaclesGivenVertexes(const Eigen::Matrix<double, 3, 4>& last4Cps,int num);
  double getgscore(PathNodePtr tentative);
  double quadraticCost(int deriv, PathNodePtr tentative); 
  double quadraticCost(const RSAstar::MatrixN &quadratic_cost,PathNodePtr tentative); 
  bool checkFeasAndFillND(std::vector<Eigen::Vector3d>& q, std::vector<Eigen::Vector3d>& n,std::vector<double>& d);
  int search(Eigen::Matrix<double, 3, 3> init_control, Eigen::Vector3d end_pt,double time_start_);
  std::vector<PathNodePtr> retrievePath(PathNodePtr current); 
  void setParam(ros::NodeHandle& nh);
  void init();
  const  MatrixNArray computeQuadraticCostJacobian();
  const  MatrixNArray computeQuadraticCoefficients();
  void setEnvironment(const EDTEnvironment::Ptr& env);
  void setBoxDetector(const BoxDetector::Ptr &detector_);
  void reset(Eigen::Vector3d odom_pos);
  Eigen::Vector3i posToIndex(Eigen::Vector3d pt);
  void pathnode2Eigen(std::vector<PathNodePtr> &path,std::vector<Eigen::Vector3d> &result);
  uint64_t C_n_k(uint64_t n, uint64_t k);
  ConvexHullsOfCurves_Std gethulls();
  std::vector<double> getndresult();
  double getGeu(PathNodePtr node1, PathNodePtr node2);
  Eigen::Vector3d gridIndex2coord(Eigen::Vector3i index);
  PathNodePtr index2gridNodePtr(Eigen::Vector3i index);
  std::vector<std::vector<Eigen::Vector3d>> getpositionss();
  double Distoobs(PathNodePtr obsnode,int num_point);
  
  enum { REACH_HORIZON = 1, REACH_END = 2, NO_PATH = 3, RUNTIME_REACHED = 4 };
  typedef shared_ptr<RSAstar> Ptr;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   

    // namespace fast_planner
};
}

#endif

