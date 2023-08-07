/**
* This file is part of Fast-Planner.
*
* Copyright 2019 Boyu Zhou, Aerial Robotics Group, Hong Kong University of Science and Technology, <uav.ust.hk>
* Developed by Boyu Zhou <bzhouai at connect dot ust dot hk>, <uv dot boyuzhou at gmail dot com>
* for more information see <https://github.com/HKUST-Aerial-Robotics/Fast-Planner>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* Fast-Planner is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Fast-Planner is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with Fast-Planner. If not, see <http://www.gnu.org/licenses/>.
*/



#ifndef _BSPLINE_OPTIMIZER_H_
#define _BSPLINE_OPTIMIZER_H_

#include <Eigen/Eigen>
#include <plan_env/edt_environment.h>
#include <ros/ros.h>
#include <plan_env/sdf_map.h>
#include <detector/Boxdetector.h>
#include <nlopt.hpp>
#include <iostream>
#include <fstream>


using namespace nlopt;

namespace fast_planner {


class BsplineOptimizer {

public:

  BsplineOptimizer() 
  {
  }
  ~BsplineOptimizer() {}

  /* main API */
  void            setEnvironment(const EDTEnvironment::Ptr& env);
  void            setParam(ros::NodeHandle& nh);
  Eigen::MatrixXd BsplineOptimizeTraj(const Eigen::MatrixXd& points, const double& ts, // N*3 row
                                                      ConvexHullsOfCurves_Std &hulls_curve, int max_num_id,
                                                      int max_time_id,double time_start,std::vector<double> nd_result,
                                                      std::vector<std::vector<Eigen::Vector3d>> positionss);


  // required inputs
  void setControlPoints(const Eigen::MatrixXd& points);
  void setBsplineInterval(const double& ts,double time_start);
  void setTerminateCond(const int& max_num_id, const int& max_time_id);
  void setHulls(ConvexHullsOfCurves_Std &hulls);
  void setKnots();
  void initializeNumOfConstraints();
  void optimize();
  template <class T>
  void x2qnd(T &x, std::vector<Eigen::Vector3d> &q, std::vector<Eigen::Vector3d> &n, std::vector<double> &d);
  void toGradDiffConstraintsDiffVariables(int var_gindex, const Eigen::Vector3d &tmp, double *grad, int r,int nn);
  void toGradSameConstraintDiffVariables(int var_gindex, const Eigen::Vector3d &tmp, double *grad, int r,int nn);
  void assignValueToGradConstraints(int var_gindex, const double &tmp, double *grad, int r, int nn);
  int gIndexQ(int i);
  int gIndexN(int i);
  int gIndexD(int i);
  bool isADecisionCP(int i);
  void assignEigenToVector(double *result, int var_gindex, const Eigen::Vector3d &tmp);
  void computeConstraints(unsigned m, double *constraints, unsigned nn, double *grad,
                                     const std::vector<Eigen::Vector3d> &q, const std::vector<Eigen::Vector3d> &n,
                                     const std::vector<double> &d);
  template <class T>
  bool areTheseConstraintsFeasible(const T &constraints);
  double computeObjFunctionJerk( std::vector<double> &grad, std::vector<Eigen::Vector3d> &q,
                                           std::vector<Eigen::Vector3d> &n, std::vector<double> &d);
  template <class T>
  bool isFeasible(const T x);
  bool isFeasible(const std::vector<Eigen::Vector3d> &q, const std::vector<Eigen::Vector3d> &n,const std::vector<double> &d);
  void transformVelBSpline2bezier(const Eigen::Matrix<double, 3, 3> &Qbs,Eigen::Matrix<double, 3, 3> &Qmv);
  void transformPosBSpline2bezier(const Eigen::Matrix<double, 4,3> &Qbs,Eigen::Matrix<double, 4, 3> &Qmv);
  void setLocalControlPoint(Eigen::Matrix<double, 4, 3> Q);
  Eigen::Vector3d current_spline_evaluate(double t_use);
  nlopt::algorithm getSolver(std::string &solver);
  void assignEigenToVector1(std::vector<double> &result, int var_gindex, const Eigen::Vector3d &tmp);
  void setND(std::vector<double> nd_result); 
  std::string getResultCode(int result);
  void setdistposition(std::vector<std::vector<Eigen::Vector3d>> distposition); 
  typedef std::unique_ptr<BsplineOptimizer>  Ptr;
  // std::unique_ptr<BsplineOptimizer> bspline_optimizers_;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  EDTEnvironment::Ptr edt_environment_;

  // transfor Eigen
  Eigen::MatrixXd pos_tran;
  Eigen::MatrixXd vel_tran;

  Eigen::MatrixXd bspline_Matrix;

  // optimize parm
  double epsilon_tol_constraints_;

  ConvexHullsOfCurves_Std hulls_;
  std::vector<double> result_nd;

  int i_min_;
  int i_max_;
  int j_min_;
  int j_max_;
  int k_min_;
  int k_max_;

  int p_, n_, m_;     // p degree, n+1 control points, m = n+p+1
  Eigen::VectorXd knots_; // knots vector

  int num_of_variables_;
  int num_of_normals_;
  int num_of_constraints_;
  int num_of_obst_;
  int num_of_segments_;

  double lambda1_,lambda2_,lambda3_;
  double distance_threshold_;
  
  double opt_time_start;
  bool have_success = false;
  bool have_cost = false;
  int iteration_num;
  
  Eigen::Vector3d a_max_,v_max_;
  double acc_,vel_;
  int index_const_obs_ = 0;
  int index_const_vel_ = 0;
  int index_const_accel_ = 0;

  Eigen::MatrixXd start_point;
  Eigen::MatrixXd end_point;
  bool got_a_feasible_solution_ = false;
  double time_first_feasible_solution_ = 0.0;
  double best_cost_so_far_ = std::numeric_limits<double>::max();
  std::vector<double> best_feasible_sol_so_far_;


  int    max_iteration_num_[4];   // stopping criteria that can be used
  double max_iteration_time_[4];  // stopping criteria that can be used
  nlopt::algorithm solver_;
  int control_variable_num_,all_variable_num_;
  std::vector<std::vector<Eigen::Vector3d>>  distposition_;
  

  // main input
  Eigen::MatrixXd control_points_;     // B-spline control points, N x dim
  double          bspline_interval_;   // B-spline knot span
  Eigen::Vector3d end_pt_;             // end of the trajectory
  int             dim_;                // dimension of the B-spline
  Eigen::Matrix<double, 4, 3> LocalControlPoint;        //

                                       //
  int    max_num_id_, max_time_id_;    // stopping criteria
  int    cost_function_;               // used to determine objective function
  double start_time_;                  // global time for moving obstacles  the time for dynamic obstacle

  /* optimization parameters */
  int    order_;                  // bspline degree

  /* intermediate variables */
  /* buffer for gradient of cost function, to avoid repeated allocation and
   * release of memory */
  int                 iter_num_;       // iteration of the solver
  double              min_cost_;       //

  /* for benckmark evaluation only */

    /* helper function */
  static void myIneqConstraints(unsigned m, double *constraints, unsigned nn, const double *x, double *grad, void *f_data);
  static double myObjFunc(const std::vector<double> &x, std::vector<double> &grad, void *my_func_data);

};
}  // namespace fast_planner
#endif