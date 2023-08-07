
#include "bspline_opt/bspline_optimizer.h"
#include <nlopt.hpp>       // 就是这个优化求解问题的库。
// using namespace std;

namespace fast_planner {

void BsplineOptimizer::setParam(ros::NodeHandle& nh) {
  nh.param("optimization/lambda1", lambda1_, -1.0);
  nh.param("optimization/lambda2", lambda2_, -1.0);
  nh.param("optimization/lambda3", lambda3_, -1.0);
  nh.param("optimization/max_vel", vel_, -1.0);
  nh.param("optimization/max_acc", acc_, -1.0);
  nh.param("optimization/distance_threshold", distance_threshold_, -1.0);


  nh.param("optimization/max_iteration_num1", max_iteration_num_[0], -1);
  nh.param("optimization/max_iteration_num2", max_iteration_num_[1], -1);
  nh.param("optimization/max_iteration_num3", max_iteration_num_[2], -1);
  nh.param("optimization/max_iteration_num4", max_iteration_num_[3], -1);

  nh.param("optimization/max_iteration_time1", max_iteration_time_[0], -1.0);
  nh.param("optimization/max_iteration_time2", max_iteration_time_[1], -1.0);
  nh.param("optimization/max_iteration_time3", max_iteration_time_[2], -1.0);
  nh.param("optimization/max_iteration_time4", max_iteration_time_[3], -1.0);

  nh.param("optimization/order", this->p_, 3);
  std::cout<<"optimization/max_iteration_time1:"<<max_iteration_time_[0]<<std::endl;

  a_max_ << acc_,acc_,acc_;
  v_max_ << vel_,vel_,vel_;
  
  std::cout<<"k7"<<std::endl;

  this->distance_threshold_ = 0.5;
  std::cout<<"distance_threshold_:"<<distance_threshold_<<std::endl;

  this->pos_tran = Eigen::MatrixXd(4, 4);
  this->pos_tran <<0.1667,0.6667,0.1667,0,
                   0, 0.6667,0.333,0,
                   0,0.3333,0.6667,0,
                   0,0.1667,0.6667,0.1667;
  this->vel_tran = Eigen::MatrixXd(3, 3);       
  this->vel_tran <<0.5000,0.5000,0,
                   0,     1.000, 0,
                   0,     0.5000,0.5000;
  
  this->bspline_Matrix = Eigen::MatrixXd(4, 4);
  this->bspline_Matrix << 0.1667,0.6667,0.1667,0,
                          -0.5  ,0  ,   0.5000,0,
                          0.5000,-1.000,0.5000,0,
                          -0.1667,0.5000,-0.5000,0.1667;

  this->epsilon_tol_constraints_ = 0.1; // if right
}

void BsplineOptimizer::setEnvironment(const EDTEnvironment::Ptr& env) {
  this->edt_environment_ = env;
}

void BsplineOptimizer::setControlPoints(const Eigen::MatrixXd& points) {
  control_points_ = points;
  dim_            = control_points_.cols();
}

void BsplineOptimizer::setBsplineInterval(const double& ts,double time_start) { bspline_interval_ = ts; this->opt_time_start = time_start;}  // 这里设置控制点的值。

void BsplineOptimizer::setTerminateCond(const int& max_num_id, const int& max_time_id) {
  max_num_id_  = max_num_id;  // 最大的速度，和轨迹规划的时间。
  max_time_id_ = max_time_id;
}

void BsplineOptimizer::setKnots()
{
    n_ = control_points_.rows() - 1;
    m_ = n_ + p_ + 1;    // 给了我们控制点的个数和一系列的值

    knots_ = Eigen::VectorXd::Zero(m_ + 1);
    for (int i = 0; i <= m_; ++i)
    {
      if (i <= p_)
      {
        knots_(i) = double(-p_ + i) * bspline_interval_; 
      }
      else if (i > p_ && i <= m_ - p_)
      {
        knots_(i) = knots_(i - 1) + bspline_interval_;
      }
      else if (i > m_ - p_)
      {
        knots_(i) = knots_(i - 1) + bspline_interval_;
      }
    }
}

void BsplineOptimizer::setHulls(ConvexHullsOfCurves_Std &hulls)

{
  this->hulls_.clear();
  this->hulls_ = hulls;
  this->num_of_obst_ = hulls_.size();

  this->i_min_ = 0;
  this->i_max_ = 3 * (n_ + 1) - 9 - 9 -1;  // because pos, vel and accel at t_init_ and t_final_ are fixed (not dec variables)
  this->j_min_ = this->i_max_ + 1;
  this->j_max_ = this->j_min_ + 3 * (m_ - 2 * p_) * num_of_obst_ - 1;
  this->k_min_ = this->j_max_ + 1;
  this->k_max_ = this->k_min_ + (m_ - 2 * p_) * num_of_obst_ - 1;

  num_of_variables_ = k_max_ + 1;  // k_max_ + 1;

  int num_of_cpoints = n_ + 1;

  control_variable_num_ = dim_ * (num_of_cpoints - 2*p_);   //只有开始的三个点是确定的.

  num_of_segments_ = n_+1 - p_;  // this is the same as num_pol_

  num_of_normals_ = num_of_segments_ * num_of_obst_;
}

void BsplineOptimizer::setND(std::vector<double> nd_result) 
{
  this->result_nd = nd_result;

}

void BsplineOptimizer::setdistposition(std::vector<std::vector<Eigen::Vector3d>> distposition) 
{
  this->distposition_ = distposition;

}


Eigen::MatrixXd BsplineOptimizer::BsplineOptimizeTraj(const Eigen::MatrixXd& points, const double& ts, // N*3 row
                                                      ConvexHullsOfCurves_Std &hulls_curve, int max_num_id,
                                                      int max_time_id,double time_start,std::vector<double> nd_result,
                                                      std::vector<std::vector<Eigen::Vector3d>> positionss) {
  setControlPoints(points);
  setBsplineInterval(ts,time_start);
  setTerminateCond(max_num_id, max_time_id);
  setKnots();
  setHulls(hulls_curve);      
  setND(nd_result);
  setdistposition(positionss);
  optimize();
  return this->control_points_;
}

void BsplineOptimizer::initializeNumOfConstraints()
{
  // hack to get the number of constraints, calling once computeConstraints(...)
  std::vector<double> xx;
  for (int i = 0; i < this->num_of_variables_; i++)
  {
    xx.push_back(0.0);
  }
  std::vector<Eigen::Vector3d> q;
  std::vector<Eigen::Vector3d> n;
  std::vector<double> d;

  double constraints[10000];  // this number should be very big!! (hack, TODO)
  x2qnd(xx, q, n, d);
  // std::cout<<"q.size():"<<q.size()<<std::endl;
  for(int i=0;i<q.size();i++)
  {
    // std::cout<<"q:"<<q[i].transpose()<<std::endl;
    q[i].setZero();
  }
  computeConstraints(0, constraints, num_of_variables_, nullptr, q, n, d);
}

void BsplineOptimizer::optimize() {
  /* initialize solver */
  iter_num_        = 0;     // 
  min_cost_        = std::numeric_limits<double>::max();
  iteration_num    =0;
  const int pt_num = control_points_.rows(); 
  const double   bound = 3.0; 

  this->start_point = control_points_.block(0,0,3,3);
  this->end_point = control_points_.block(pt_num-3,0,3,3);

  nlopt::opt opt(nlopt::AUGLAG, this->num_of_variables_);  // need to create it here because I need the # of variables
  nlopt::opt local_opt(nlopt::LD_LBFGS,this->num_of_variables_);  // need to create it here because I need the # of variables  LD_TNEWTON LD_LBFGS

  local_opt.set_xtol_rel(1e-6);  // stopping criteria.
  local_opt.set_ftol_rel(1e-6);  // stopping criteria.
  local_opt.set_maxtime(max_iteration_time_[max_time_id_]);
  local_opt.set_maxeval(max_iteration_num_[max_num_id_]);

  opt.set_maxeval(max_iteration_num_[0]);
  opt.set_maxtime(max_iteration_time_[0]);
  opt.set_local_optimizer(local_opt);
  opt.set_ftol_rel(1e-6);  // stopping criteria.
  opt.set_xtol_rel(1e-6);  // Stopping criteria.
  
  // nlopt::opt opt(nlopt::LD_CCSAQ, this->num_of_variables_);   // nlopt::LD_MMA,nlopt::LD_CCSAQ,nlopt::LD_SLSQP,nlopt::LN_COBYLA
  // opt.set_maxeval(max_iteration_num_[max_num_id_]);
  // opt.set_maxtime(max_iteration_time_[max_time_id_]);
  // opt.set_xtol_rel(1e-8);  // Stopping criteria.
  // opt.set_ftol_rel(1e-10);  // stopping criteria.
  /*设置限制的值*/
  std::vector<double> lb(num_of_variables_);
  std::vector<double> ub(num_of_variables_);
  std::vector<double> x_result(num_of_variables_);   // 优化点的数量
  std::vector<double>  control_point;
  for(int i =p_;i < pt_num - p_;i++)
  {
    control_point.push_back(control_points_(i,0));
    control_point.push_back(control_points_(i,1));
    control_point.push_back(control_points_(i,2));
  }

  for (int i = i_min_; i <=i_max_; ++i) {         
    x_result[i] = control_point[i];
  }

  for (int i = 0; i < control_variable_num_; ++i) {
      lb[i] =  x_result[i] - bound; // 规划的范围
      ub[i] =  x_result[i] + bound;
  }
  // normals n
  int tempnum = 0;
  for (int j = j_min_; j <=j_max_; j++)
  {
    x_result[j] = this->result_nd[tempnum];
    tempnum = tempnum + 1;
    lb[j] = -1e9;
    ub[j] = 1e9;
  }
  // coefficients d
  for (int k = k_min_; k <=k_max_; k++)
  {
    x_result[k] = this->result_nd[tempnum];
    tempnum = tempnum + 1;
    lb[k] = -1e9;
    ub[k] = 1e9;
  }
  opt.set_lower_bounds(lb);
  opt.set_upper_bounds(ub);
  // std::cout<<"999vv"<<std::endl;
  
  /*设置重要的最小的数值*/
  opt.set_min_objective(BsplineOptimizer::myObjFunc,this);
  // std::cout<<"99988"<<std::endl;

  /*设置约束*/
  initializeNumOfConstraints(); // 主要目的是计算约束的个数。
  // std::cout<<"999999"<<std::endl;
  std::vector<double> tol_constraints;
  for (int i = 0; i < num_of_constraints_; i++)
  {
    tol_constraints.push_back(epsilon_tol_constraints_);
  }
  /*限制约束*/
  opt.add_inequality_mconstraint(BsplineOptimizer::myIneqConstraints, this, tol_constraints);

  try {
    double        final_cost;
    nlopt::result result = opt.optimize(x_result, final_cost);
    ROS_INFO_STREAM("[NL] Optimal, code=" << getResultCode(result));
  } 
  catch (std::exception& e) {
    ROS_WARN("[Optimization]: nlopt exception");
    cout << e.what() << endl;
  }

  // 成功优化
  std::vector<Eigen::Vector3d> q_result;
  std::vector<Eigen::Vector3d> n_result;
  std::vector<double> d_result;
  if(got_a_feasible_solution_ == true)
  { std::cout<<"num2"<<std::endl;
    x2qnd(best_feasible_sol_so_far_, q_result, n_result, d_result);
    std::cout<<"num3"<<std::endl;
  }
  else{
    std::cout<<"num1"<<std::endl;
    x2qnd(x_result, q_result, n_result, d_result);
    std::cout<<"num1"<<std::endl;
  }

  for (int i=0;i<q_result.size();i++)
  {
    this->control_points_.row(i) = q_result[i];
  }
  

  double all_dist=0;
  for (int i = 0; i < num_of_segments_; i++)
  {
    Eigen::Matrix<double, 4, 3> Q;
    Q.row(0) = this->control_points_.row(i);
    Q.row(1) = this->control_points_.row(i+1);
    Q.row(2) = this->control_points_.row(i+2);
    Q.row(3) = this->control_points_.row(i+3);
    for(int j=0;j<this->distposition_.size();j++)
    {
       
      for(int k=0;k<4;k++) 
      {
        double dist = (distposition_[j][i].transpose()-Q.row(k)).norm();
        all_dist+=dist;
      }   
    }
  }

  // std::ofstream outputFile("/home/jzp/data1.txt",std::ios::app);

  // if (outputFile.is_open()) {
  //       outputFile << all_dist << std::endl;

  //       outputFile.close();
  // }
  // std::cout<<"all_dist:"<<all_dist<<std::endl;

  // std::cout<<"iteration_num:"<<iteration_num<<std::endl;
}

double BsplineOptimizer::myObjFunc(const std::vector<double> &x, std::vector<double> &grad, void* my_func_data)
{
  BsplineOptimizer* opt = reinterpret_cast<BsplineOptimizer*>(my_func_data);

  std::vector<Eigen::Vector3d> q;
  std::vector<Eigen::Vector3d> n;
  std::vector<double> d;
  opt->x2qnd(x, q, n, d);
  
  double cost = opt->computeObjFunctionJerk(grad, q, n, d);
  if(opt->have_cost == true)
  {
    if (cost < opt->min_cost_)
    {
      opt->min_cost_ = cost;
      // Copy onto the std::vector)
      opt->best_feasible_sol_so_far_.clear();
      opt->best_feasible_sol_so_far_.resize(x.size());
      for (int i = 0; i < x.size(); i++)
      {
        opt->best_feasible_sol_so_far_[i] = x[i];
        // std::cout<<"x[i]:"<<x[i]<<std::endl;
      }
      opt->got_a_feasible_solution_ = true;
    }
    opt->have_cost == false;
  }
  opt->iteration_num ++;
  return cost;
}

// m是约束的个数，nn是要求解的变量个数。
void BsplineOptimizer::myIneqConstraints(unsigned m, double *constraints, unsigned nn, const double *x, double *grad, void *f_data)
{
   //nn is the length of x, m is the length of result 约束变量的个数, x 是输入的结果 m 是约束向量的个数吧.
  BsplineOptimizer *opt = reinterpret_cast<BsplineOptimizer *>(f_data);
  std::vector<Eigen::Vector3d> q;
  std::vector<Eigen::Vector3d> n;
  std::vector<double> d;
  opt->x2qnd(x, q, n, d);      // 每一次都会调用这个函数
  opt->computeConstraints(m, constraints, nn, grad, q, n, d); // 为什么要先初始化约束

  if (opt->areTheseConstraintsFeasible(constraints))
  {
    opt->have_cost = true;
  }
  return;
}

template <class T>
void BsplineOptimizer::x2qnd(T &x, std::vector<Eigen::Vector3d> &q, std::vector<Eigen::Vector3d> &n, std::vector<double> &d)
{
  q.clear();
  n.clear();
  d.clear();

  for (int i=0;i<3;i++)
  {
    Eigen::Vector3d a =start_point.row(i);
    q.push_back(a);
  }

  // Control points (3x1)
  for (int i = i_min_; i <= i_max_ - 2; i = i + 3)
  {
    q.push_back(Eigen::Vector3d(x[i], x[i + 1], x[i + 2]));
  }

  for (int i=0;i<3;i++)
  {
    Eigen::Vector3d a =end_point.row(i);
    q.push_back(a);
  }

  // Normals vectors (3x1)
  for (int j = j_min_; j <= j_max_ - 2; j = j + 3)
  {
    n.push_back(Eigen::Vector3d(x[j], x[j + 1], x[j + 2]));
  }

  // d values (1x1)
  for (int k = k_min_; k <= k_max_; k = k + 1)
  {
    d.push_back(x[k]);
  }
}

// r is the constraint index
// nn is the number of variables
// var_gindex is the index of the variable of the first element of the vector
void BsplineOptimizer::toGradDiffConstraintsDiffVariables(int var_gindex, const Eigen::Vector3d &tmp, double *grad, int r,
                                                     int nn)

{
  grad[r * nn + var_gindex] = tmp(0);
  grad[(r + 1) * nn + var_gindex + 1] = tmp(1);
  grad[(r + 2) * nn + var_gindex + 2] = tmp(2);
}

void BsplineOptimizer::toGradSameConstraintDiffVariables(int var_gindex, const Eigen::Vector3d &tmp, double *grad, int r,int nn)
{
  grad[r * nn + var_gindex] = tmp(0);
  grad[r * nn + var_gindex + 1] = tmp(1);
  grad[r * nn + var_gindex + 2] = tmp(2);
}

void BsplineOptimizer::assignValueToGradConstraints(int var_gindex, const double &tmp, double *grad, int r, int nn)
{
  grad[r * nn + var_gindex] = tmp;
}

// global index of the first element of the control point i
int BsplineOptimizer::gIndexQ(int i)
{
  return 3 * i - 9;  
}

// global index of the first element of the normal i
int BsplineOptimizer::gIndexN(int i)
{
  return 3 * i + this->j_min_;
}

int BsplineOptimizer::gIndexD(int i)
{
  return i + this->k_min_;
}

bool BsplineOptimizer::isADecisionCP(int i)

{  // If Q[i] is a decision variable
  return ((i >= 3) && i <= (n_ - 3));    //最后三个控制点都是确定的
}

void BsplineOptimizer::assignEigenToVector(double *result, int var_gindex, const Eigen::Vector3d &tmp)

{
  result[var_gindex] = tmp(0);
  result[var_gindex + 1] = tmp(1);
  result[var_gindex + 2] = tmp(2);
}

// m is the number of constraints, nn is the number of variables
void BsplineOptimizer::computeConstraints(unsigned m, double *constraints, unsigned nn, double *grad,
                                     const std::vector<Eigen::Vector3d> &q, const std::vector<Eigen::Vector3d> &n,
                                     const std::vector<double> &d)
{  
  Eigen::Vector3d ones = Eigen::Vector3d::Ones();
  int r = 0;
  // grad is a vector with nn*m elements
  // Initialize grad to 0 all the elements, is needed I think， m 是不等式约束的数量,nn是决策变量的数量。
  if (grad)  // 开始初始化了,先设置初始值.
  {
    for (int i = 0; i < nn * m; i++)
    {
      grad[i] = 0.0;         // 每一个变量的约束
    }
  }
  // std::cout<<"q.size()3:"<<q.size()<<std::endl;

  index_const_obs_ = r;
  // See here why we can use an epsilon of 1.0:
  // http://www.joyofdata.de/blog/testing-linear-separability-linear-programming-r-glpk/
  double epsilon = 1;

  /////////////////////////////////////////////////
  //////////// PLANES CONSTRAINTS    //////////////
  /////////////////////////////////////////////////
  // std::cout<<"1"<<n_ - 2<<std::endl;
  for (int i = 0; i <= (n_ - 3); i++)  // i  is the interval (\equiv segment) // 是我们的第几段
  {
    for (int obst_index = 0; obst_index < num_of_obst_; obst_index++)
    {
      int ip = obst_index * num_of_segments_ + i;  // index plane
      // std::cout<<"q.size()4:"<<q.size()<<std::endl;
      // impose that all the vertexes of the obstacle are on one side of the plane
      for (int j = 0; j < hulls_[obst_index][i].cols(); j++)  // Eigen::Vector3d vertex : hulls_[obst_index][i]
      {
        Eigen::Vector3d vertex = hulls_[obst_index][i].col(j);
        constraints[r] = -(n[ip].dot(vertex) + d[ip] - epsilon);  //+d[ip] // f<=0

        if (grad)
        {
          toGradSameConstraintDiffVariables(gIndexN(ip), -vertex, grad, r, nn);// important things
          assignValueToGradConstraints(gIndexD(ip), -1, grad, r, nn);
        }
        r++;
      }
      // std::cout<<"q.size()5:"<<q.size()<<std::endl;

      Eigen::Matrix<double, 4, 3> Qbs;  // b-spline
      Eigen::Matrix<double, 4, 3> Qmv;  // other basis "basis_" (minvo, bezier,...).
      Qbs.row(0) = q[i];
      Qbs.row(1) = q[i + 1];
      Qbs.row(2) = q[i + 2];
      Qbs.row(3) = q[i + 3];
      transformPosBSpline2bezier(Qbs, Qmv);  // Now Qmv is a matrix whose each row contains a "basis_" control point

      Eigen::Vector3d q_ipu;
      for (int u = 0; u <= 3; u++)
      {
        q_ipu = Qmv.row(u);                                     // if using the MINVO basis
        constraints[r] = (n[ip].dot(q_ipu) + d[ip] + epsilon);  //  // fi<=0

        if (grad)
        {
          toGradSameConstraintDiffVariables(gIndexN(ip), q_ipu, grad, r, nn); // 约束优化函数。
          assignValueToGradConstraints(gIndexD(ip), 1, grad, r, nn);

          for (int k = 0; k <= 3; k++)
          {  
            if (isADecisionCP(i+k))  // If Q[i] is a decision variable
            {
              toGradSameConstraintDiffVariables(gIndexQ(i + k), pos_tran(u, k) * n[ip], grad, r, nn);
            }                                                   // 转化为b样条的矩阵
          }       
          
        }
        r++;
      }
    }
  }

  index_const_vel_ = r;
  // std::cout<<"index_const_vel_ = r;"<<r<<std::endl;

  /////////////////////////////////////////////////
  ////////// VELOCITY CONSTRAINTS    //////////////
  /////////////////////////////////////////////////
  double t_vel = this->p_/(3*this->bspline_interval_);
  for (int i = 0; i <= n_ - 3; i++)  // If using BSpline basis, v0 and v1 are already determined by initial_state
  { 
    double c1 = p_ / (knots_(i + p_ + 1) - knots_(i + 1));
    Eigen::Vector3d v_iM2 = t_vel * (q[i + 1] - q[i]);  // q 的值，很重要的呢.
    Eigen::Vector3d v_iM1 = t_vel * (q[i+2] - q[i + 1]);
    Eigen::Vector3d v_i = t_vel * (q[i + 3] - q[i+2]);
    // std::cout<<"vi:"<<v_i<<std::endl;

    Eigen::Matrix<double, 3, 3> Qbs;  // b-spline
    Eigen::Matrix<double, 3, 3> Qmv;  // other basis "basis_" (minvo, bezier,...).

    Qbs.col(0) = v_iM2;
    Qbs.col(1) = v_iM1;
    Qbs.col(2) = v_i;

    transformVelBSpline2bezier(Qbs, Qmv); //速度也是可以转化的。速度转化矩阵

    ///////////////////////// ANY BASIS //////////////////////////////
    for (int j = 0; j < 3; j++)
    {  
      //|v_{i-2+j}| <= v_max ////// v_{i-2}, v_{i-1}, v_{i}

      Eigen::Vector3d v_iM2Pj = Qmv.col(j);  // v_{i-2+j};

      // Constraint v_{i-2+j} - vmax <= 0牛的
      assignEigenToVector(constraints, r, v_iM2Pj - v_max_);  // f<=0   xyz 三个方向的数值

      if (grad)
      {
        // and now assign it to the vector grad
        for (int u = 0; u < 3; u++)
        {
          if (isADecisionCP(i + u))  // If Q[i] is a decision variable
          {
            toGradDiffConstraintsDiffVariables(gIndexQ(i + u), -c1*ones, grad, r, nn);//partials.col(u)
          }

          // v_{i-2+u} depends on q_{i-2+u+1}
          if (isADecisionCP(i + u + 1))  // If Q[i+1] is a decision variable
          {
            toGradDiffConstraintsDiffVariables(gIndexQ(i + u + 1),c1*ones, grad, r, nn);//partials.col(u + 1)
          }
        }
      }
      r = r + 3;


      // Constraint -v_{i-2+j} - vmax <= 0  // 连续段的值
      assignEigenToVector(constraints, r, -v_iM2Pj - v_max_);  // f<=0

      if (grad)
      {
        // and now assign it to the vector grad
        for (int u = 0; u < 3; u++)
        {
          if (isADecisionCP(i  + u))  // If Q[i] is a decision variable
          {
            toGradDiffConstraintsDiffVariables(gIndexQ(i  + u), c1*ones, grad, r, nn);
          }

          // v_{i-2+u} depends on q_{i-2+u+1}
          if (isADecisionCP(i + u + 1))  // If Q[i+1] is a decision variable
          {
            toGradDiffConstraintsDiffVariables(gIndexQ(i  + u + 1), -c1*ones, grad, r, nn);
          }
        }
      }
      r = r + 3;
    }
  }
  

  index_const_accel_ = r;
  // std::cout<<"index_const_accel_ = r;"<<r<<std::endl;
  /////////////////////////////////////////////////
  ////////// ACCELERATION CONSTRAINTS    //////////
  /////////////////////////////////////////////////
  double t_acc = (this->p_-1)/(2*this->bspline_interval_);
  for (int i = 1; i <= (n_ - 2); i++)  // a0 is already determined by the initial state //真的很重要的

  {
    double c1 = p_ / (knots_(i + p_ + 1) - knots_(i + 1));
    double c2 = p_ / (knots_(i + p_ + 1 + 1) - knots_(i + 1 + 1));
    double c3 = (p_ - 1) / (knots_(i + p_ + 1) - knots_(i + 2));

    Eigen::Vector3d v_i = c1 * (q[i + 1] - q[i]);
    Eigen::Vector3d v_iP1 = c2 * (q[i + 2] - q[i + 1]);
    Eigen::Vector3d a_i = c3 * (v_iP1 - v_i);
    // std::cout<<"r:"<<r<<"aI:"<<a_i<<std::endl;

    assignEigenToVector(constraints, r, a_i - a_max_);  // f<=0
    if (grad)
    {
      if (isADecisionCP(i))  // If Q[i] is a decision variable
      {
        toGradDiffConstraintsDiffVariables(gIndexQ(i), c3 * c1 * ones, grad, r, nn);
      }
      else
      {
        if (isADecisionCP(i + 1))  // If Q[i+1] is a decision variable
        {
          toGradDiffConstraintsDiffVariables(gIndexQ(i + 1), (-c3 * c2 - c3 * c1) * ones, grad, r, nn);
        }
        if (isADecisionCP(i + 2))  // If Q[i+2] is a decision variable
        {
          toGradDiffConstraintsDiffVariables(gIndexQ(i + 2), c3 * c2 * ones, grad, r, nn);
        }
      }
    }
    r = r + 3;     // 最大速度约束

    assignEigenToVector(constraints, r, -a_i - a_max_);  // f<=0
    if (grad)
    {
      if (isADecisionCP(i))  // If Q[i] is a decision variable
      {
        toGradDiffConstraintsDiffVariables(gIndexQ(i), -c3 * c1 * ones, grad, r, nn);
      }

      else
      {
        if (isADecisionCP(i + 1))  // If Q[i+1] is a decision variable
        {
          toGradDiffConstraintsDiffVariables(gIndexQ(i + 1), -(-c3 * c2 - c3 * c1) * ones, grad, r, nn);
        }
        if (isADecisionCP(i + 2))  // If Q[i+2] is a decision variable
        {
          toGradDiffConstraintsDiffVariables(gIndexQ(i + 2), -c3 * c2 * ones, grad, r, nn);
        }
      }
    }
    r = r + 3;
  }

  // std::cout<<"endlconstra"<<std::endl;

  // index_const_normals_ = r; // 这个在一定范围，我们不考虑
  // 控制点的范围都在我们的里面也很重要的
  num_of_constraints_ = r;  // + 1 has already been added in the last loop of the previous for loop;
}

template <class T>
bool BsplineOptimizer::areTheseConstraintsFeasible(const T &constraints)
{
  for (int i = 0; i < num_of_constraints_; i++)
  {
    if (constraints[i] > epsilon_tol_constraints_)  // constraint is not satisfied yet  // 判断是不是有效的值。
    {
      return false;
    }
  }
  return true;
}
 

// nn is the number of variables
double BsplineOptimizer::computeObjFunctionJerk(std::vector<double> &grad, std::vector<Eigen::Vector3d> &q,
                                                std::vector<Eigen::Vector3d> &n, std::vector<double> &d)
{
  // Cost. See Lyx derivation (notes.lyx)
  double cost = 0.0;
  Eigen::Matrix<double, 1, 4> tmp;
  tmp << 0.0, 0.0, 0.0, 6.0;
  // std::cout<<"aaaaa1"<<std::endl;
  std::vector<Eigen::Vector3d> gradient;
  for (int i = 0; i < n_+1; i++)   // 几个控制点
  {
    gradient.push_back(Eigen::Vector3d::Zero());
  }

  for (int i = 0; i < num_of_segments_; i++)
  {
    Eigen::Matrix<double, 1, 4> A_i_times_tmp = tmp*this->bspline_Matrix;

    Eigen::Matrix<double, 4, 3> Q;
    Q.row(0) = q[i];
    Q.row(1) = q[i + 1];
    Q.row(2) = q[i + 2];
    Q.row(3) = q[i + 3];

    cost += (A_i_times_tmp*Q).squaredNorm();  // 这个矩阵我也可以得到

    // I only need to do this if grad!=NULL
    for (int u = 0; u <= 3; u++)
    {
      if(isADecisionCP(i+u))
      {
        gradient[i + u] += lambda1_*2 * (A_i_times_tmp*Q) * A_i_times_tmp(u);    // 计算这个很重要的梯度值。看了一下，好像优化的方法不是很重要的，主要就是寻路的方法
      }
    }
  
  } 
  // std::cout<<"jerk_cost:"<<lambda1_*cost<<std::endl;
  
  double total_error = 0;
  double error =0;

  for (int i = 0; i < num_of_segments_; i++)
  { 
      double          dist;
      Eigen::Vector3d dist_grad;   // g_zero(0, 0, 0);
      edt_environment_->evaluateEDTWithGrad(q[i], -1.0, dist, dist_grad);

      if (dist < distance_threshold_) {
          total_error += pow(dist - distance_threshold_, 2);
          if(isADecisionCP(i))
          {
            gradient[i] += lambda2_*2.0 * (dist - distance_threshold_) * dist_grad;
          }
          
      }

  }


  // for (int i = 0; i < num_of_segments_; i++)
  // {
  //   Eigen::Matrix<double, 4, 3> Q;
  //   Q.row(0) = q[i];
  //   Q.row(1) = q[i + 1];
  //   Q.row(2) = q[i + 2];
  //   Q.row(3) = q[i + 3];
  //   // std::cout<<"Q:"<<Q<<std::endl;
  //   setLocalControlPoint(Q);
  //   for (double j_time = 0; j_time<=1;j_time = j_time + 0.2)
  //   {
  //       double t_use = j_time;
  //       Eigen::Vector3d point, grad_p(0,0,0);
  //       point = current_spline_evaluate(t_use);  // 就是分成几段求解。
  //       // std::cout<<"point1:"<<point<<std::endl;
        
  //       double dist =0;
  //       edt_environment_->evaluateEDTWithGrad(point, -1.0, dist, grad_p);
  //       // if (grad_p.norm() > 1e-4) grad_p.normalize();  
  //       // std::cout<<"point:"<<dist<<std::endl;  
        
  //       if(dist > distance_threshold_) continue;

  //       double  diff = dist - distance_threshold_;
  //       error = 0.5 * diff * diff / distance_threshold_;
  //       total_error += error;

  //       // I only need to do this if grad!=NULL
  //       Eigen::VectorXd aa(4);
  //       aa << 1, t_use, pow(t_use,2), pow(t_use,3);

  //       Eigen::Matrix<double, 1, 4> A_Matrix = aa.transpose()*this->bspline_Matrix;

  //       for (int u = 0; u <= 3; u++)
  //       { 
  //         if(isADecisionCP(i+u))
  //         {
  //           double segment_grad = A_Matrix(u);
  //           gradient[i+u] += lambda2_*(diff/distance_threshold_)*grad_p*segment_grad;
  //         }
  //       }
  //   }
  // }   

  // double total_position_error = 0;
  double position_error =0;

  for (int i = 0; i < num_of_segments_; i++)
  {
    Eigen::Matrix<double, 4, 3> Q;
    Q.row(0) = q[i];
    Q.row(1) = q[i + 1];
    Q.row(2) = q[i + 2];
    Q.row(3) = q[i + 3];
    // std::cout<<"this->distposition_.siz"<<this->distposition_.size()<<std::endl;
    for(int j=0;j<this->distposition_.size();j++)
    {
       
      for(int k=0;k<4;k++) 
      {
        double dist = (distposition_[j][i].transpose()-Q.row(k)).norm();
        // std::cout<<"dist"<<distposition_[j][i]<<std::endl;
        if(dist>1.8)
        {
          continue;
        }
        else
        {
           position_error += dist;
           if(isADecisionCP(i+k))
           {
             gradient[i+k] += lambda3_*2.0*(Q.row(k).transpose()-distposition_[j][i]);
           }
        }
      }   
    }
  }



  // std::cout<<"jerk1_cost:"<<lambda2_*total_error<<std::endl;
  // std::cout<<"jerk2_cost:"<<lambda3_*position_error<<std::endl;

  cost = lambda1_*cost + lambda2_*total_error+lambda3_*position_error;
  // 把距离
  if (!grad.empty())
  {
    // Initialize to zero all the elements, IT IS NEEDED (if not it doesn't converge)
    for (int i = 0; i < num_of_variables_; i++)
    {
      grad[i] = 0.0;
    }

    int temp_index = 0;
    for (int i = 3; i < n_+1-3 ; i++)  // 已经被确定了
    {
      grad[temp_index]     =      gradient[i](0);
      grad[temp_index+1]   =      gradient[i](1);
      grad[temp_index+2]   =      gradient[i](2);
      temp_index = temp_index + 3;
    }
  }
  // std::cout<<"jerk2_cost:"<<cost<<std::endl;
  return cost;   // 求解相应点的梯度值。
}

void BsplineOptimizer::transformVelBSpline2bezier(const Eigen::Matrix<double, 3, 3> &Qbs,
                                                 Eigen::Matrix<double, 3, 3> &Qmv)
{
  Qmv = this->vel_tran*Qbs ;
}

void BsplineOptimizer::transformPosBSpline2bezier(const Eigen::Matrix<double, 4,3> &Qbs,
                                                 Eigen::Matrix<double, 4, 3> &Qmv)
{
  Qmv = this->pos_tran*Qbs;
}

void BsplineOptimizer::setLocalControlPoint(Eigen::Matrix<double, 4, 3> Q)
{
  this->LocalControlPoint = Q;
}

Eigen::Vector3d BsplineOptimizer::current_spline_evaluate(double t_use)
{
  // double t_use = (time - t_start)/(t_end - t_start);
  Eigen::VectorXd t_pow(4);
  t_pow << 1, t_use, pow(t_use,2), pow(t_use,3);
  Eigen::Vector3d point = t_pow.transpose()*this->bspline_Matrix*this->LocalControlPoint;
  return point;  
}

nlopt::algorithm BsplineOptimizer::getSolver(std::string &solver)
{
  // nlopt::algorithm_from_string("LD_MMA"); //doesn't work in c++

  if (solver == "LD_MMA")
  {
    return nlopt::LD_MMA;
  }
  else if (solver == "LN_NELDERMEAD")
  {
    return nlopt::LN_NELDERMEAD;
  }
  else if (solver == "LN_SBPLX")
  {
    return nlopt::LN_SBPLX;
  }
  else if (solver == "LN_PRAXIS")
  {
    return nlopt::LN_PRAXIS;
  }
  else if (solver == "LD_AUGLAG")
  {
    return nlopt::LD_AUGLAG;
  }
  else if (solver == "LD_AUGLAG_EQ")
  {
    return nlopt::LD_AUGLAG_EQ;
  }
  else if (solver == "LN_BOBYQA")
  {
    return nlopt::LN_BOBYQA;
  }
  else if (solver == "LD_SLSQP")
  {
    return nlopt::LD_SLSQP;
  }
  else if (solver == "LN_NEWUOA")
  {
    return nlopt::LN_NEWUOA;
  }
  else if (solver == "LN_NEWUOA_BOUND")
  {
    return nlopt::LN_NEWUOA_BOUND;
  }
  else if (solver == "LD_TNEWTON_PRECOND_RESTART")
  {
    return nlopt::LD_TNEWTON_PRECOND_RESTART;
  }
  else if (solver == "LD_TNEWTON_RESTART")
  {
    return nlopt::LD_TNEWTON_RESTART;
  }
  else if (solver == "LD_TNEWTON_PRECOND")
  {
    return nlopt::LD_TNEWTON_PRECOND;
  }
  else if (solver == "LD_VAR1")
  {
    return nlopt::LD_VAR1;
  }
  else if (solver == "LD_VAR2")
  {
    return nlopt::LD_VAR2;
  }
  else if (solver == "LD_LBFGS_NOCEDAL")
  {
    return nlopt::LD_LBFGS_NOCEDAL;
  }
  else if (solver == "LD_LBFGS")
  {
    return nlopt::LD_LBFGS;
  }
  else if (solver == "LD_CCSAQ")
  {
    return nlopt::LD_CCSAQ;
  }
  else
  {
    std::cout << "Are you sure this solver exists?" << std::endl;
    abort();
  }
}

std::string BsplineOptimizer::getResultCode(int result)
{
  switch (result)
  {
    case -5:
      return "Forced_Stop";
    case -4:
      return "Roundoff_limited";
    case -3:
      return "Out_of_memory";
    case -2:
      return "Invalid_args";
    case -1:
      return "Failure";
    case 1:
      return "Success";
    case 2:
      return "Stopval_reached";
    case 3:
      return "Ftol_reached";
    case 4:
      return "Xtol_reached";
    case 5:
      return "Maxeval_reached";
    case 6:
      return "Maxtime_reached";
    default:
      return "Result_Code_unknown";
  }
}


} //namespace 


