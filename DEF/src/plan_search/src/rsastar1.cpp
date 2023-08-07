
#include <plan_search/rsastar.h>
#include <sstream>

using namespace std;
using namespace Eigen;

namespace def_planner
{
RSAstar::~RSAstar()
{
  // 搜索的这么多的点.
}


void RSAstar::setParam(ros::NodeHandle& nh)
{
  std::cout<<"aa"<<std::endl;
  nh.param("search/max_vel", max_vel_, -1.0);
  nh.param("search/max_acc", max_acc_, -1.0);
  nh.param("search/horizon", horizon_, -1.0);
  nh.param("search/resolution_astar", resolution_, -1.0);
  nh.param("search/tie_breaker", tie_breaker, -1.0);
  nh.param("search/max_runtime", this->max_runtime_, -1.0);
  nh.param("manager/ts", this->ts, -1.0);
  std::cout<<"bb"<<std::endl;
  std::cout<<"search/tie_breaker:"<<tie_breaker<<std::endl;
  std::cout<<"this->ts:"<<this->ts<<std::endl;
}

void RSAstar::reset()
{
  // expanded_nodes_.clear();

  std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator> empty_queue;
  openList_.swap(empty_queue);
  
  path_nodes_.clear();
  result_.clear();

  end_node = new PathNode;
  start_node = new PathNode;
  
  std::cout<<"reset1"<<std::endl;
  
  std::cout<<"reset"<<std::endl;
  this->edt_environment_->sdf_map_->getMinMaxId(this->min_id, this->max_id); 
  this->min_id(2) = 0;
  this->max_id(2) = 5;

  initGridNodeMap();

  
}


void RSAstar::clearNodeMap() {
  for (int i = 0; i < x_size; i++) {
    for (int j = 0; j < y_size; j++) {
      for (int k = 0; k < z_size; k++) {
        delete BSplineGridNodeMap[i][j][k];
      }
      delete[] BSplineGridNodeMap[i][j];
    }
    delete[] BSplineGridNodeMap[i];
  }
  delete[] BSplineGridNodeMap;
}

void RSAstar::init()
{
  /* ---------- map params ---------- */
  std::cout<<"cc"<<std::endl;
  this->inv_resolution_ = 1.0 / resolution_;
  edt_environment_->sdf_map_->getRegion(origin_, map_size_3d_);

  cout << "origin_: " << origin_.transpose() << endl;
  cout << "map size: " << map_size_3d_.transpose() << endl;  // get map

  double current_pow_inv_dt = 1.0;
  for (int i = 0; i < 2 * (P_ + 1); ++i) 
  {
    pow_inv_dt_[i] = current_pow_inv_dt;   // 时间的间隔的值
    current_pow_inv_dt /= ts;
  }
  std::cout<<"dd"<<std::endl;
  quadratic_cost_weight.fill(0.0);        // 选择哪一个东西
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
  quadratic_cost_jacobian = computeQuadraticCostJacobian();

  separator_solver_ = new separator::Separator();
  std::cout<<"ee"<<std::endl;

  Eigen::MatrixXd trans_matrix(3,4);
  trans_matrix<<-1,1,0,0,
                0 ,-1,1,0,
                0,0,-1,1;
  Eigen::MatrixXd bspline2bezier_diff1_matrix1 = (computeBasisMatrix_Bezier(1).inverse()).block(0,0,3,3)*computeBasisMatrix_Bspline(1).block(0,0,3,3)*trans_matrix;
  bspline2bezier_diff1_matrix = bspline2bezier_diff1_matrix1/this->ts;
  Eigen::MatrixXd transacc_matrix(2,4);
  transacc_matrix<<-1,-2,1,0,
                0 ,-1,-2,1;
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
  // std::cout<<"bspline2bezier_matrix:"<<bspline2bezier_matrix<<std::endl;
  // std::cout<<"bspline2bezier_matrix:"<<bspline2bezier_matrix<<std::endl;

  // 采样好两千个点进行储存,不知道可行吗.
  // std::random_device rd;
  // std::mt19937 gen(rd());
  // std::normal_distribution<> dist(0, 1); // 均值为0，标准差为1的正态分布
  // const double mean = 1; // 均值
  // const double std_dev = 0.5; // 标准差
  // int num_samples = 0;
  // while (num_samples < 2000) {
  //   double value = mean + std_dev * dist(gen); // 缩放和平移
  //   if (value >= 0 && value <= max_vel_) {
  //       // 处理采样结果
  //       num_samples++;
  //       all_combinations_.push_back(value);
  //       all_combinations_.push_back(-value);
  //   }
  //   }
  // std::uniform_int_distribution<> dist1(0, 3999);
  // int num_samples1 = 0;
  // int x,y,z;
  //   while (num_samples1 < 100) {
  //       x = dist1(gen);
  //       y = dist1(gen);
  //       z = dist1(gen);
  //      Eigen::Vector3d v (all_combinations_[x],all_combinations_[y],all_combinations_[z]);
  //      vs.push_back(v);
  //   }

  // std::random_device rd;
  // std::mt19937 gen(rd());
  // std::normal_distribution<> dist(0, 1); // 均值为0，标准差为1的正态分布
  // const double mean = max_vel_/2 ; // 均值
  // const double std_dev = 1; // 标准差
  // const int num_samples = 2000;
  // const int num_samples1 = 500;
  // const int max_index = num_samples * 2;

  // // 采样所有满足条件的值
  // int num_found = 0;
  // while (num_found < num_samples) {
  //   double value = mean + std_dev * dist(gen); // 缩放和平移
  //   if (value >= 0 && value <= max_vel_) {
  //       all_combinations_.push_back(value);
  //       all_combinations_.push_back(-value);
  //       num_found++;
  //       // std::cout<<"value:"<<value<<std::endl;
  //   }
  // } 

  // // 抽样
  // std::uniform_int_distribution<> dist1(0, max_index-1);
  // std::set<int> sampled_indices;
  // while (sampled_indices.size() < num_samples1) {
  //   int index = dist1(gen);
  //   if (sampled_indices.find(index) == sampled_indices.end()) {
  //       sampled_indices.insert(index);
  //       int x = index / (num_samples*2);
  //       int y = (index / 2) % num_samples;
  //       int z = index % 2;
  //       Eigen::Vector3d v (all_combinations_[x],all_combinations_[y],all_combinations_[z]);
  //       vs.push_back(v);
  //   }
  // }


  // Generate 10x10x10 grid
  std::vector<Eigen::Vector3d> points;
  int num_seg =10;
  double interval = 2.0 * max_vel_ / num_seg;
  for (int i = 0; i < num_seg; i++) {
    double x = -max_vel_ + i * interval;
    for (int j = 0; j < num_seg; j++) {
        double y = -max_vel_ + j * interval;
        for (int k = 0; k < num_seg; k++) {
            double z = -max_vel_ + k * interval;
            Eigen::Vector3d point(x, y, z);
            double distance = ((this->ts * this->P_) * point / (1.0 * this->P_)).norm();
            // std::cout<<"distance:"<<distance<<std::endl;
            if (distance <= this->ts*max_vel_*sqrt(3)) {
                points.push_back(point);
            }
        }
    }
  }

  this->vs = points;
  std::cout<<"points"<<points.size()<<std::endl;

  std::cout<<"ff"<<std::endl;
}

double RSAstar::getDiagDis(PathNodePtr node1, PathNodePtr node2) {
  int dx = abs(node1->index(0) - node2->index(0));
  int dy = abs(node1->index(1) - node2->index(1));
  int dz = abs(node1->index(2) - node2->index(2));// x y z 

  int diag = min(min(dx, dy), dz);// 最小距离
  dx -= diag;
  dy -= diag;
  dz -= diag;  
  double d = 0;
  if (dx == 0) {  //判断哪一个是最小的
    d = sqrt(3.0) * diag + sqrt(2.0) * min(dy, dz) + abs(dy - dz); //牛的,对角线的值为重要的值，对角线的距离，A*算法对角线距离
  }
  if (dy == 0) {
    d = sqrt(3.0) * diag + sqrt(2.0) * min(dx, dz) + abs(dx - dz);
  }
  if (dz == 0) {
    d = sqrt(3.0) * diag + sqrt(2.0) * min(dx, dy) + abs(dx - dy);
  }
  // double d = (node1->position - node2->position).norm();
  return d;    // 到终点的距离
}
double RSAstar::getHeu(PathNodePtr node1, PathNodePtr node2) {   // h是启发函数
  return tie_breaker *getDiagDis(node1, node2);  // break,很重要的值.
}

double RSAstar::getGeu(PathNodePtr node1, PathNodePtr node2) {   // h是启发函数
  return getDiagDis(node1, node2);  // break,很重要的值.
}
// std::vector<PathNodePtr> RSAstar::Neighbor(PathNodePtr current) {

//   // std::cout<<"current->dist"<<current->dist<<std::endl;
//   neighborlist_.clear();
//   std::vector<PathNodePtr> nbrs;
//   // std::cout<<"vs:"<<vs.size()<<std::endl;
//   for (auto vi : vs)
//   {
//     PathNodePtr neighbor = new PathNode;
//     if (vi.norm() < 1e-5)   // 速度太慢了,我并不是很需要
//     {
//       continue;
//     }  
//     // std::cout<<"vs:"<<vs.size()<<std::endl;
//     // std::cout<<"jjjj"<<std::endl;
//     double distance = ((this->ts*this->P_) * vi / (1.0 * P_)).norm();
//     // std::cout<<"distance:"<<distance<<std::endl;
//     // std::cout<<"jjjj2"<<current->dist<<std::endl;
//     if(distance <= 0.5)   // 判断是不是小于那个重要的值
//     {
//       if(distance < current->dist)  // 只要小于这个最小距离就很撞上.
//       {
//         neighbor->position = (this->ts*P_) * vi / (1.0 * P_) + current->position; //时间和速度控制  这个的时间。
//         std::cout<<"neighbor->position:"<<neighbor->position<<std::endl;
//         neighbor->index = posToIndex(neighbor->position);
//       }         
//     }
//     else
//     {
//       continue;
//     }   // 采样了这么多点不知道会有什么问题
//     // std::cout<<"hhh"<<std::endl;
    
//     // neighbor->g_score = current->g_score + weightEdge(current, neighbor);
//     // neighbor->h_score = h(neighbor);  // 这个，这里有问题
//     auto ptr_to_voxel = neighborlist_.find(neighbor->index);
//     bool already_exist = (ptr_to_voxel != neighborlist_.end());
//     if(!already_exist)
//     {
//       nbrs.push_back(neighbor);
//       // std::cout<<"neighbor->position"<<neighbor->position<<std::endl;
//       neighborlist_[neighbor->index] = true;
//     }

//   }
//   // std::cout<<"nbrs:"<<nbrs.size()<<std::endl;
//   return nbrs;
// }

void RSAstar::Neighbor(PathNodePtr current,std::vector<PathNodePtr>& neighbors) {

  neighborlist_.clear();
  neighborlist_[current->index] = true;
  std::vector<PathNodePtr> nbrs;
  // std::cout<<"current->position:"<<current->position.transpose()<<std::endl;
  // std::cout<<"current->dist:"<<current->dist<<std::endl;
  
  for (auto vi : this->vs)
  {
    PathNodePtr neighbor = new PathNode;
    
    if (vi.norm() < 1e-4)
    {
      continue;
    }  
    
    double distance = ((this->ts*this->P_) * vi / (1.0 * P_)).norm();
    if(distance < current->dist)
    {
      Eigen::Vector3d pt = (this->ts*P_) * vi / (1.0 * P_) + current->position;
      Eigen::Vector3i index = posToIndex(pt);
      // std::cout<<"neighbor->position:"<<index.transpose()<<std::endl;
      if(index(0)>=0&&index(0)<this->x_size&&index(1)>=0&&index(1)<this->y_size&&index(2)>=0&&index(2)<this->z_size)
      {
        neighbor = BSplineGridNodeMap[index(0)][index(1)][index(2)];
        if(neighbor->node_state == NOT_EXPAND)
        {
          neighbor->position = pt;
        }
      }
      else
      {
        delete neighbor;
        continue;
      }
      // std::cout<<"neighbor->position:"<<neighbor->position.transpose()<<std::endl;

      auto ptr_to_voxel = neighborlist_.find(neighbor->index);
      bool already_exist = (ptr_to_voxel != neighborlist_.end());
      if(!already_exist)
      {
        nbrs.push_back(neighbor);
        neighborlist_[neighbor->index] = true;
      }
    }         
    else
    {
      delete neighbor;
      continue;
    }
  }
  // std::cout<<"nbrs:"<<nbrs.size()<<std::endl;

  neighbors = nbrs;
  // std::cout<<"nbrs2:"<<nbrs.size()<<std::endl;
}



bool RSAstar::CheckStaticObstacle(PathNodePtr neighbor,double num) {
    Matrix<double, P_ + 1, 3> Q;
    Q = local_control_points;
    Q.row(P_) = neighbor->position.transpose();
    double t_init = time_start + ts*(num-P_);
    double t_end  = t_init + ts;
    int n=5;
    // std::cout<<"num:"<<num<<std::endl;
    // std::cout<<"t_init:"<<t_init<<std::endl;
    // std::cout<<"t_end :"<<t_end <<std::endl;
    for(double time=t_init;time<=t_end;time=time+double(ts/n))
    {
      double t_use = (time - t_init)/ts;
      Eigen::VectorXd t_pow(4);
      t_pow<< 1,pow(t_use,1),pow(t_use,2),pow(t_use,3);
      Eigen::Vector3d point =t_pow.transpose()*bspline_basis_matrix*Q;
      // std::cout<<"time:"<<time <<std::endl;
      if(this->edt_environment_->sdf_map_->getInflateOccupancy(point) == 1)
      {
        return true;
      }
    }
    return false;
}

int RSAstar::getNumControlNow(PathNodePtr current)
{
  std::vector<PathNodePtr> path;
  path.push_back(current);
  while (current->parent != NULL) {
    current = current-> parent;
    path.push_back(current);
  }
  return path.size();
}

bool RSAstar::checkTrajDynamics(PathNodePtr neighbor) {

  if (P_ < 2) {
    cerr << "RSAstar::checkTrajDynamics : degree of bspline is too low" << endl;
    return false;
  }
  Matrix<double, P_ + 1, 3> bspline_diff0_control_points;
  Matrix<double, P_, 3> bezier_diff1_control_points;
  Matrix<double, P_ - 1, 3> bezier_diff2_control_points;

  bspline_diff0_control_points = local_control_points;
  
  bspline_diff0_control_points.row(P_) = neighbor->position.transpose();
  // std::cout<<"bspline_diff0_control_points:"<<bspline_diff0_control_points<<std::endl;
  bezier_diff1_control_points = (bspline2bezier_diff1_matrix * bspline_diff0_control_points);//.block(0, 0, P_, 3);
  // std::cout<<"bezier_diff1_control_points:"<<bezier_diff1_control_points<<std::endl;

  bezier_diff2_control_points = (bspline2bezier_diff2_matrix * bspline_diff0_control_points);//.block(0, 0, P_ - 1, 3);
  // std::cout<<"bezier_diff2_control_points:"<<bezier_diff2_control_points<<std::endl;


  // std::cout<<"coffe:"<<bezier_diff1_control_points.maxCoeff()<<" "<<bezier_diff1_control_points.minCoeff()<<" "<<bezier_diff2_control_points.maxCoeff()<<" "<<bezier_diff2_control_points.minCoeff()<<std::endl;

  if (bezier_diff1_control_points.maxCoeff() > max_vel_ ||   //返回矩阵中最大和最小值
      bezier_diff1_control_points.minCoeff() < -max_vel_ ||
      bezier_diff2_control_points.maxCoeff() > max_acc_ ||
      bezier_diff2_control_points.minCoeff() < -max_acc_
      ) {
    return false;//Not dynamically feasible
  }

  return true;//Dynamically feasible
}

void RSAstar::setLocalControlPoints(PathNodePtr current) {
  for (int i = 0; i < P_; i++) {
    this->local_control_points.row(P_ - 1 - i) = current->position.transpose();
    if (current->parent != NULL)
      current = current->parent;     // 这一段轨迹的作用是什么
    else {
      // if (i != P_ - 1)
        
        // cerr << "error: RSAstar::setLocalControlPoints : i=" << i << endl;
    }
  }
  return;
}



double RSAstar::getgscore(PathNodePtr tentative) {
  double cost(0.0);
  for (int deriv = 1; deriv < P_; deriv++) {
    cost = cost + quadratic_cost_weight[deriv - 1] * quadraticCost(deriv, tentative);    // 给了一个系数的值，很重要的了。
  }
  return cost;
}


double RSAstar::quadraticCost(int deriv, PathNodePtr tentative) {
  return quadraticCost(quadratic_cost_jacobian[deriv - 1] * pow_inv_dt_[2 * deriv - 1], tentative);
}

double RSAstar::quadraticCost(const RSAstar::MatrixN &quadratic_cost,
                                        PathNodePtr tentative) {
  int controlpts_num = 1;
  vector<Vector3d> controlpts_coord;
  PathNodePtr it = tentative;
  for (controlpts_coord.push_back(it->position); it->parent != NULL;) {  // 回溯前面所有的点
    controlpts_coord.push_back(it->position);
    controlpts_num++;
    it = it->parent;
    if (controlpts_num == P_ + 1) break;
  }
  if (controlpts_coord.size() < P_ + 1) return 0;

  double inc_cost[3];
  for (int i = 0; i < 3; i++) {
    VectorN coefficients;
    for (int j = 0; j < P_ + 1; j++)
      coefficients[j] = controlpts_coord[j](i);  // 当前的五个控制点
    inc_cost[i] = coefficients.transpose() * quadratic_cost * coefficients;   // x y z 三个方向的值
  }
  return inc_cost[0] + inc_cost[1] + inc_cost[2] + tentative->g_score;  // 之前的值，很重要的
}

bool RSAstar::collidesWithObstacles(PathNodePtr &neighbor,int num)
{
  Eigen::Matrix<double, 3, 4> last4Cps;  // Each column contains a control point
  Eigen::Matrix<double, P_ + 1, 3> bspline_diff0_control_points;
  bspline_diff0_control_points = local_control_points;
  bspline_diff0_control_points.row(P_) = neighbor->position.transpose();
  last4Cps = (bspline2bezier_matrix*bspline_diff0_control_points).transpose();
  // std::cout<<"now_collide"<<std::endl;
  bool collides = collidesWithObstaclesGivenVertexes(last4Cps,num);
  return collides;

}

bool RSAstar::collidesWithObstaclesGivenVertexes(const Eigen::Matrix<double, 3, 4>& last4Cps,int num)
{
  bool satisfies_LP = true;
  Eigen::Vector3d n_i;
  double d_i;
  double t_init = time_start + ts*(num-P_);
  // std::cout<<"now_collide3"<<std::endl;
  if(box_detector_->getnumtrack()==0)
  {
    return (!satisfies_LP);
  }
  // std::cout<<"now_collide4"<<std::endl;
  std::vector<Eigen::Matrix<double, 3, Eigen::Dynamic>> obstacles = box_detector_->getIntervalHull(t_init,this->ts);
  // std::cout<<"now_collide5"<<std::endl;
  //   interval  这个第几段是处理障碍物的第几段 
  for (int obst_index = 0; obst_index < obstacles.size(); obst_index++)
  {
    // std::cout<<"now_collide1"<<std::endl;
    satisfies_LP = separator_solver_->solveModel(n_i, d_i, obstacles[obst_index], last4Cps); // 第几个障碍物的这一段
    // std::cout<<"now_collide2"<<std::endl;
    if (satisfies_LP == false)
    {
      goto exit;
    }
  }
  return (!satisfies_LP);
exit:
  return (!satisfies_LP);
}   // 设置的值很重要

bool RSAstar::checkFeasAndFillND(std::vector<Eigen::Vector3d> &q, std::vector<Eigen::Vector3d>& n,
                                       std::vector<double>& d)
{
  int num_of_segments_ = q.size() - P_;
  int num = num_of_segments_*box_detector_->getnumtrack();
  n.resize(std::max(num, 0), Eigen::Vector3d::Zero());
  d.resize(std::max(num, 0), 0.0);
  if(box_detector_->getnumtrack() == 0)
  {
    return true;
  }
  hulls_curve = box_detector_-> getAllIntervalnHull(time_start,ts,num_of_segments_);

  Eigen::Matrix<double, 3, 4> last4Cps;  // Each column contains a control point

  bool isFeasible = true;

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
      bool solved = separator_solver_->solveModel(n_i, d_i, hulls_curve[obst_index][index_interv], last4Cps_new_basis); // 第几个障碍物的这一段
      if (solved == false)
      {
        isFeasible = false;
        return false;
      }
      n[obst_index * num_of_segments_ + index_interv] = n_i;  //第一段每一个障碍物，第二段每一个障碍物。
      d[obst_index * num_of_segments_ + index_interv] = d_i;
  // for (int i = 0; i <= (n_ - 3); i++)  // i  is the interval (\equiv segment) // 是我们的第几段
  // {
  //   for (int obst_index = 0; obst_index < num_of_obst_; obst_index++)
  //   {
  //     int ip = obst_index * num_of_segments_ + i;  // index plane
      if(!checkTrajDynamics1(last4Cps.transpose()))
      {
        isFeasible = false;
      }

    }
  }
  return isFeasible;
}

bool RSAstar::checkTrajDynamics1(Eigen::Matrix<double, 4, 3> local_control_points) {

  if (P_ < 2) {
    cerr << "RSAstar::checkTrajDynamics : degree of bspline is too low" << endl;
    return false;
  }
  Matrix<double, P_ + 1, 3> bspline_diff0_control_points;
  Matrix<double, P_, 3> bezier_diff1_control_points;
  Matrix<double, P_ - 1, 3> bezier_diff2_control_points;

  bspline_diff0_control_points = local_control_points;

  bezier_diff1_control_points = (bspline2bezier_diff1_matrix * bspline_diff0_control_points).block(0, 0, P_, 3);

  bezier_diff2_control_points = (bspline2bezier_diff2_matrix * bspline_diff0_control_points).block(0, 0, P_ - 1, 3);

  if (bezier_diff1_control_points.maxCoeff() > max_vel_ ||   //返回矩阵中最大和最小值
      bezier_diff1_control_points.minCoeff() < -max_vel_ ||
      bezier_diff2_control_points.maxCoeff() > max_acc_ ||
      bezier_diff2_control_points.minCoeff() < -max_acc_
      ) {
    return false;//Not dynamically feasible
  }

  return true;//Dynamically feasible
}

const typename RSAstar::MatrixNArray RSAstar::computeQuadraticCostJacobian() {
  typename RSAstar::MatrixNArray m_array;
  typename RSAstar::MatrixN bspline_basis_matrix;
  bspline_basis_matrix = RSAstar::computeBasisMatrix_Bspline();
  typename RSAstar::MatrixNArray
      quadratic_coefficients = RSAstar::computeQuadraticCoefficients();

  for (int i = 0; i < P_ - 1; ++i) {
    RSAstar::MatrixN m;
    m = bspline_basis_matrix.transpose() *
        quadratic_coefficients[i] * bspline_basis_matrix;

    m_array[i] = m.template cast<double>();
  }
  return m_array;
}

const typename RSAstar::MatrixNArray RSAstar::computeQuadraticCoefficients() {
  typename RSAstar::MatrixNArray res;
  typename RSAstar::MatrixN base_coefficients =
      RSAstar::computeBaseCoefficients();

  for (int derivative = 1; derivative < P_; derivative++) {
    res[derivative - 1].setZero();

    for (int col = 0; col < P_ + 1 - derivative; col++) {
      for (int row = 0; row < P_ + 1 - derivative; row++) {
        double exp = (P_ - derivative) * 2 + 1 - row - col;

        res[derivative - 1](P_ - row, P_ - col) =
            base_coefficients(derivative, P_ + 1 - 1 - row) *
                base_coefficients(derivative, P_ + 1 - 1 - col) * 2 / exp;
      }
    }
  }

  return res;
}

uint64_t RSAstar::C_n_k(uint64_t n, uint64_t k) {
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


typename RSAstar::MatrixN RSAstar::computeBaseCoefficients() {
  typename RSAstar::MatrixN base_coefficients;

  base_coefficients.setZero();
  base_coefficients.row(0).setOnes();

  const int DEG = P_;
  int order = DEG;
  for (int n = 1; n < P_ + 1; n++) {
    for (int i = DEG - order; i < P_ + 1; i++) {
      base_coefficients(n, i) = (order - DEG + i) * base_coefficients(n - 1, i);
    }
    order--;
  }
  return base_coefficients.template cast<double>();
}

const typename RSAstar::MatrixN RSAstar::computeBasisMatrix_Bspline(int diff_degree) {
  if (diff_degree > P_) {
    cerr << "RSAstar::MatrixN RSAstar::computeBasisMatrix_Bspline: diff_degree  = " << diff_degree
         << endl;
    return RSAstar::MatrixN::Zero();
  }

  typename RSAstar::MatrixN m;
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

const typename RSAstar::MatrixN RSAstar::computeBasisMatrix_Bezier(int diff_degree) {
  if (diff_degree > P_) {
    cerr << "RSAstar::MatrixN RSAstar::computeBasisMatrix_Bezier: diff_degree  = " << diff_degree
         << endl;
    return RSAstar::MatrixN::Zero();
  }

  typename RSAstar::MatrixN m;
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

void RSAstar::setEnvironment(const EDTEnvironment::Ptr& env)
{
  this->edt_environment_ = env;
}

void RSAstar::setBoxDetector(const BoxDetector::Ptr &detector_)
{
  this->box_detector_ = detector_;
}

Eigen::Vector3i RSAstar::posToIndex(Eigen::Vector3d pt)
{
  // std::cout<<"111"<<std::endl;
  // std::cout << "Position: " << pt.transpose() << std::endl;
  // std::cout << "Origin: " << origin_.transpose() << std::endl;
  // std::cout << "Resolution: " << inv_resolution_ << std::endl;
  // std::cout << "Resolution: " << pt.transpose()<< std::endl;
  Eigen::Vector3i idx = ((pt - origin_) * inv_resolution_).array().floor().cast<int>();
  idx = idx - this->min_id;
  // std::cout<<"222"<<std::endl;
  // std::cout << "Index: " << idx.transpose() << std::endl;
  return idx;
}

void RSAstar::setinitPathNode(Eigen::Matrix<double, 3, 3> init_control)
{
    PathNodePtr node = new PathNode;
    node->position = init_control.row(0);
    node->index = posToIndex(node->position);
    node->dist = this->edt_environment_->sdf_map_->getDistance(node->position);
    node->g_score = 0;
    node->h_score = getHeu(node, end_node);  //乘了一个数据的值
    node->node_state = IN_CLOSE_SET;
    start_nodes_.push_back(node);

    for (int i= 1;i<P_;i++)
    {
       PathNodePtr temp = new PathNode;
       if(i==1)
       {
          temp->position = init_control.row(i);
          temp->index = posToIndex(temp->position);
          temp->dist = this->edt_environment_->sdf_map_->getDistance(temp->position);
          temp->g_score = 0;
          temp->h_score = getHeu(temp, end_node);  //乘了一个数据的值
          //  std::cout<<"h_score:"<<temp->h_score<<std::endl;
          temp->node_state = IN_CLOSE_SET; 
          temp->parent=node;
          node=temp;
          start_nodes_.push_back(node);
       }
       else
       {
          Eigen::Vector3d pt = init_control.row(i);
          Eigen::Vector3i index = posToIndex(pt);
          // std::cout<<"index:"<<index<<std::endl;
          temp = BSplineGridNodeMap[index(0)][index(1)][index(2)]; 
          temp->position = pt;
          temp->dist = this->edt_environment_->sdf_map_->getDistance(pt);
          temp->g_score = 0;
          temp->h_score = getHeu(temp, end_node);  //乘了一个数据的值
          temp->node_state = IN_OPEN_SET;
          temp->parent=node;
          start_nodes_.push_back(temp);
       }
    }   
}

std::vector<PathNodePtr> RSAstar::retrievePath(PathNodePtr current) {
  std::vector<PathNodePtr> path;
  path.push_back(current);
  while (current->parent != NULL) {
    current = current-> parent;
    path.push_back(current);
  }
  std::reverse(std::begin(path), std::end(path)); 
  return path;
}

void RSAstar::pathnode2Eigen(std::vector<PathNodePtr> &path,std::vector<Eigen::Vector3d> &result)
{
  PathNodePtr current = path.back();   // 零就是第一个
  result.push_back(current->position);
  while (current->parent != NULL) {
    current = current-> parent;
    result.push_back(current->position);
  }
  std::reverse(std::begin(result), std::end(result)); 
}

std::vector<Eigen::Vector3d> RSAstar::getctrlpoint()
{
  return result_;
}

ConvexHullsOfCurves_Std RSAstar::gethulls()
{
  return hulls_curve;
}
std::vector<double> RSAstar::getndresult()
{
  std::vector<double> result;
  std::vector<Eigen::Vector3d> n;
  std::vector<double> d; 
  for(int i = 0;i<n.size();i++ )
  {
    result.push_back(n[i](0));
    result.push_back(n[i](1));
    result.push_back(n[i](2));
  }
  for(int i = 0;i<d.size();i++ )
  {
    result.push_back(d[i]);
  }
  return result;
}

// 初始的三个控制点设置  每一行是一个控制点.
int RSAstar::search(Eigen::Matrix<double, 3, 3> init_control, Eigen::Vector3d end_pt,double time_start)
{

  box_detector_-> StoreAllTrack();
  std::cout<<"ww"<<std::endl;
   // 设置起点，轨迹控制点判断
  this->time_start = time_start;

  PathNodePtr currentPtr  = new PathNode;;
  end_pt_ = end_pt;   // 设置这个终点
  end_node->index = posToIndex(end_pt);
  end_node->position = end_pt_;

  setinitPathNode(init_control); 
  start_node = start_nodes_[P_-1];
  
  openList_.push(start_node);   //这个值真的很重要,把这个点加入算法。

  std::cout << "[A*] Running..." << std::endl;
  // expanded_nodes_.insertopenlist(start_node->index,start_node);

  MyTimer timer_astar(true);  //设置开始的A*设置的范围
  int count = 0;
  while (openList_.size() > 0)
  { 
    clock_t start_time = clock();
    // Check if runtime is over
    currentPtr = openList_.top(); 
    openList_.pop();

    if(currentPtr->node_state == IN_CLOSE_SET)
    {
      continue;
    }

    std::cout<<"openList_:"<<openList_.size()<<std::endl;
    int num_Point2 = getNumControlNow(currentPtr);
    std::cout<<"num_Point:"<<num_Point2<<std::endl;

    count = count + 1; 
    // std::cout<<"currentPtr->in:"<<currentPtr->index.transpose()<<std::endl; 
    currentPtr->node_state = IN_CLOSE_SET;

    if (timer_astar.ElapsedMs() > max_runtime_*1000)    // (max_runtime_ * 1000))  // 
    {
      std::cout << "[A*] Max Runtime was reached" << std::endl;
      if(currentPtr==NULL)
      {
        std::cout<<"NULL"<<std::endl;
        return NO_PATH;
      }
      else
      {

        path_nodes_ = retrievePath(currentPtr); //时间到了得到的轨迹也可以进行判断
        if(path_nodes_.size() > 6)
        {   
            std::cout<<"NULL_length"<<std::endl;
            pathnode2Eigen(path_nodes_,result_);
            bool isFeasible = checkFeasAndFillND(result_, n, d);
            return RUNTIME_REACHED;             // 需要满足一定的长度才有执行曲线的必要
        }
        else
        {
            std::cout<<"NULL_path"<<std::endl;
            return NO_PATH;
        }
      }   
    }
    // std::cout<<"mm"<<std::endl;
    // 到达起点的距离
    double dist = (currentPtr->position - end_pt).norm();
    double dist_start = (currentPtr->position - start_node->position).norm(); // 到开始位置的值，很重要的问题
    // std::cout<<"currentPtr->position:"<<currentPtr->position.transpose()<<std::endl;
    // std::cout<<"dist:"<<dist<<std::endl;
    // std::cout<<"dist_start t:"<<dist_start <<std::endl;
    
    if(dist < 0.2)         // 判断是不是到达了终点，到达了终点
    {
      if(currentPtr->index == end_node->index)
      {
        std::cout<<"dist_equa"<<std::endl;
        //到达了终点的距离
        path_nodes_ = retrievePath(currentPtr);  //到达了终点的值  ,回溯控制点 
        PathNodePtr goal;        // 扩展它三个终点值.
        for(int i=0;i<P_-1;i++)
        { 
          goal->position = end_pt;
          goal->index = posToIndex(end_pt);
          path_nodes_.push_back(goal); // 得到了很重要的值,没有必要父节点是什么.
        }
      }
      else
      {
        //到达了终点的距离
        std::cout<<"dist_end"<<std::endl;
        path_nodes_ = retrievePath(currentPtr);  //到达了终点的值  ,回溯控制点 
        PathNodePtr goal;        // 扩展它三个终点值.
        for(int i=0;i<P_;i++)
        { 
          goal->position=end_pt;
          goal->index = posToIndex(end_pt);
          path_nodes_.push_back(goal); // 得到了很重要的值,没有必要父节点是什么.
        }
      }
      // 开始最后两个控制点在终点。需要多加两个控制点。
      pathnode2Eigen(path_nodes_,result_);
      bool isFeasible = checkFeasAndFillND(result_, n, d);
      return REACH_END;
    }

    if (dist_start > 4)       // 就按照b样条的可行性来吧,时间尽量少一点把.
    {
        std::cout<<"dist_start"<<std::endl;
        path_nodes_ = retrievePath(currentPtr); //到达了相应的位置。
        pathnode2Eigen(path_nodes_,result_);
        bool isFeasible = checkFeasAndFillND(result_, n, d);
        return REACH_HORIZON;
    }
    
    std::vector<PathNodePtr> neighbors;
    // std::cout<<"start_neighbor"<<std::endl;
    
    // 这里是要计时的代码
    Neighbor(currentPtr,neighbors);         // 父节点可以得到
    // std::cout<<"neighbors:"<<neighbors.size()<<std::endl;
    
    setLocalControlPoints(currentPtr); 
    // std::cout<<"neighbors2:"<<neighbors.size()<<std::endl;
    int num_Point1 = getNumControlNow(currentPtr);
    // std::cout<<"num_Point:"<<num_Point1<<std::endl;
    // std::cout<<"currentPtr:"<<currentPtr->position.transpose()<<std::endl;
    // 判断四个 1.closelist 2.动力学约束 3.静态碰撞 4.动态避让
    for (int i = 0; i < neighbors.size(); i++) 
    { 
      PathNodePtr neighborPtr = neighbors[i];
      // std::cout<<"neighbors:"<<neighborPtr->position.transpose()<<std::endl;
      // 是不是在closelist里面
      if(neighborPtr->node_state == IN_CLOSE_SET)
      {
        continue;
      }
      // std::cout<<"add_openlist1"<<std::endl;
      // std::cout<<"neighborPtr->position:"<<neighborPtr->position<<std::endl;
      if(this->edt_environment_->sdf_map_->getInflateOccupancy(neighborPtr->position) == 1)
      {
        continue;
      }
      // std::cout<<"add_openlist2"<<std::endl;

      if (!checkTrajDynamics(neighborPtr)) 
      {
          continue;
      }
      // std::cout<<"add_openlist3"<<std::endl;

      int num_Point = getNumControlNow(currentPtr);
      // std::cout<<"num_Point:"<<num_Point<<std::endl;
      // 检查这四个控制点是否碰撞
      bool collides = collidesWithObstacles(neighborPtr,num_Point); 
      // std::cout<<"collides"<<std::endl;
      if (collides)
      {
         continue;
      }
      // std::cout<<"add_openlist4"<<std::endl;
      collides = CheckStaticObstacle(neighborPtr,num_Point);
      if (collides)
      {
         continue;
      }
      // std::cout<<"add_openlist"<<std::endl;

      double tentative_gScore = currentPtr->g_score + getGeu(neighborPtr,currentPtr); // 找到了g_score的值  

      // std::cout<<"g:"<<tentative_gScore<<std::endl;
      if (neighborPtr->node_state != IN_CLOSE_SET) {       // 判断是不是在openlist里面或者 closelist里面
        //discover a new node
        neighborPtr->parent = currentPtr;
        neighborPtr->g_score = tentative_gScore;
        neighborPtr->h_score =  getHeu(neighborPtr, end_node);
        // std::cout<<"h:"<<neighborPtr->h_score<<std::endl;
        neighborPtr->dist = this->edt_environment_->sdf_map_->getDistance(neighborPtr->position);
        neighborPtr->node_state = IN_OPEN_SET;
        openList_.push(neighborPtr);
        // expanded_nodes_.insertopenlist(neighborPtr->index,neighborPtr);
              //  PathNodePtr temp = new PathNode;

        //std::cout<<"gscore"<<tentative_gScore<<"heu"<<getHeu(neighborPtr, endPtr)<<endl;
        continue;
      // } else if (tentative_gScore < neighborPtr->g_score) {   //需要更新
      //   //in open set and need update
      //   // PathNodePtr temp = new PathNode;
      //   // *temp = *neighborPtr;
      //   // neighborPtr->node_state =true;

      //   neighborPtr->parent = currentPtr;
      //   neighborPtr->g_score = tentative_gScore;
      //   neighborPtr->h_score  =  getHeu(neighborPtr, end_node);
      //   // std::cout<<"h1:"<<neighborPtr->h_score<<std::endl;
      //   neighborPtr->dist = this->edt_environment_->sdf_map_->getDistance(neighborPtr->position);
      //   //put neighbor in open set and record it.
      //   // 把这个点从openlist里面删除。插入到很离谱的控制点，感觉很离谱
      //   // openList_.push(neighborPtr); // 把这个新的加入重要的地位.
      //   PathNodePtr tem = openList_.top();
      //   openList_.pop();
      //   openList_.push(tem);

        // delete temp;
        // expanded_nodes_.open2closelist(neighborPtr->index,neighborPtr);  // 把这个加入到closelist里面的值，很重要的,没什么好删除的直接判断
      }
    }
    // clock_t end_time = clock();
    // double elapsed_time_ms = double(end_time - start_time) / CLOCKS_PER_SEC * 1000;
    // std::cout << "Elapsed time: " << elapsed_time_ms << "ms" << std::endl;
  }
  return NO_PATH;
}

void RSAstar::initGridNodeMap() {   // 本地地图的大小

  Eigen::Vector3i index =this->max_id - this->min_id;

  this->x_size = abs(index(0));
  this->y_size = abs(index(1));
  this->z_size = abs(index(2));
  std::cout<<"size:"<<this->x_size<<this->y_size<<this->z_size<<std::endl;

  BSplineGridNodeMap = new PathNodePtr **[x_size];
  for (int i = 0; i < x_size; i++) {
    BSplineGridNodeMap[i] = new PathNodePtr *[y_size];
    for (int j = 0; j < y_size; j++) {
      BSplineGridNodeMap[i][j] = new PathNodePtr[z_size];
      for (int k = 0; k < z_size; k++) {
        Vector3i tmpIdx(i, j, k);
        BSplineGridNodeMap[i][j][k] = new PathNode(tmpIdx);
      }
    }
  }
}

}
