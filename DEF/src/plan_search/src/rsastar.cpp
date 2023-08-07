
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
  nh.param("search/resolution", resolution_, -1.0);
  nh.param("search/tie_breaker", this->tie_breaker, -1.0);
  nh.param("search/tie_breaker1", this->tie_breaker1, -1.0);
  nh.param("search/tie_breaker2", this->tie_breaker2, -1.0);
  nh.param("search/tie_breaker3", this->tie_breaker3, -1.0);
  nh.param("search/max_runtime", this->max_runtime_, -1.0);
  nh.param("manager/ts", this->ts, -1.0);
  std::cout<<"bb"<<std::endl;
  std::cout<<"search/max_vel:"<<max_vel_<<std::endl;
  std::cout<<"search/tie_breaker:"<<tie_breaker<<std::endl;
  std::cout<<"this->ts:"<<this->ts<<std::endl;
}

void RSAstar::reset(Eigen::Vector3d odom_pos)
{
  // expanded_nodes_.clear();

  std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator> empty_queue;
  openList_.swap(empty_queue);
  
  this->path_nodes_.clear();
  this->result_.clear();
  this->hulls_curve.clear();
  this->n_result.clear();
  this->d_result.clear();

  end_node = new PathNode;
  start_node = new PathNode;
  this->start_nodes_.clear();
  
  
  // std::cout<<"reset"<<std::endl;
  // std::cout<<"odom_pos"<<odom_pos<<std::endl;
  double multiple = 2.0;
  Eigen::Vector3d min_pos_ = odom_pos - Eigen::Vector3d(5.5*multiple,5.5*multiple,4.5*multiple);
  Eigen::Vector3d max_pos_ = odom_pos + Eigen::Vector3d(5.5*multiple,5.5*multiple,4.5*multiple);
  if(min_pos_(0)<-50)
  {
    min_pos_(0) = -50;
  }
  if(min_pos_(1)<-50)
  {
    min_pos_(1) = -50;
  }
  if(min_pos_(2)<0)
  {
    min_pos_(2) = 0;
  }

  if(max_pos_(0)>50)
  {
    max_pos_(0) = 50;
  }
  if(max_pos_(1)>50)
  {
    max_pos_(1) = 50;
  }
  if(max_pos_(2)>5)
  {
    max_pos_(2) = 5;
  }

  // std::cout<<"min_pos_"<<min_pos_<<std::endl;
  // std::cout<<"max_pos_"<<max_pos_<<std::endl;
  // std::cout<<"inv_resolution_:"<<inv_resolution_<<std::endl;

  this->min_id = ((min_pos_ - origin_) * inv_resolution_).array().floor().cast<int>();;
  this->max_id = ((max_pos_ - origin_) * inv_resolution_).array().floor().cast<int>();;

  // this->edt_environment_->sdf_map_->getMinMaxId(this->min_id, this->max_id); 
  this->min_id(2) = 0;
  this->max_id(2) = 5/resolution_;
  // this->min_id(0) = 0;
  // this->max_id(0) = 5/resolution_;
  // this->min_id(1) = 0;
  // this->max_id(1) = 5/resolution_;
  // std::cout<<"min_id"<<min_id<<std::endl;
  // std::cout<<"min_id"<<max_id<<std::endl;

  initGridNodeMap();
  this->positionss.clear();

  
}

void RSAstar::initGridNodeMap() {   // 本地地图的大小

  Eigen::Vector3i index =this->max_id - this->min_id;

  this->x_size = abs(index(0));
  this->y_size = abs(index(1));
  this->z_size = abs(index(2));
  // std::cout<<"size:"<<this->x_size<<this->y_size<<this->z_size<<std::endl;

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
  // std::cout<<"size1:"<<std::endl;
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
  // std::cout<<"cc"<<std::endl;
  this->inv_resolution_ = 1.0 / resolution_;
  edt_environment_->sdf_map_->getRegion(origin_, map_size_3d_);
  origin_(2) = 0;

  // cout << "origin_: " << origin_.transpose() << endl;
  // cout << "map size: " << map_size_3d_.transpose() << endl;  // get map

  double current_pow_inv_dt = 1.0;
  for (int i = 0; i < 2 * (P_ + 1); ++i) 
  {
    pow_inv_dt_[i] = current_pow_inv_dt;   // 时间的间隔的值
    current_pow_inv_dt /= ts;
  }
  // std::cout<<"dd"<<std::endl;
  quadratic_cost_weight.fill(0.0);        
  quadratic_cost_weight = {0,0,1,0};

  diff_matrix = MatrixN::Zero();
  for (int i = 0; i < P_; i++)
  {
     diff_matrix(i, i + 1) = i + 1;
  }
  diff_matrix = diff_matrix /this->ts;
  bspline_basis_matrix = computeBasisMatrix_Bspline();
  bezier_basis_matrix = computeBasisMatrix_Bezier();
  bspline2bezier_matrix = bezier_basis_matrix.inverse() * bspline_basis_matrix;
  quadratic_cost_jacobian = computeQuadraticCostJacobian();


  separator_solver_ = new separator::Separator();
  // std::cout<<"ee"<<std::endl;

  Eigen::MatrixXd trans_matrix(3,4);
  trans_matrix<<-1,1,0,0,
                0 ,-1,1,0,
                0,0,-1,1;
//   Eigen::MatrixXd bspline2bezier_diff1_matrix1 = (computeBasisMatrix_Bezier(1).inverse()).block(0,0,3,3)*computeBasisMatrix_Bspline(1).block(0,0,3,3)*trans_matrix;
  Eigen::MatrixXd bspline_diff1_matrix1 = trans_matrix;
  bspline_diff1_matrix = bspline_diff1_matrix1/this->ts;
  Eigen::MatrixXd transacc_matrix(2,4);
  transacc_matrix<<1,-2,1,0,
			 0 ,1,-2,1;
  Eigen::MatrixXd bspline_diff2_matrix1 = transacc_matrix;//(computeBasisMatrix_Bezier(2).inverse()).block(0,0,2,2)*computeBasisMatrix_Bspline(1).block(0,0,2,2)*transacc_matrix;
  bspline_diff2_matrix = (bspline_diff2_matrix1/this->ts)/this->ts;


  this->dist_bspline = this->ts*max_vel_;
}

double RSAstar::getDiagDis(PathNodePtr node1, PathNodePtr node2) {
  int dx = abs(node1->index(0) - node2->index(0));
  int dy = abs(node1->index(1) - node2->index(1));
  int dz = abs(node1->index(2) - node2->index(2));// x y z 

  // int diag = min(min(dx, dy), dz);// 最小距离
  // dx -= diag;
  // dy -= diag;
  // dz -= diag;  
  // double d = 0;
  // if (dx == 0) {  //判断哪一个是最小的
  //   d = sqrt(3.0) * diag + sqrt(2.0) * min(dy, dz) + abs(dy - dz); //牛的,对角线的值为重要的值，对角线的距离，A*算法对角线距离
  // }
  // if (dy == 0) {
  //   d = sqrt(3.0) * diag + sqrt(2.0) * min(dx, dz) + abs(dx - dz);
  // }
  // if (dz == 0) {
  //   d = sqrt(3.0) * diag + sqrt(2.0) * min(dx, dy) + abs(dx - dy);
  // }
  Eigen::Vector3d pt = node1->position - node2->position;
  // double d = (node1->position - node2->position).norm();
  double d = pt(0)*pt(0)+pt(1)*pt(1)+pt(2)*pt(2)*this->tie_breaker2;
  return d;    // 到终点的距离
}
double RSAstar::getHeu(PathNodePtr node1, PathNodePtr node2,int num) {   // h是启发函数
//   std::cout<<"getHeu:"<<getDiagDis(node1, node2)<<std::endl;
  // double distance = 0;////this->edt_environment_->sdf_map_->getDistance(node1->position);
  // for(int i=0;i<box_detector_->getnumtrack();i++)
  // {
  //   distance+= (node1->position-this->positionss[i][num]).norm();
  // }
  // if(distance==10000)
  // {
  //    distance = 10;
  // }
  // std::cout<<"distance:"<<distance<<std::endl;
  // std::cout<<"getDiagDis(node1, node2):"<<getDiagDis(node1, node2)<<std::endl;
  // double distance = this->edt_environment_->sdf_map_->getDistance(node1->position);
  // double distance;
  // if(node1->dist==10000)
  // {
  //    distance = 10;
  // }
  // else{
  //   distance =node1->dist;
  // }
  // std::cout<<"dist"<<this->tie_breaker *getDiagDis(node1, node2)+this->tie_breaker1*std::exp(-distance)<<std::endl;

  return this->tie_breaker *getDiagDis(node1, node2);//+this->tie_breaker1*distance;  // break,很重要的值.
}

double RSAstar::getGeu(PathNodePtr node1, PathNodePtr node2) {   // h是启发函数
  return getDiagDis(node1, node2);  // break,很重要的值.
}


std::vector<Eigen::Vector3i> RSAstar::Neighbor(Eigen::Vector3i current_idx,double distance) {
  std::vector<Vector3i> nbrs;
  Eigen::Vector3i nbr(0, 0, 0);

  // double current_vel = std::min(distance,0.9*this->dist_bspline)/this->resolution_;
  for(int scale=2;scale<=3;scale++)
  {
    double current_vel = (0.3*scale*this->dist_bspline)/this->resolution_;
    // std::cout<<"current_vel:"<<current_vel<<std::endl;
    if(current_vel<1)
    {
      current_vel =1.01;
    }

    int dir_0_0 = floor(current_vel); // gezishu
    int dir_45_0 = floor(current_vel / 1.414);
    int dir_45_45 = floor(current_vel / 1.732);  //45 - 45 那么就是很重要的值

    for (int i = -1; i < 2; ++i) {
      for (int j = -1; j < 2; ++j) {
        for (int k = -1; k < 2; ++k) {
          switch (abs(i) + abs(j) + abs(k)) {
            case 0:continue;
            case 1:nbr(0) = current_idx(0) + i * dir_0_0;
              nbr(1) = current_idx(1) + j * dir_0_0;
              nbr(2) = current_idx(2) + k * dir_0_0;
              break;
            case 2:nbr(0) = current_idx(0) + i * dir_45_0;
              nbr(1) = current_idx(1) + j * dir_45_0;
              nbr(2) = current_idx(2) + k * dir_45_0;
              break;
            case 3:nbr(0) = current_idx(0) + i * dir_45_45;
              nbr(1) = current_idx(1) + j * dir_45_45;
              nbr(2) = current_idx(2) + k * dir_45_45;
              break;
            default:continue;
          }
          nbrs.push_back(nbr);
        }
      }
    }
  }
  // std::cout<<"nbrs:"<<nbrs.size()<<std::endl;
  return nbrs;
}


bool RSAstar::CheckStaticObstacle(PathNodePtr neighbor) {
    Matrix<double, P_ + 1, 3> Q;
    Q = local_control_points;
    Q.row(P_) = neighbor->position.transpose();

    for(double time=0;time<=1;time=time+0.2)
    {
      double t_use = time;
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
    cerr << "RSAstar degree of bspline is too low" << endl;
    return false;
  }
  Matrix<double, P_ + 1, 3> bspline_diff0_control_points;
  Matrix<double, P_, 3> bspline_diff1_control_points;
  Matrix<double, P_ - 1, 3> bspline_diff2_control_points;

  bspline_diff0_control_points = local_control_points;
  static int count =0;
  if(count==0)
  {
    std::cout<<"local_control_points:"<<local_control_points<<std::endl;
    count++;
  }
  
  
  bspline_diff0_control_points.row(P_) = neighbor->position.transpose();
  bspline_diff1_control_points = (this->bspline_diff1_matrix * bspline_diff0_control_points);//.block(0, 0, P_, 3);
  bspline_diff2_control_points = (this->bspline_diff2_matrix * bspline_diff0_control_points);//.block(0, 0, P_ - 1, 3);
 
  if (bspline_diff1_control_points.maxCoeff() > max_vel_ ||   //返回矩阵中最大和最小值
      bspline_diff1_control_points.minCoeff() < -max_vel_ ||
      bspline_diff2_control_points.maxCoeff() > max_acc_ ||
      bspline_diff2_control_points.minCoeff() < -max_acc_
      ) {
    return false;//Not dynamically feasible
  }

  return true;//Dynamically feasible
}

void RSAstar::setLocalControlPoints(PathNodePtr current) {
  for (int i = 0; i < P_; i++) {
    this->local_control_points.row(P_ - 1 - i) = current->position.transpose();
    if (current->parent != NULL)
      current = current->parent;     
    else {

    }
  }
  return;
}

const typename RSAstar::MatrixNArray RSAstar::computeQuadraticCostJacobian() {
  typename RSAstar::MatrixNArray m_array;
  typename RSAstar::MatrixN bspline_basis_matrix;
  bspline_basis_matrix = RSAstar::computeBasisMatrix_Bspline();
  typename RSAstar::MatrixNArray
      quadratic_coefficients = RSAstar::computeQuadraticCoefficients();

  for (int i = 0; i < P_ ; ++i) {
    RSAstar::MatrixN m;
    m = bspline_basis_matrix.transpose() *
        quadratic_coefficients[i] * bspline_basis_matrix;

    m_array[i] = m.template cast<double>();
  }
  std::cout<<"m_array[2]"<<m_array[2]<<std::endl;
  return m_array;
}

const typename RSAstar::MatrixNArray RSAstar::computeQuadraticCoefficients() {
  typename RSAstar::MatrixNArray res;
  typename RSAstar::MatrixN base_coefficients =
      RSAstar::computeBaseCoefficients();

  for (int derivative = 1; derivative <= P_; derivative++) {
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

   std::cout<<"m_array[2]"<<res[2]<<std::endl;
  return res;
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
  std::cout<<"m_array[2]"<<base_coefficients<<std::endl;
  return base_coefficients.template cast<double>();
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

double RSAstar::getgscore(PathNodePtr tentative) {
   
  double cost = 0; 
  cost = cost + quadraticCost(3, tentative);    // 给了一个系数的值，很重要的了。
  // std::cout<<"cost:"<<cost<<std::endl;
  return cost;
}


double RSAstar::quadraticCost(int deriv, PathNodePtr tentative) {
//    std::cout<<"ada"<<quadratic_cost_jacobian[deriv - 1]<<std::endl;
//    std::cout<<"ada"<<pow_inv_dt_[2 * deriv - 1]<<std::endl;

  return quadraticCost(quadratic_cost_jacobian[deriv - 1] * pow_inv_dt_[2 * deriv - 1], tentative);
}

double RSAstar::quadraticCost(const RSAstar::MatrixN &quadratic_cost,
                                        PathNodePtr tentative) {
//   int controlpts_num = 1;
//   vector<Vector3d> controlpts_coord;
//   PathNodePtr it = tentative;
//   for (controlpts_coord.push_back(it->position); it->parent != NULL;) {  // 回溯前面所有的点
//     controlpts_coord.push_back(it->position);
//     controlpts_num++;
//     // std::cout<<controlpts_num<<std::endl;
//     it = it->parent;
//     if (controlpts_num == P_ + 1) break;
//   }
  vector<Vector3d> controlpts_coord;
  for(int i=0;i<P_;i++)
  {
    controlpts_coord.push_back(this->local_control_points.row(i));
  }
  controlpts_coord.push_back(tentative->position);
  if (controlpts_coord.size() < P_ + 1) 
  {
    // std::cout<<"1111"<<std::endl;
    return 0;
  }

  double inc_cost[3];
  for (int i = 0; i < 3; i++) {
    VectorN coefficients;
    for (int j = 0; j < P_ + 1; j++)
      coefficients[j] = controlpts_coord[j](i);  // 当前的4个控制点
    inc_cost[i] = coefficients.transpose() * quadratic_cost * coefficients;   // x y z 三个方向的值
  }
  return inc_cost[0] + inc_cost[1] + inc_cost[2] ;//+ tentative->g_score;  // 之前的值，很重要的
}

bool RSAstar::collidesWithObstacles(PathNodePtr neighbor,int num)
{
  Eigen::Matrix<double, 3, 4> last4Cps;  // Each column contains a control point
  Eigen::Matrix<double, P_ + 1, 3> bspline_diff0_control_points;
  bspline_diff0_control_points = local_control_points;
  bspline_diff0_control_points.row(P_) = neighbor->position.transpose();
  last4Cps = (this->bspline2bezier_matrix*bspline_diff0_control_points).transpose();
  bool collides = collidesWithObstaclesGivenVertexes(last4Cps,num);
//   std::cout<<"neighbors_index6"<<std::endl;
  return collides;

}

bool RSAstar::collidesWithObstaclesGivenVertexes(const Eigen::Matrix<double, 3, 4>& last4Cps,int num)
{
  if(box_detector_->getnumtrack()==0)
  {
    return false;
  }

  bool satisfies_LP = true;
  Eigen::Vector3d n_i;
  double d_i;
  // double t_init = this->time_start + ts*(num-P_);
  int segment = num - P_;
  for(int k =0;k<1;k++)
  {
    if(segment+k<0)
    {
      continue;
    }
  std::vector<Eigen::Matrix<double, 3, Eigen::Dynamic>> obstacles = box_detector_->getIntervalHull(segment+k);  
  for (int obst_index = 0; obst_index < obstacles.size(); obst_index++)
  {
    auto start_time = std::chrono::high_resolution_clock::now();
    satisfies_LP = separator_solver_->solveModel(n_i, d_i, obstacles[obst_index], last4Cps); // 第几个障碍物的这一段
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    // std::cout << "执行时间glpk: " << duration.count() << " wei秒" << std::endl;

    if (satisfies_LP == false)
    {
      return (!satisfies_LP);
    }
  }
  }
  return (!satisfies_LP);
}   

bool RSAstar::checkFeasAndFillND(std::vector<Eigen::Vector3d> &q, std::vector<Eigen::Vector3d>& n,
                                       std::vector<double>& d)
{

    if(box_detector_->getnumtrack() == 0)
    {
        return true;
    }

    int num_of_segments_ = q.size() - P_;
    int num = num_of_segments_*box_detector_->getnumtrack();
    n.resize(std::max(num, 0), Eigen::Vector3d::Zero());
    d.resize(std::max(num, 0), 0.0);
	  // this->hulls_curve = box_detector_-> getAllIntervalnHull(this->time_start,ts,num_of_segments_);
    this->hulls_curve = box_detector_->gettemphull(num_of_segments_);

       
    Eigen::Matrix<double, 3, 4> last4Cps;  // Each column contains a control point

    // Check obstacles constraints (and compute n and d)

    // std::cout<<"num_of_segments_:"<<num_of_segments_<<std::endl;
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
			// for(int jj=0;jj<hulls_curve[obst_index][index_interv].cols();jj++)
			// {
			// 	std::cout<<"cols:"<<hulls_curve[obst_index][index_interv].col(jj)<<std::endl;
			// }
			    //  std::cout<<last4Cps<<std::endl;
            bool solved = separator_solver_->solveModel(n_i, d_i, hulls_curve[obst_index][index_interv], last4Cps_new_basis); // 第几个障碍物的这一段
			// std::cout<<"n_i:"<<n_i.transpose()<<std::endl;
			// std::cout<<"d_i:"<<d_i<<std::endl;
            if (solved == false)
            {
               return false;
            }
            n[obst_index * num_of_segments_ + index_interv] = n_i;  //第一段每一个障碍物，第二段每一个障碍物。
            d[obst_index * num_of_segments_ + index_interv] = d_i;

          }
    }
        
    return true;
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

void RSAstar::setinitPathNode(Eigen::Matrix<double, 3, 3> init_control)
{
    PathNodePtr node = new PathNode;
    node->position = init_control.row(0);
    // std::cout<<"pt"<<node->position<<std::endl;
    node->index = posToIndex(node->position);
    node->dist = this->edt_environment_->sdf_map_->getDistance(node->position);
    node->g_score = 0;
    node->h_score = 0;//getHeu(node, end_node);  //乘了一个数据的值
    node->node_state = IN_CLOSE_SET;
    this->start_nodes_.push_back(node);
    // std::cout<<"ff"<<std::endl;

    for (int i= 1;i<P_;i++)
    {
       PathNodePtr temp = new PathNode;
       if(i==1)
       {
          temp->position = init_control.row(i);
          // std::cout<<"pt"<<temp->position<<std::endl;
          temp->index = posToIndex(temp->position);
          temp->dist = this->edt_environment_->sdf_map_->getDistance(temp->position);
          temp->g_score = 0;
          temp->h_score = 0;   //getHeu(temp, end_node);  //乘了一个数据的值
          //  std::cout<<"h_score:"<<temp->h_score<<std::endl;
          temp->node_state = IN_CLOSE_SET; 
          temp->parent=node;
          node=temp;
          this->start_nodes_.push_back(node);
          // std::cout<<"ff1"<<std::endl;
       }
       else
       {
          Eigen::Vector3d pt = init_control.row(i);
          Eigen::Vector3i index = posToIndex(pt);
          // std::cout<<"pt"<<pt<<std::endl;
          //   std::cout<<"min_id"<<min_id<<std::endl;
          //   std::cout<<"min_id"<<max_id<<std::endl;
          // std::cout<<"index"<<index(0)<<" "<<index(1)<<" "<<index(2)<<std::endl;
          // std::cout<<"index"<<x_size<<" "<<y_size<<" "<<z_size<<std::endl;
          // std::cout<<"ff22"<<std::endl;
          if(index(2)<0)
          {
            index(2)=0;
          }
          temp = BSplineGridNodeMap[index(0)][index(1)][index(2)];
          // std::cout<<"ff21"<<std::endl; 
          temp->position = pt;
          temp->dist = this->edt_environment_->sdf_map_->getDistance(pt);
          temp->g_score = 0;
          temp->h_score = getHeu(temp, end_node,0);  //乘了一个数据的值
          temp->node_state = IN_OPEN_SET;
          temp->parent=node;
          this->start_nodes_.push_back(temp);
          // std::cout<<"ff2"<<std::endl;
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
//   PathNodePtr current = path.back();   // 零就是第一个
//   result.push_back(current->position);
//   while (current->parent != NULL) {
//     current = current-> parent;
//     result.push_back(current->position);
//   }
//   std::reverse(std::begin(result), std::end(result)); 
  for(int i=0;i<path.size();i++)
  {
    result.push_back(path[i]->position);
  }

}

Eigen::MatrixXd RSAstar::getctrlpoint()
{
    Eigen::MatrixXd ctr_pt;
    ctr_pt = Eigen::MatrixXd(3,this->result_.size());
    for(int i=0;i<this->result_.size();i++)
    {
      ctr_pt.col(i) = result_[i];
    }
    return ctr_pt;
}

ConvexHullsOfCurves_Std RSAstar::gethulls()
{
  return hulls_curve;
}
std::vector<double> RSAstar::getndresult()
{
  std::vector<double> result; 
  for(int i = 0;i<this->n_result.size();i++ )
  {
    result.push_back(n_result[i](0));
    result.push_back(n_result[i](1));
    result.push_back(n_result[i](2));
  }
  for(int i = 0;i<this->d_result.size();i++ )
  {
    result.push_back(this->d_result[i]);
  }
  return result;
}

PathNodePtr RSAstar::index2gridNodePtr(Eigen::Vector3i index) {
  return BSplineGridNodeMap[index(0)][index(1)][index(2)];
}

Eigen::Vector3d RSAstar::gridIndex2coord(Eigen::Vector3i index) {
  Vector3d pt;
  index = index+this->min_id;

  pt(0) = ((double) index(0) + 0.5) * resolution_ + origin_(0);
  pt(1) = ((double) index(1) + 0.5) * resolution_ + origin_(1);
  pt(2) = ((double) index(2) + 0.5) * resolution_ + origin_(2);

  return pt;
}

Eigen::Vector3i RSAstar::posToIndex(Eigen::Vector3d pt)
{
  Eigen::Vector3i idx = ((pt - origin_) * inv_resolution_).array().floor().cast<int>();
  idx = idx - this->min_id;
  return idx;
}

std::vector<std::vector<Eigen::Vector3d>> RSAstar::getpositionss()
{
  return this->positionss;
}

double RSAstar::Distoobs(PathNodePtr obsnode,int num_point)
{
  // auto start_time = std::chrono::high_resolution_clock::now();
  int num = num_point - P_;
  double distance = 0;
  for(int i=0;i<this->positionss.size();i++)
  {
    distance += (this->positionss[i][num]-obsnode->position).norm();
  }

  // auto end_time = std::chrono::high_resolution_clock::now();
  // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
  // std::cout << "执行时间xunhan: " << duration.count() << " wei秒" << std::endl;
  // std::cout<<"distance:"<<distance<<std::endl;
  if(distance>2)
  {
    distance =0;
  }
  else
  {
    distance = 1/std::exp(distance);
  }
  
  return distance;
}

// 初始的三个控制点设置  每一行是一个控制点.
int RSAstar::search(Eigen::Matrix<double, 3, 3> init_control, Eigen::Vector3d end_pt,double time_start_)
{

  box_detector_-> StoreAllTrack(time_start_,this->ts,this->positionss);// now status
  this->time_start = time_start_;
  
  // std::cout<<"end_pt_1:"<<std::endl;
  // std::cout<<"end_pt_:"<<end_pt<<std::endl;
  Eigen::Vector3d temp_start = init_control.row(2);
  double dist_end_pt_ = (temp_start - end_pt).norm();
  Eigen::Vector3d direct = end_pt-temp_start;
  Eigen::Vector3d temp_end_pt_;
  for(double count =0;count<dist_end_pt_;count+=0.3)
  {
    temp_end_pt_ = temp_start+(count/dist_end_pt_)*direct;
    // std::cout<<"end_pt_:"<<temp_end_pt_ <<std::endl;
    Eigen::Vector3i index = posToIndex(temp_end_pt_);
    // std::cout<<"index:"<<index <<std::endl;
    if (index(0) < -5 || index(0) >= x_size
          || index(1) < -5 || index(1) >= y_size
          || index(2) < -5 || index(2) >= z_size)
    {
       
       break;
    }
    else{
        this->end_pt_ = temp_end_pt_;
        continue;
    }

  }
  // std::cout<<"end_pt_2:"<<std::endl;

  //   this->end_pt_ = end_pt;   // 设置这个终点
  end_node->index = posToIndex(this->end_pt_);
  end_node->position = this->end_pt_;
  // std::cout<<"end_pt_3:"<<std::endl;

  setinitPathNode(init_control); 
  // std::cout<<"end_pt_4:"<<std::endl;
  // std::cout<<"end_pt_6:"<<std::endl;
  start_node = start_nodes_[P_-1];
  std::cout<<start_node->node_state<<std::endl;
  // std::cout<<"end_pt_3:"<<std::endl;
  
  openList_.push(start_node);   //这个值真的很重要,把这个点加入算法。
  std::vector<Eigen::Vector3i> neighbors_index;
  // std::cout<<"end_pt_4:"<<std::endl;
  PathNodePtr neighborPtr = new PathNode;;
  std::cout << "[A*] Running..." << std::endl;
  PathNodePtr currentPtr  = new PathNode;
  // std::cout<<"end_pt_1:"<<end_pt_<<std::endl;

  double tentative_gScore; //实验性的值，展示不确定的值
  int num_iter = 0;
  while (openList_.size() > 0)
  { 
    // std::cout<<"end_pt_ggggggg:"<<std::endl;
    num_iter++;
    // std::cout<<"num_iter:"<<num_iter<<std::endl;
    currentPtr = openList_.top(); 
    openList_.pop();

    if(currentPtr->node_state == IN_CLOSE_SET)
    {
      // std::cout<<"openList_23332"<<std::endl;
      continue;
    }
    
    // std::cout<<"currentPtr:"<<currentPtr->position.transpose()<<std::endl;
    // int num_Point = getNumControlNow(currentPtr);
    // std::cout<<"num_Point:"<<num_Point<<std::endl;
    // std::cout<<"openList_:"<<openList_.size()<<std::endl;
    currentPtr->node_state = IN_CLOSE_SET;

    double dist = (currentPtr->position - this->end_pt_).norm();
    // Eigen::Vector3d dist_start = currentPtr->position - start_node->position; // 到开始位置的值，很重要的问题
    double dist_start = (currentPtr->position - start_node->position).norm();
    // std::cout<<"openList_222"<<std::endl;
    // end_pt;
    if(dist < 0.2)         // 判断是不是到达了终点，到达了终点
    {
      // std::cout<<"end22"<<std::endl;
      if(currentPtr->index == end_node->index)
      {
        // 到达了终点的距离
        // std::cout<<"end33"<<std::endl;
        path_nodes_ = retrievePath(currentPtr);  //到达了终点的值  ,回溯控制点 
        // PathNodePtr goal = new PathNode;        // 扩展它三个终点值.
        // PathNodePtr goal1 = new PathNode;
        for(int i=0;i<P_;i++)
        { 
          PathNodePtr goal = new PathNode;
          goal->position=end_node->position;
          goal->index = posToIndex(end_node->position);
          path_nodes_.push_back(goal); // 得到了很重要的值,没有必要父节点是什么.
        }
        // std::cout<<"end33"<<std::endl;
        // for(int i=0;i<P_-1;i++)
        // { 
        //   goal->position = end_node->position;
        //   goal->index = posToIndex(end_node->position);
        //   path_nodes_.push_back(goal); // 得到了很重要的值,没有必要父节点是什么.
        // }

          // goal->position = end_node->position;
          // goal->index = posToIndex(end_node->position);
          // path_nodes_.push_back(goal);
          // goal1->position = end_node->position;
          // goal1->index = posToIndex(end_node->position);
          // path_nodes_.push_back(goal1);

      }
      else
      {
        // std::cout<<"end44"<<std::endl;
        path_nodes_ = retrievePath(currentPtr);  //到达了终点的值  ,回溯控制点 
        for(int i=0;i<P_;i++)
        { 
          PathNodePtr goal = new PathNode;
          goal->position=end_node->position;
          goal->index = posToIndex(end_node->position);
          path_nodes_.push_back(goal); // 得到了很重要的值,没有必要父节点是什么.
        }
      }
      // std::cout<<"end55"<<std::endl;
      pathnode2Eigen(path_nodes_,this->result_);
      // std::cout<<"end66"<<std::endl;
      bool isFeasible = checkFeasAndFillND(this->result_, this->n_result, this->d_result);
      // std::cout<<"fff22"<<std::endl;
      return REACH_END;
    }

    // if (std::abs(dist_start.maxCoeff()) > 4.0||std::abs(dist_start.minCoeff())>4.0)       // 就按照b样条的可行性来吧,时间尽量少一点把.
    if (dist_start>4)
    {
        // std::cout<<"end11"<<std::endl;
        path_nodes_ = retrievePath(currentPtr); //到达了相应的位置。
        pathnode2Eigen(path_nodes_,this->result_);
        bool isFeasible = checkFeasAndFillND(this->result_, this->n_result, this->d_result);
        // std::cout<<"fff"<<std::endl;
        return REACH_HORIZON;
    }
    // std::cout<<"openList_333"<<std::endl;

    
    setLocalControlPoints(currentPtr);
    neighbors_index = Neighbor(currentPtr->index,currentPtr->dist);
    // std::cout<<"neighbors_index"<<neighbors_index.size()<<std::endl;

    // 判断四个 1.closelist 2.动力学约束 3.静态碰撞 4.动态避让
    // auto start_time = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < neighbors_index.size(); i++) 
    {
      if (neighbors_index[i](0) < 0 || neighbors_index[i](0) >= x_size
          || neighbors_index[i](1) < 0 || neighbors_index[i](1) >= y_size
          || neighbors_index[i](2) < 0 || neighbors_index[i](2) >= z_size)
          continue;
      
      neighborPtr = index2gridNodePtr(neighbors_index[i]);
      neighborPtr->position = gridIndex2coord(neighbors_index[i]);

    //   std::cout<<"neighbors_index1"<<std::endl;
      if(neighborPtr->node_state == IN_CLOSE_SET)
      {
        continue;
      }
    //   std::cout<<"neighbors_index2"<<std::endl;
      if(this->edt_environment_->sdf_map_->getInflateOccupancy(neighborPtr->position) == 1)
      {
        continue;
      }
      if(this->edt_environment_->sdf_map_->getDistance(neighborPtr->position) <=0.15)
      {
        continue;
      }

    //   std::cout<<"neighbors_index3"<<std::endl;
      bool collides = CheckStaticObstacle(neighborPtr);
      if (collides)
      {
         continue;
      }
    //   std::cout<<"neighbors_index4"<<std::endl;
      if (!checkTrajDynamics(neighborPtr)) {
        continue;
      }
    //   std::cout<<"neighbors_index7"<<std::endl;

      int num_Point = getNumControlNow(currentPtr);
      // auto start_time = std::chrono::high_resolution_clock::now();
      bool collides1 = collidesWithObstacles(neighborPtr,num_Point);
      // auto end_time = std::chrono::high_resolution_clock::now();
      // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
      // std::cout << "执行时间coll: " << duration.count() << " wei秒" << std::endl;
      
      if (collides1)
      {
         continue;
      }
    //   std::cout<<"neighbors_index5"<<std::endl;


      // double static_cost = 1.0;
      // tentative_gScore = currentPtr->g_score  + getgscore(neighborPtr);  //g_score，设置好了值 getGeu
      // auto start_time = std::chrono::high_resolution_clock::now();
      tentative_gScore = currentPtr->g_score  + getGeu(currentPtr,neighborPtr)+this->tie_breaker1*getgscore(neighborPtr)+this->tie_breaker3*Distoobs(neighborPtr,num_Point); 
      // auto end_time = std::chrono::high_resolution_clock::now();
      // auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time);
      // std::cout << "执行时间xunhan: " << duration.count() << " na秒" << std::endl;


      if (neighborPtr->node_state == NOT_EXPAND) {    // 判断是不是在openlist里面. 

        neighborPtr->parent = currentPtr;
        neighborPtr->g_score = tentative_gScore;
        neighborPtr->h_score = getHeu(neighborPtr, end_node,num_Point-P_);
        neighborPtr->node_state = IN_OPEN_SET;
        neighborPtr->dist = this->edt_environment_->sdf_map_->getDistance(neighborPtr->position);
        openList_.push(neighborPtr);

        continue;
      } else if (tentative_gScore <= neighborPtr->g_score) {   //需要更新
        //in open set and need update
        neighborPtr->parent = currentPtr;
        neighborPtr->g_score = tentative_gScore;
        neighborPtr->h_score = getHeu(neighborPtr, end_node,num_Point-P_);
        neighborPtr->dist = this->edt_environment_->sdf_map_->getDistance(neighborPtr->position);
        openList_.push(neighborPtr);

      }

    }
    // auto end_time = std::chrono::high_resolution_clock::now();
    // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    // std::cout << "执行时间xunhan: " << duration.count() << " wei秒" << std::endl;

    
  }
  return NO_PATH;
}

}
