#ifndef _DETECTOR_H_
#define _DETECTOR_H_


#include <detector/nanodet_openvino.h>
#include <Eigen/Dense>
#include <iostream>
#include <cmath>
#include <vector>
#include <Eigen/QR>
#include <detector/Hungarian.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <queue>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <thread>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <detector/cgal_utils.hpp>
#include <ros/package.h>
#include <chrono>
#include <pcl/surface/convex_hull.h>

struct box3D
{
  float Vx, Vy;
  float x, y, z;
  float x_width, y_width, z_width;
};

struct object_rect {
    int x;
    int y;
    int width;
    int height;
};

struct cluster  // 3D of people
{
  Eigen::Vector3d centroid;
  Eigen::Vector3d bbox;  // Total side x, total side y, total side z;
  double time;           // in seconds   现在的时间值。用于估计拟合轨迹
  double vx = 0;
  double vy = 0;

  void print() const
  {
    std::streamsize ss = std::cout.precision();  // original precision
    std::cout << "Centroid= " << centroid.transpose() << ", " << "bbox= " << bbox.transpose() << " time= " << time << std::endl;
  }
};

struct polynomial
{
  Eigen::VectorXd coeff_x;  // [a b c d ...]' of Int0 
  Eigen::VectorXd coeff_y;  // [a b c d ...]' of Int0 
  // Eigen::VectorXd coeff_z;  // [a b c d ...]' of Int0   // 只有多项式的系数就行了 

  // void clear()
  // {
  //   // times.clear();
  //   // coeff_x.reset();
  //   // coeff_y.reset();
  //   // coeff_z.clear();
    
  // }
  void clear()
  {
    coeff_x.setZero();
    coeff_y.setZero();
  // coeff_z.setZero();
  }

  Eigen::VectorXd eval(double t) const
  {
    if(coeff_x.size() == 0)
    {
      Eigen::VectorXd a(2);
      a.setZero();
      return a;
    }
    Eigen::VectorXd result(2);
    Eigen::VectorXd tmp(coeff_x.size());
    for(int i=0;i < coeff_x.size();i++)
    {
      tmp(i) = pow(t,i);
    }
    result.x() = coeff_x.transpose() * tmp;
    result.y() = coeff_y.transpose() * tmp;
    // result.z() = coeff_z.transpose() * tmp;
    return result;
  }

};

class track
{
public:
  bool is_new = true;
  bool init_fit = false;  
  double t_start;
  double x_start; 
  double y_start;
  double time_traj;
  
  track(cluster& c,  int num)          // 设置的次数，每次拟合点的数量。设置需要多少个点
  {
    this->num = num;
    this->is_new = true;

    history.push_back(c);  // Constant initialization 最小的点数量可以开始了。初始化第一个初始化的值
    initKf(c);
    num_samples = 1;    //更新了几个点就可以开始拟合了，拟合的标志性点，初始化了
    time_traj = c.time;
    
  }

  void addToHistory(cluster& c)
  { 
    // std::cout<<"1"<<std::endl;
    if((c.centroid-getLatestposition()).norm()>0.5)// last and now position
        return;
    
    time_traj = c.time;
    kfEstimate(c);
    //  std::cout<<"2"<<std::endl;
    history.push_back(c);
    if(history.size() > num)
    {
      history.pop_front();
    }
    // num_samples ++;
    // if(num_samples>=4)
    // {
      init_fit= true;
    // }
    

    // if (history.size() > num)   // 到了这么多点开始初始化
    // {
    //   history.pop_front();      // Delete the oldest element
    //   if(!init_fit)
    //   {
    //     polyfit2D();
    //     init_fit= true;
    //   }
    // }
    // // std::cout<<"3"<<std::endl;
    // num_samples = num_samples + 1;
    // if(init_fit && num_samples>int(num*4/5))
    // {
    //   num_samples = 0;
    //   polyfit2D();         //拟合结果呢
    //   // std::cout<<"traj.coeff_x"<<traj.coeff_x.transpose()<<std::endl;
    //   // std::cout<<"traj.coeff_y"<<traj.coeff_y.transpose()<<std::endl;
    // }                 // 碰撞检测是不断更新的
  }

  void polyfit2D()
  {
    std::vector<double> t = getTime();
    std::vector<double> x = getX();
    std::vector<double> y = getY();
    t_start = t[0];
    x_start = x[0];
    y_start = y[0];
    for(int i=0;i<t.size();i++)
    {
       t[i] = t[i]-t_start;
       x[i] = x[i]-x_start;
       y[i] = y[i]-y_start;
    }
    // for(int i=0;i<t.size();i++)
    // {
    //   std::cout<<"t"<<t[i]<<std::endl;
    //   std::cout<<"x"<<x[i]<<std::endl;
    //   std::cout<<"y"<<y[i]<<std::endl;
    // }
    // std::vector<double> z = getZ();
    polyfit(t,x,traj.coeff_x);
    polyfit(t,y,traj.coeff_y);
    // polyfit(t,z,traj.coeff_z);
  }

  void polyfit(	const std::vector<double> &t,const std::vector<double> &v,Eigen::VectorXd &coeff,int order =1)
  {
    // Create Matrix Placeholder of size n x k, n= number of datapoints, k = order of polynomial, for exame k = 3 for cubic polynomial
	  Eigen::MatrixXd T(t.size(), order + 1);
	  Eigen::VectorXd V = Eigen::VectorXd::Map(&v.front(), v.size());
	  Eigen::VectorXd result;

	  // check to make sure inputs are correct
	  assert(t.size() == v.size());
	  assert(t.size() >= order + 1);
	  // Populate the matrix
	  for(size_t i = 0 ; i < t.size(); ++i)
	  {
		  for(size_t j = 0; j < order + 1; ++j)
		  {
			  T(i, j) = pow(t.at(i), j);
		  }
	  }
	  // std::cout<<T<<std::endl;
	  // Solve for linear least square fit
	  result  = T.householderQr().solve(V);
    coeff = result;
  }

  std::vector<double> getTime()
  {
    std::vector<double> t;
    for(int i=0;i < history.size();i++)
    {
      t.push_back(history[i].time);
    }
    return t;
  }
  std::vector<double> getX()
  {
    std::vector<double> x;
    for(int i=0;i < history.size();i++)
    {
      x.push_back(history[i].centroid.x());
    }
    return x;
  }
  std::vector<double> getY()
  {
    std::vector<double> y;
    for(int i=0;i < history.size();i++)
    {
      y.push_back(history[i].centroid.y());
    }
    return y;
  }
  std::vector<double> getZ()
  {
    std::vector<double> z;
    for(int i=0;i < history.size();i++)
    {
      z.push_back(history[i].centroid.z());
    }
    return z;
  }
  double getLatestTime()  // 最后的时间.
  {
    return (history.back().time);
  }

  bool shouldDestory(double t)
  {
    if(t-getLatestTime()>max_seconds_keeping_traj)
    {
      return true;
    }
    else{
      return false;
    }
  }

  Eigen::Vector3d getLatestposition()
  {
    return (history.back().centroid);
  }

  std::pair<double,double> getV()
  {
    double sum_x = 0;
    double sum_y = 0;
    double count = 0;
    cluster b;
    for(int i=1;i<this->history.size();i++)
    {
      b= history[i];
      sum_x += b.vx;
      sum_y += b.vy;
      count ++;
    }
    double x,y;
    // std::cout<<"getV()1"<<std::endl;
    x = sum_x/count;
    y = sum_y/count;
    std::pair<double ,double> result = std::make_pair(x,y);
    // std::cout<<"getV()2"<<std::endl;
    return result;
  }

  cluster getLastCluster()
  {
    return (history.back());
  }


  // std::vector<Eigen::Vector3d> getFuturePositionvis(double duration)   // vis
  // { 
  //   // std::cout<<"------------- predict -------------"<<std::endl;
  //   // std::cout<<"traj.coeff_x"<<traj.coeff_x.transpose()<<std::endl;
  //   // std::cout<<"traj.coeff_y"<<traj.coeff_y.transpose()<<std::endl;
  //   std::vector<Eigen::Vector3d> positions;
  //   positions.push_back(getLatestposition());
  //   double time_now = getLatestTime();
  //   double time_end = time_now + duration;
  //   for (double t= time_now;t<time_end;t = t+0.2)
  //   {
  //     double time = t-t_start;
  //     Eigen::VectorXd position;
  //     // std::cout<<"eval1"<<std::endl;
  //     position = traj.eval(time);
  //     // std::cout<<"eval3"<<std::endl;
  //     double z = getLatestposition().z();  // 只在两个方向拟合了
  //     double x = getLatestposition().x();  // 只在两个方向拟合了position(0)+x_start
  //     // std::cout<<"eval"<<std::endl;
  //     positions.push_back(Eigen::Vector3d(position(0)+x_start,position(1)+y_start,z));
  //     // std::cout<<"t: "<<t<<" "<<"position:"<<position(1)+y_start<<std::endl;
  //   }
  //   // std::cout<<"------------- predict endl -------------"<<std::endl;
  //   return positions;
    
  // }

  // Eigen::Vector3d getOneTimePosition(double time)   // vis
  // { 
  //   Eigen::VectorXd position;
  //   Eigen::Vector3d result;
  //   double time1 = time-t_start;
  //   position = traj.eval(time1);
  //   double z = getLatestposition().z();  // 只在两个方向拟合了
  //   result<<position(0)+x_start,position(1)+y_start,z;  
  //   return result;

  // }

  std::vector<Eigen::Vector3d> getFuturePositionvis(double duration)   // vis
  { 
    // std::cout<<"------------- predict -------------"<<std::endl;
    // std::cout<<"traj.coeff_x"<<traj.coeff_x.transpose()<<std::endl;
    // std::cout<<"traj.coeff_y"<<traj.coeff_y.transpose()<<std::endl;
    std::vector<Eigen::Vector3d> positions;
    std::pair<double,double> v_ = getV();
    Eigen::Vector3d pt = getLatestposition() ;
    positions.push_back(getLatestposition());
    double time_now = getLatestTime();
    double time_end = time_now + duration;
    for (double t= time_now;t<time_end;t = t+0.2)
    {
      double dalta_t = t-time_now;
      double dist_x = dalta_t*v_.first;
      double dist_y = dalta_t*v_.second;

      Eigen::Vector3d position;
      // std::cout<<"getV()3"<<std::endl;
      // std::cout<<"eval3"<<std::endl;
      position << pt(0)+dist_x,pt(1)+dist_y,pt(2);
      // double x = getLatestposition().x();  // 只在两个方向拟合了position(0)+x_start
      // std::cout<<"eval"<<std::endl;
      positions.push_back(position);
      // std::cout<<"getV()4"<<std::endl;
      // std::cout<<"t: "<<t<<" "<<"position:"<<position(1)+y_start<<std::endl;
    }
    // std::cout<<"------------- predict endl -------------"<<std::endl;
    return positions;
    
  }
  
  Eigen::Vector3d getOneTimePosition(double time)   // vis
  { 
    //  std::cout<<"getV()5"<<std::endl;
    std::pair<double,double> v_ = getV();
    Eigen::VectorXd position;
    Eigen::Vector3d result;
    double time_last = getLatestTime();
    double dalta_t = time - time_last;
    position = getLatestposition();
    double dist_x = dalta_t*v_.first;
    double dist_y = dalta_t*v_.second;
    double z = getLatestposition().z();  // 只在两个方向拟合了
    result<<position(0)+dist_x,position(1)+dist_y,z; 
    //  std::cout<<"getV()6"<<std::endl; 
    return result;
  }

  void initKf(cluster a) { 
		// initialize filter 
    Eigen::MatrixXd states_tem(4,1);
		states_tem << a.centroid.x(),
                  a.centroid.y(),
                  0,
                  0;
    this->states = states_tem;  //初始状态估计
    this->time = a.time;
    Eigen::MatrixXd H_temp(2, 4);
    H_temp << 1, 0, 0, 0, 
              0, 1, 0, 0;
    this->H_ = H_temp; 
    // std::cout<<"add a new track"<<std::endl;
   
    P_ = Eigen::MatrixXd::Identity(4, 4) * this->eP_;
	  Q_ = Eigen::MatrixXd::Identity(4, 4) * this->eQ_;
	  R_ = Eigen::MatrixXd::Identity(2, 2) * this->eR_;
    

	}

  void kfEstimate(cluster &b) {
		// predict
    // std::cout<<"af"<<std::endl;
    double dt_ = b.time - getLatestTime();   // sec
    // std::cout<<"getLatestTime():"<<getLatestTime()<<std::endl;
    // std::cout<<"b.time:"<<b.time<<std::endl;
    // std::cout<<"dt_:"<<dt_<<std::endl;
    Eigen::MatrixXd A_tem(4,4);
    Eigen::MatrixXd B_tem(4,2);
    // std::cout<<"ac"<<std::endl;
		A_tem << 1, 0, dt_, 0,
				     0, 1, 0, dt_,
				     0, 0, 1, 0,
				     0, 0, 0, 1;
    // std::cout<<"ahhhh"<<std::endl;
    this->A_ = A_tem;
    // std::cout<<"ag"<<std::endl;
    double dt_2 = 0.5*dt_*dt_;
    Eigen::VectorXd u(2); u<<0,0;      // 二维空间的卡尔曼滤波
    // std::cout<<"ab"<<std::endl;  
    B_tem << dt_2, 0,
             0, dt_2,
             0, 0,    // 速度这里应该也需要
             0, 0;
    this->B_ = B_tem;

    // std::cout<<"a"<<std::endl;
    Eigen::VectorXd z(2);z<< b.centroid(0),b.centroid(1);
    // predict
		states = this->A_ * this->states + this->B_*u;  // 初始化估计
    // std::cout<<"ab"<<std::endl;
		P_ = this->A_ * P_ * this->A_.transpose() + this->Q_;
    // std::cout<<"ac"<<std::endl;
		// update
		Eigen::MatrixXd S = this->R_ + this->H_ * P_ * this->H_.transpose(); // innovation matrix
    // std::cout<<"ad"<<std::endl;
		Eigen::MatrixXd K = P_ * this->H_.transpose() * S.inverse(); // kalman gain
    // std::cout<<"af"<<std::endl;
		states = states  + K * (z - this->H_ * states );
    // std::cout<<"ag"<<std::endl;
		P_ = (Eigen::MatrixXd::Identity(P_.rows(),P_.cols()) - K * this->H_) * P_; 
    // std::cout<<"b"<<std::endl;
    b.centroid.x() = states(0,0);
    b.centroid.y() = states(1,0);
    b.vx = states(2,0);
    b.vy = states(3,0);
    // std::cout<<"position:"<<b.centroid.transpose()<<std::endl;
    // std::cout<<"time:"<<b.time<<std::endl;
    // std::cout<<"----------------"<<std::endl;
    // std::cout<<"states"<<states<<std::endl;

	}
  bool isdynammic()
  { 
    if(!init_fit)
    {
      return false;
    }
    int flag = 0;
    for (int i=0;i<history.size();i++)
    {
      if(sqrt(pow(history[i].vx,2)+pow(history[i].vy,2))>0.1)
      {
        flag++;
      }
    }
    if(flag > int(num*2/3))
    {
      return true;
    }
    else
    {
      return false;
    } 
  }

  Eigen::Vector3d getMaxBbox()
  {
    double max_x = -std::numeric_limits<double>::max();
    double max_y = -std::numeric_limits<double>::max();
    double max_z = -std::numeric_limits<double>::max();

    for (auto& c : history)
    {
      max_x = std::max(c.bbox.x(), max_x);
      max_y = std::max(c.bbox.y(), max_y);
      max_z = std::max(c.bbox.z(), max_z);
    }

    return Eigen::Vector3d(max_x, max_y, max_z);
  }


private:
  int num_samples;                  // 更新点的个数，有多少个点就可以开始拟合了。先要有一个初始化拟合
  int num;                      // 最小多少点可以拟合,拟合的点的个数。
  std::deque<cluster> history;  //[t-N], [t-N+1],...,[t] (i.e. index of the oldest element is 0)  双端队列
  double max_seconds_keeping_traj = 2;   // 轨迹保留的最多的时间。 // 这一条轨迹值不值得，已经进行了合理的检测。
  polynomial traj;

  // KALMAN FILTER	
	// Eigen::MatrixXd states_;
  Eigen::MatrixXd A_;
  Eigen::MatrixXd B_;  //加速度
  // matrix of observation model
  Eigen::MatrixXd H_;
  double eP_ = 0.3;
  double eQ_ = 0.002;
  double eR_ = 0.0001;

  // e_p: 0.3
  // e_q: 0.002
  // e_r: 0.3
  
  Eigen::MatrixXd P_ ;
	Eigen::MatrixXd Q_ ;
	Eigen::MatrixXd R_ ;
  Eigen::MatrixXd states;
  double time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

class BoxDetector
{
  private:
    /*nanodet 类*/
    NanoDet *detector_;
    int height;
    int width;

    double fx_, fy_, cx_,cy_;
    Eigen::Matrix3d orientation_; // current orientation
    Eigen::Vector3d position_; // current position

    std::string depthTopicName_; // depth image topic
    std::string imageTopicName_; // depth image topic
	  std::string poseTopicName_;  // pose topic
    std::string odomTopicName_; // odom topic
    std::string depthoutTopicName_;
    int  localizationMode_; 

    cv::Mat depthImage_;
    cv::Mat rgbImage_;
    // cv::Mat depthImage_;
    Eigen::Matrix4d body2Cam_; // from body frame to camera frame
    double depthScale_; // value / depthScale

    double time;
    std_msgs::Header  header;
    
    std::vector<cluster> clusters;
    std::vector<cluster> clusters_temp;
    std::vector<track> all_tracks_;  // 现在的所有的轨迹
    std::vector<track> tem_All_tracks_;  // 现在的所有的轨迹
    ConvexHullsOfCurves_Std hulls_curves_;
    double ts_,time_st_;


    /*ros*/
    ros::NodeHandle node_;
    ros::Timer dynamicBoxPubTimer_, obstacleTrajPubTimer_ ;

    std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> depthSub_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> imageSub_; 

	 std::shared_ptr<message_filters::Subscriber<geometry_msgs::PoseStamped>> poseSub_;
	 typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image, geometry_msgs::PoseStamped> imagePoseSync;
	 std::shared_ptr<message_filters::Synchronizer<imagePoseSync>> depthPoseSync_;

	 std::shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odomSub_;
	 typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image, nav_msgs::Odometry> imageOdomSync;
	 std::shared_ptr<message_filters::Synchronizer<imageOdomSync>> depthOdomSync_;

   ros::Publisher depthImage_pub_;
   ros::Publisher obstacleTrajPub_;
   ros::Publisher dynamicBoxPub_;
   ros::Publisher people_pub;

   const int color_list[80][3] =
   {
    //{255 ,255 ,255}, //bg
    {216 , 82 , 24},
    {236 ,176 , 31},
    {125 , 46 ,141},
    {118 ,171 , 47},
    { 76 ,189 ,237},
    {238 , 19 , 46},
    { 76 , 76 , 76},
    {153 ,153 ,153},
    {255 ,  0 ,  0},
    {255 ,127 ,  0},
    {190 ,190 ,  0},
    {  0 ,255 ,  0},
    {  0 ,  0 ,255},
    {170 ,  0 ,255},
    { 84 , 84 ,  0},
    { 84 ,170 ,  0},
    { 84 ,255 ,  0},
    {170 , 84 ,  0},
    {170 ,170 ,  0},
    {170 ,255 ,  0},
    {255 , 84 ,  0},
    {255 ,170 ,  0},
    {255 ,255 ,  0},
    {  0 , 84 ,127},
    {  0 ,170 ,127},
    {  0 ,255 ,127},
    { 84 ,  0 ,127},
    { 84 , 84 ,127},
    { 84 ,170 ,127},
    { 84 ,255 ,127},
    {170 ,  0 ,127},
    {170 , 84 ,127},
    {170 ,170 ,127},
    {170 ,255 ,127},
    {255 ,  0 ,127},
    {255 , 84 ,127},
    {255 ,170 ,127},
    {255 ,255 ,127},
    {  0 , 84 ,255},
    {  0 ,170 ,255},
    {  0 ,255 ,255},
    { 84 ,  0 ,255},
    { 84 , 84 ,255},
    { 84 ,170 ,255},
    { 84 ,255 ,255},
    {170 ,  0 ,255},
    {170 , 84 ,255},
    {170 ,170 ,255},
    {170 ,255 ,255},
    {255 ,  0 ,255},
    {255 , 84 ,255},
    {255 ,170 ,255},
    { 42 ,  0 ,  0},
    { 84 ,  0 ,  0},
    {127 ,  0 ,  0},
    {170 ,  0 ,  0},
    {212 ,  0 ,  0},
    {255 ,  0 ,  0},
    {  0 , 42 ,  0},
    {  0 , 84 ,  0},
    {  0 ,127 ,  0},
    {  0 ,170 ,  0},
    {  0 ,212 ,  0},
    {  0 ,255 ,  0},
    {  0 ,  0 , 42},
    {  0 ,  0 , 84},
    {  0 ,  0 ,127},
    {  0 ,  0 ,170},
    {  0 ,  0 ,212},
    {  0 ,  0 ,255},
    {  0 ,  0 ,  0},
    { 36 , 36 , 36},
    { 72 , 72 , 72},
    {109 ,109 ,109},
    {145 ,145 ,145},
    {182 ,182 ,182},
    {218 ,218 ,218},
    {  0 ,113 ,188},
    { 80 ,182 ,188},
    {127 ,127 ,  0},
  };

  
  public:
  BoxDetector()
  {
  }
  ~BoxDetector(){}
  
  template <typename T> 
  inline T norm(const T &x, const T &y){
	  return std::sqrt(std::pow(x,2)+std::pow(y,2));
  }

  template <typename T> 
  inline T distance(const T &Ax, const T &Ay, const T &Bx, const T &By){
	  return norm<T>(Ax-Bx, Ay-By);
  }

  void init(ros::NodeHandle& nh);
  int  resize_uniform(cv::Mat& src, cv::Mat& dst, cv::Size dst_size, object_rect& effect_area);
  void draw_bboxes(const cv::Mat& bgr, const std::vector<BoxInfo>& bboxes, object_rect effect_roi);
  void updatebox();
  double getCostRowColum(cluster& a, track& b);
  void addNewTrack( cluster& c);
  inline void getCameraPose(const nav_msgs::OdometryConstPtr& odom, Eigen::Matrix4d& camPoseMatrix);
  inline void getCameraPose(const geometry_msgs::PoseStampedConstPtr& pose, Eigen::Matrix4d& camPoseMatrix);
  // void imagePoseCB(const sensor_msgs::ImageConstPtr& img,const sensor_msgs::ImageConstPtr& img1, const geometry_msgs::PoseStampedConstPtr& pose);
  // void imageOdomCB(const sensor_msgs::ImageConstPtr& img,const sensor_msgs::ImageConstPtr& img1, const nav_msgs::OdometryConstPtr& odom);
  void imagePoseCB(const cv::Mat& img,const cv::Mat& img1, const geometry_msgs::PoseStampedConstPtr& pose);
  void imageOdomCB(const cv::Mat& img,const cv::Mat& img1, const nav_msgs::OdometryConstPtr& odom);
  void updatetrack(std::vector<track> &all_tracks_);
  void publish3dBox(const std::vector<box3D> &boxes, const ros::Publisher &publisher, const char &color); 
  void Vector3d2geometry(Eigen::Vector3d &position,geometry_msgs::Point &p);
  void publishBestTraj();
  box3D cluster2box3D(cluster tem);
  void obstacleTrajPubCB(const ros::TimerEvent&);
  void dynamicBoxPubCB(const ros::TimerEvent&);
  int getnumtrack();
  // ConvexHullsOfCurve_Std getIntervalHull(double t_init,double ts);
  ConvexHullsOfCurve_Std getIntervalHull(int segment);
  ConvexHullsOfCurve_Std getIntervalHull(int mode,double t_init,double ts);
  ConvexHullsOfCurves_Std getAllIntervalnHull(double t_init,double ts,int num_segment);
  void StoreAllTrack(double time_st,double ts);
  std::vector<std::vector<double>> getboxes();
  ConvexHullsOfCurves_Std gettemphull(int q_segment);

  typedef std::shared_ptr<BoxDetector> Ptr;
   
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};



#endif
















