#include <detector/Boxdetector.h>

void BoxDetector::init(ros::NodeHandle& nh) {

    std::string package_path = ros::package::getPath("detector");
    std::string model_path_string = package_path + "/cfg/nanodet-plus-m_416_openvino.xml";
    // 转换为const char*类型
    const char* model_path = model_path_string.c_str();
    detector_ = new NanoDet(model_path); // 实例化 NanoDet 类的对象 在这里实例化了
    this->height = detector_->input_size[0];
    this->width = detector_->input_size[1];

    this->body2Cam_ << 0.0, 0.0, 1.0, 0.0,
      -1.0, 0.0, 0.0, 0.0,
      0.0, -1.0, 0.0, -0.02,
      0.0, 0.0, 0.0, 1.0;


    node_ = nh;
    node_.param("sdf_map/fx", this->fx_, -1.0);
    node_.param("sdf_map/fy", this->fy_, -1.0);
    node_.param("sdf_map/cx", this->cx_, -1.0);
    node_.param("sdf_map/cy", this->cy_, -1.0);
    node_.param("sdf_map/k_depth_scaling_factor", this->depthScale_, -1.0);

	// depthImage_pub_ = node_.advertise<sensor_msgs::Image>(this->depthoutTopicName_, 10);
  this->pose_pub = this->node_.advertise<geometry_msgs::PoseStamped>("/my_pose_topic", 40);
  this->obstacleTrajPub_ = this->node_.advertise<visualization_msgs::MarkerArray>( "/traj_marker", 10);
  this->dynamicBoxPub_ = this->node_.advertise<visualization_msgs::MarkerArray>("/box_visualization_marker", 10);
	this->dynamicBoxPubTimer_ = this->node_.createTimer(ros::Duration(0.2), &BoxDetector::dynamicBoxPubCB, this);
	this->obstacleTrajPubTimer_ = this->node_.createTimer(ros::Duration(0.2),&BoxDetector::obstacleTrajPubCB, this);
  this->people_pub = node_.advertise<sensor_msgs::PointCloud2>("/camera/point/output", 10);
  this->boximage_pub = node_.advertise<sensor_msgs::Image>("/box/image/output", 10);  
}

void BoxDetector::dynamicBoxPubCB(const ros::TimerEvent&){
    std::vector<box3D> tem;
    for (int i =0;i<all_tracks_.size();i++)
    {
        cluster tem1 = all_tracks_[i].getLastCluster(); // last cluster.
        tem.push_back(cluster2box3D(tem1));
    }
    // std::cout<<"boxa"<<std::endl;
    if(tem.size()!=0)
    {
		   this->publish3dBox(tem, this->dynamicBoxPub_, 'b');
      //  std::cout<<"boxb"<<std::endl;
    }
}

void BoxDetector::obstacleTrajPubCB(const ros::TimerEvent&){	
	this->publishBestTraj();
}

box3D BoxDetector::cluster2box3D(cluster tem)
{
  box3D tem1;
  tem1.x_width = tem.bbox.x();
  tem1.y_width = tem.bbox.y();
  tem1.z_width = tem.bbox.z();
  tem1.x = tem.centroid.x();
  tem1.y = tem.centroid.y();
  tem1.z = tem.centroid.z();
  tem1.Vx =tem .vx;
  tem1.Vy =tem .vy;
  return tem1;

}

void BoxDetector::publishBestTraj(){       // 打印预测的轨迹
    // std::cout<<"all_tracks_.size()"<<all_tracks_.size()<<std::endl;
    if(all_tracks_.size()==0)
    {
      return;
    }
		visualization_msgs::Marker line; 
		visualization_msgs::MarkerArray lines;   
		line.header.frame_id = "world";
		line.type = visualization_msgs::Marker::LINE_LIST;
		line.action = visualization_msgs::Marker::ADD;
		line.scale.x = 0.05;
		
		line.color.g = 1.0;
		line.color.a = 1.0;
		line.lifetime = ros::Duration(0.6);
		for (int i=0 ; i<this->all_tracks_.size() ; i++) {
			if (all_tracks_[i].init_fit == false)//||all_tracks_[i].isdynammic() == false) 
      {
				continue;  
			}  
      std::vector<Eigen::Vector3d> positions = all_tracks_[i].getFuturePositionvis(2); // 未来多长时间的值
      // std::cout<<"boxgg"<<std::endl; 

			for (int k=0; k<positions.size()-1 ; k++) {
        geometry_msgs::Point p1,p2 ;
        Vector3d2geometry(positions[k],p1);
        Vector3d2geometry(positions[k+1],p2);
				line.points.push_back(p1);
				line.points.push_back(p2);
			}
			
			lines.markers.push_back(line); 
			line.id++;
		}
		this->obstacleTrajPub_.publish(lines);
}

void BoxDetector::Vector3d2geometry(Eigen::Vector3d &pt,geometry_msgs::Point &p)
{
  p.x=pt.x();
  p.y=pt.y();
  p.z=pt.z();
}

void BoxDetector::publish3dBox(const std::vector<box3D> &boxes, const ros::Publisher &publisher, const char &color) {

		// visualization using bounding boxes 
		visualization_msgs::Marker line;
		visualization_msgs::MarkerArray lines;
		line.header.frame_id = "world";
		line.type = visualization_msgs::Marker::LINE_LIST;
		line.action = visualization_msgs::Marker::ADD;
		line.ns = "box3D";	
		line.scale.x = 0.1;

		if (color=='g') {
			line.color.g = 1.0;
		}
		else if (color=='b') {
			line.color.b = 1.0;
		}
		else {
			line.color.r = 1.0;
		}

		line.color.a = 1.0;
		line.lifetime = ros::Duration(0.6);

		for(size_t i = 0; i < boxes.size(); i++){
			// visualization msgs

			double x = boxes[i].x; 
			double y = boxes[i].y; 
			double z = boxes[i].z; 

			double x_width = std::max(boxes[i].x_width,boxes[i].y_width);
			double y_width = std::max(boxes[i].x_width,boxes[i].y_width);
			double z_width = boxes[i].z_width;
			
			vector<geometry_msgs::Point> verts;
			geometry_msgs::Point p;
			// vertice 0
			p.x = x-x_width / 2.; p.y = y-y_width / 2.; p.z = z-z_width / 2.;
			verts.push_back(p);

			// vertice 1
			p.x = x-x_width / 2.; p.y = y+y_width / 2.; p.z = z-z_width / 2.;
			verts.push_back(p);

			// vertice 2
			p.x = x+x_width / 2.; p.y = y+y_width / 2.; p.z = z-z_width / 2.;
			verts.push_back(p);

			// vertice 3
			p.x = x+x_width / 2.; p.y = y-y_width / 2.; p.z = z-z_width / 2.;
			verts.push_back(p);

			// vertice 4
			p.x = x-x_width / 2.; p.y = y-y_width / 2.; p.z = z+z_width / 2.;
			verts.push_back(p);

			// vertice 5
			p.x = x-x_width / 2.; p.y = y+y_width / 2.; p.z = z+z_width / 2.;
			verts.push_back(p);

			// vertice 6
			p.x = x+x_width / 2.; p.y = y+y_width / 2.; p.z = z+z_width / 2.;
			verts.push_back(p);

			// vertice 7
			p.x = x+x_width / 2.; p.y = y-y_width / 2.; p.z = z+z_width / 2.;
			verts.push_back(p);
			
			int vert_idx[12][2] = {
				{0,1},
				{1,2},
				{2,3},
				{0,3},
				{0,4},
				{1,5},
				{3,7},
				{2,6},
				{4,5},
				{5,6},
				{4,7},
				{6,7}
			};
			
			for (size_t i=0;i<12;i++){
				line.points.push_back(verts[vert_idx[i][0]]);
				line.points.push_back(verts[vert_idx[i][1]]);
			}
			
			lines.markers.push_back(line);
			
			line.id++;
		}
		// publish
		publisher.publish(lines);
}

int BoxDetector::resize_uniform(cv::Mat& src, cv::Mat& dst, cv::Size dst_size, object_rect& effect_area)
{
    int w = src.cols;
    int h = src.rows;
    int dst_w = dst_size.width;
    int dst_h = dst_size.height;
    //std::cout << "src: (" << h << ", " << w << ")" << std::endl;
    dst = cv::Mat(cv::Size(dst_w, dst_h), CV_8UC3, cv::Scalar(0));

    float ratio_src = w * 1.0 / h;
    float ratio_dst = dst_w * 1.0 / dst_h;

    int tmp_w = 0;
    int tmp_h = 0;
    if (ratio_src > ratio_dst) {
        tmp_w = dst_w;
        tmp_h = floor((dst_w * 1.0 / w) * h);
    }
    else if (ratio_src < ratio_dst) {
        tmp_h = dst_h;
        tmp_w = floor((dst_h * 1.0 / h) * w);
    }
    else {
        cv::resize(src, dst, dst_size);
        effect_area.x = 0;
        effect_area.y = 0;
        effect_area.width = dst_w;
        effect_area.height = dst_h;
        return 0;
    }
    cv::Mat tmp;
    cv::resize(src, tmp, cv::Size(tmp_w, tmp_h));

    if (tmp_w != dst_w) {
        int index_w = floor((dst_w - tmp_w) / 2.0);
        //std::cout << "index_w: " << index_w << std::endl;
        for (int i = 0; i < dst_h; i++) {
            memcpy(dst.data + i * dst_w * 3 + index_w * 3, tmp.data + i * tmp_w * 3, tmp_w * 3);
        }
        effect_area.x = index_w;
        effect_area.y = 0;
        effect_area.width = tmp_w;
        effect_area.height = tmp_h;
    }
    else if (tmp_h != dst_h) {
        int index_h = floor((dst_h - tmp_h) / 2.0);
        //std::cout << "index_h: " << index_h << std::endl;
        memcpy(dst.data + index_h * dst_w * 3, tmp.data, tmp_w * tmp_h * 3);
        effect_area.x = 0;
        effect_area.y = index_h;
        effect_area.width = tmp_w;
        effect_area.height = tmp_h;
    }
    else {
        printf("error\n");
    }
    return 0;
}

void BoxDetector::draw_bboxes(const cv::Mat& bgr, const std::vector<BoxInfo>& bboxes, object_rect effect_roi)
{
    static const char* class_names[] = { "person", "bicycle", "car", "motorcycle", "airplane", "bus",
                                        "train", "truck", "boat", "traffic light", "fire hydrant",
                                        "stop sign", "parking meter", "bench", "bird", "cat", "dog",
                                        "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe",
                                        "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
                                        "skis", "snowboard", "sports ball", "kite", "baseball bat",
                                        "baseball glove", "skateboard", "surfboard", "tennis racket",
                                        "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl",
                                        "banana", "apple", "sandwich", "orange", "broccoli", "carrot",
                                        "hot dog", "pizza", "donut", "cake", "chair", "couch",
                                        "potted plant", "bed", "dining table", "toilet", "tv", "laptop",
                                        "mouse", "remote", "keyboard", "cell phone", "microwave", "oven",
                                        "toaster", "sink", "refrigerator", "book", "clock", "vase",
                                        "scissors", "teddy bear", "hair drier", "toothbrush"
    };

    cv::Mat image = bgr.clone();
    int src_w = image.cols;
    int src_h = image.rows;
    int dst_w = effect_roi.width;
    int dst_h = effect_roi.height;
    float width_ratio = (float)src_w / (float)dst_w;
    float height_ratio = (float)src_h / (float)dst_h;


    for (size_t i = 0; i < bboxes.size(); i++)
    {
        const BoxInfo& bbox = bboxes[i];
        cv::Scalar color = cv::Scalar(this->color_list[bbox.label][0], this->color_list[bbox.label][1], this->color_list[bbox.label][2]);
        //fprintf(stderr, "%d = %.5f at %.2f %.2f %.2f %.2f\n", bbox.label, bbox.score,
        //    bbox.x1, bbox.y1, bbox.x2, bbox.y2);
        
        double x1 = (bbox.x1 - effect_roi.x) * width_ratio;
        double y1 = (bbox.y1 - effect_roi.y) * height_ratio;
        double x2 = (bbox.x2 - effect_roi.x) * width_ratio;
        double y2 = (bbox.y2 - effect_roi.y) * height_ratio;
        if((bbox.x1 - effect_roi.x) * width_ratio<0)
        {
          x1 =0;
        }
        if((bbox.y1 - effect_roi.y) * height_ratio<0)
        {
          y1=0;
        }
        if((bbox.x2 - effect_roi.x) * width_ratio>image.cols)
        {
          x2=image.cols;
        }
        if((bbox.y2 - effect_roi.y) * height_ratio>image.rows)
        {
         y2=image.rows;
        }

        cv::rectangle(image, cv::Rect(cv::Point(x1, y1),
                                      cv::Point(x2, y2)), color);

        char text[256];
        sprintf(text, "%s %.1f%%", class_names[bbox.label], bbox.score * 100);

        int baseLine = 0;
        cv::Size label_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.4, 1, &baseLine);

        int x = (bbox.x1 - effect_roi.x) * width_ratio;
        int y = (bbox.y1 - effect_roi.y) * height_ratio - label_size.height - baseLine;
        if (y < 0)
            y = 0;
        if (x + label_size.width > image.cols)
            x = image.cols - label_size.width;

        cv::rectangle(image, cv::Rect(cv::Point(x, y), cv::Size(label_size.width, label_size.height + baseLine)),
            color, -1);

        cv::putText(image, text, cv::Point(x, y + label_size.height),
            cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255));
    }


    // static int count=0;
    // std::string filename = "/home/jzp/result_image/0__5_1_5/image_" + std::to_string(count) + ".jpg";
    // bool result = cv::imwrite(filename, image);
    // count++;

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    boximage_pub.publish(msg);

    // cv::imshow("image", image);

    // cv::waitKey(10);
}

void BoxDetector::updatebox(){
  if (this->depthImage_.empty()||this->rgbImage_.empty()){
	return;
  }
  ros::Time mode_time = ros::Time::now();
  auto start = std::chrono::high_resolution_clock::now();
  // Do some work here

 
  this->clusters.clear();
  cv::Mat image = this->rgbImage_.clone();
  object_rect effect_roi;
  cv::Mat resized_img;
  resize_uniform(image, resized_img, cv::Size(this->width, this->height), effect_roi);
  auto results = detector_->detect(resized_img, 0.4, 0.5);

  // draw_bboxes(image, results, effect_roi);

  cv::Mat temp_depth;
  temp_depth = this->depthImage_.clone();
  int src_w = image.cols;
  int src_h = image.rows;
  int dst_w = effect_roi.width;
  int dst_h = effect_roi.height;
  float width_ratio = (float)src_w / (float)dst_w;
  float height_ratio = (float)src_h / (float)dst_h;

  for (int i=0;i < results.size();i++)
  {
    BoxInfo bbox = results[i];
    if(bbox.label != 0)
    {
      continue;
    }
    double x1 = (bbox.x1 - effect_roi.x) * width_ratio;
    double y1 = (bbox.y1 - effect_roi.y) * height_ratio;
    double x2 = (bbox.x2 - effect_roi.x) * width_ratio;
    double y2 = (bbox.y2 - effect_roi.y) * height_ratio;
    if((bbox.x1 - effect_roi.x) * width_ratio<0)
    {
      x1 =0;
    }
    if((bbox.y1 - effect_roi.y) * height_ratio<0)
    {
      y1=0;
    }
    if((bbox.x2 - effect_roi.x) * width_ratio>temp_depth.cols)
    {
      x2=temp_depth.cols;
    }
    if((bbox.y2 - effect_roi.y) * height_ratio>temp_depth.rows)
    {
      y2=temp_depth.rows;
    }
    cv::Rect roi = cv::Rect(cv::Point(x1, y1),cv::Point(x2, y2)); ///rect
    
    cv::Mat img_roi = temp_depth(roi); // 获取ROI
    img_roi.setTo(0); // 设置ROI内的像素值为0   这一段会改变img图像吗

  }
  // std::cout<<"2"<<std::endl;

  // sensor_msgs::ImagePtr msg = cv_bridge::CvImage(this->header, sensor_msgs::image_encodings::TYPE_32FC1, temp_depth).toImageMsg();
  // depthImage_pub_.publish(msg);
  
  for (int i=0;i < results.size();i++)
  {
    // std::cout<<"have"<<std::endl;
    BoxInfo bbox = results[i];
    if(bbox.label != 0)
    {
      continue;
    }
    double x1 = (bbox.x1 - effect_roi.x) * width_ratio;
    double y1 = (bbox.y1 - effect_roi.y) * height_ratio;
    double x2 = (bbox.x2 - effect_roi.x) * width_ratio;
    double y2 = (bbox.y2 - effect_roi.y) * height_ratio;
    if((bbox.x1 - effect_roi.x) * width_ratio<0)
    {
      x1 =0;
    }
    if((bbox.y1 - effect_roi.y) * height_ratio<0)
    {
      y1=0;
    }
    if((bbox.x2 - effect_roi.x) * width_ratio>temp_depth.cols)
    {
      x2=temp_depth.cols;
    }
    if((bbox.y2 - effect_roi.y) * height_ratio>temp_depth.rows)
    {
      y2=temp_depth.rows;
    }
    cv::Rect rect = cv::Rect(cv::Point(x1, y1),cv::Point(x2, y2)); ///rect

    cv::Mat image_box =this->depthImage_(rect);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr( new pcl::PointCloud<pcl::PointXYZ> () );
    int Radius=5;
    int center_y = int(image_box.rows/ 2);
    int center_x = int(image_box.cols / 2);
    // std::cout<<"center_y:"<<center_x<<std::endl;
    // std::cout<<"center_y:"<<center_y<<std::endl;
    double distance=0;
    double distance_the=0.20;  // 需要根据深度图的单位变化
    int count=0;
    for ( int y = 0; y < image_box.rows;  y=y+2 ) {
        for ( int x = 0; x < image_box.cols; x=x+2 ) {
            pcl::PointXYZ pt,pt_tem;
            if ( image_box.at<float>(y, x) != 0 )
            {
                pt.z = image_box.at<float>(y, x);//danwei  m
                // std::cout<<"pt.z:"<<pt.z<<std::endl;
                pt.x = (x+rect.x-this->cx_)*pt.z/this->fx_;
                pt.y = (y+rect.y-this->cy_)*pt.z/this->fy_;  //这个值很重要的

                // pt_tem.x = pt.z;
                // pt_tem.y = -pt.x;
                // pt_tem.z = -pt.y;
                // std::cout<<"point:"<<"add"<<std::endl;

                if (abs(y-center_y)<=Radius&&abs(x-center_x)<=Radius)
                {
                    count++;
                    // std::cout<<"count:"<<"add"<<std::endl;
                    distance=distance+pt.z;  // 32FC1 是以米为单位的
                }
                cloud_ptr->points.push_back( pt );
            }
            else
            {
                pt.z = 0;
                pt.x = 0;
                pt.y = 0;
                // cloud_ptr->points.push_back( pt );
            }
        }
    }
    // std::cout<<"distance:"<<distance<<std::endl;
    distance = distance/count;

    // std::cout<<"cloud_ptr->points:"<<cloud_ptr->points.size()<<std::endl;
    // std::cout<<"count:"<<count<<std::endl;
    // std::cout<<"distance:"<<distance<<std::endl;


    double min_x = std::numeric_limits<double>::max();
    double max_x = -std::numeric_limits<double>::max();
    double min_y = std::numeric_limits<double>::max();
    double max_y = -std::numeric_limits<double>::max();
    double min_z = std::numeric_limits<double>::max();
    double max_z = -std::numeric_limits<double>::max();

    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_z = 0.0;
    
    int count_num =0 ;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_people( new pcl::PointCloud<pcl::PointXYZ> () );
    for(auto pointcb: cloud_ptr->points)
    {   
      // std::cout<<pointcb.z<<std::endl;
        if(abs(pointcb.z-distance)<distance_the)
        {   
            count_num = count_num +1; 
            pcl::PointXYZ pt_tem;
            Eigen::Vector3d tem(pointcb.x,pointcb.y,pointcb.z);
            tem = this->orientation_*tem + this->position_;
            pt_tem.x = tem (0);
            pt_tem.y = tem (1);
            pt_tem.z = tem (2);
            cloud_people->points.push_back(pt_tem);
            if(pointcb.x <= min_x)
            {
                min_x=pointcb.x;
            }
            if(pointcb.y <= min_y)
            {
                min_y=pointcb.y;
            }
            if(pointcb.z <= min_z)
            {
                min_z=pointcb.z;
            }
            if(pointcb.x >= max_x)
            {
                max_x=pointcb.x;
            }
            if(pointcb.y >=max_y)
            {
                max_y=pointcb.y;
            }
            if(pointcb.z >= max_z)
            {
                max_z=pointcb.z;
            }   
            sum_x+=  pointcb.x;
            sum_y+=  pointcb.y;
            sum_z+=  pointcb.z;        

        }
    }
    if(count_num ==0)
    {
      continue;
    }
    // std::cout<<"have_cout"<<std::endl;
    // sensor_msgs::PointCloud2 input_cloud;
    // pcl::toROSMsg(*cloud_people,input_cloud);
    // input_cloud.header.frame_id = "world";
    // input_cloud.header.stamp = ros::Time::now();
    // people_pub.publish(input_cloud);
    // std::cout<<"count_num "<<count_num <<std::endl;
    // std::cout<<"sum_x"<<sum_x<<" "<<sum_y<<" "<<sum_z<<std::endl;


    // double mean_x = sum_x / (cloud_people->points).size();
    // double mean_y = sum_y / (cloud_people->points).size();
    // double mean_z = sum_z / (cloud_people->points).size();
    double mean_x = (max_x+min_x)/2;   //sum_x / (cloud_people->points).size();
    double mean_y = (max_y+min_y)/2;                   //sum_y / (cloud_people->points).size();
    double mean_z = (max_z+min_z)/2;                   //sum_z / (cloud_people->points).size();
    Eigen::Vector3d vec (mean_x, mean_y, mean_z);

    // geometry_msgs::PoseStamped pose_msg;
    // // pose_msg.header.stamp=this->header.stamp; // 设置时间戳为当前时间
    // pose_msg.header.stamp=ros::Time::now(); // 设置时间戳为当前时间
    // // pose_msg.header.stamp=mode_time;
    // pose_msg.header.frame_id = "world";

    // pose_msg.pose.position.x = mean_z;
    // pose_msg.pose.position.y = -mean_x;
    // pose_msg.pose.position.z = -mean_y;
    // pose_msg.pose.orientation.x = 0.0;
    // pose_msg.pose.orientation.y = 0.0;
    // pose_msg.pose.orientation.z = 0.0;
    // pose_msg.pose.orientation.w = 1.0;

    // // 发布消息
    // pose_pub.publish(pose_msg);
    // std::cout<<"publish"<<std::endl;



    if(double(vec.norm())>=7)
    {
      continue;
    }
    // std::cout<<mean_x<<" "<<mean_y<<" "<<mean_z<<std::endl;
    if(mean_z<6&&mean_z>0.5)
    {  // 这个距离可能需要优化一下
        cluster tmp;
        // tmp.centroid = this->orientation_*Eigen::Vector3d(mean_x, mean_y, mean_z)+this->position_;
        tmp.centroid = this->orientation_*vec+this->position_;
        Eigen::MatrixXd tol(3,3);

        // tmp.bbox = Eigen::Vector3d(2 * (std::max(fabs(max_z - mean_z), fabs(min_z - mean_z))),
        //                           2 * (std::max(fabs(max_x - mean_x), fabs(min_x - mean_x))),
        //                        2 * (std::max(fabs(max_y - mean_y), fabs(min_y - mean_y))));

        tmp.bbox = Eigen::Vector3d(fabs(max_z-min_z),fabs(max_x-min_x),fabs(max_y-min_y));
        tmp.time = this->time;
        // std::cout<<"afafaf"<<std::endl;
        if(tmp.bbox(0)<1&&tmp.bbox(1)<1&&tmp.bbox(2)<2.4)
        {
          this->clusters.push_back(tmp); // 障碍物的处理都完成了
          this->clusters_temp = clusters;
        }
    }
  }
  // std::cout<<"clusters.size()<"<<clusters.size()<<std::endl;
  // std::cout<<"all_tracks_.size()"<<all_tracks_.size()<<std::endl;
  // std::cout<<"3"<<std::endl;
  // init trajectory
  // std::cout<<"----------------"<<std::endl;
  // std::cout<<"position:"<<clusters[0].centroid.transpose()<<std::endl;
  // std::cout<<"time:"<<this->time<<std::endl;


  if(all_tracks_.size() == 0 && this->clusters.size() != 0)
  {
    /*add new track*/
    for(int i = 0;i< this->clusters.size();i++)
    {
       addNewTrack(this->clusters[i]);
      //  std::cout<<"4"<<std::endl;
    }   
    // std::cout<<"4"<<std::endl;
  }
  else if(all_tracks_.size() == 0 && this->clusters.size() == 0)
  { 
    // std::cout<<"5"<<std::endl;
  }
  else if(all_tracks_.size() != 0 && this->clusters.size() == 0)
  {
    //  updatetrack(all_tracks_);
    //  std::cout<<"6"<<std::endl;
  }
  else{

    std::vector<cluster> tem_clusters;
    std::vector<cluster> tem_for_addclusters;
    std::vector<track> tem_all_tracks_ = all_tracks_; 
    for(int i=0;i<this->clusters.size();i++)
    { 
      bool add_new_track = true;
      cluster a = clusters[i];
      for (int j=0;j<all_tracks_.size();j++)
      {  
        track b = all_tracks_[j];
        if(distance(a.centroid(0),a.centroid(1),b.getLatestposition()(0),b.getLatestposition()(1)) < 2.0 ) //和轨迹的匹配)
        { 
          add_new_track = false;
        }
      }
      if(add_new_track)
      {
        tem_for_addclusters.push_back(a);
      }
      else{
        tem_clusters.push_back(a);
      } 
    }
    this->clusters = tem_clusters;
    for(int i=0;i<tem_for_addclusters.size();i++)
    {
      addNewTrack(tem_for_addclusters[i]);
      // std::cout<<"hh"<<std::endl;
    }
    if(this->clusters.size()==0)
    {
      return;
    }
    // std::cout<<"clusters.size()22<"<<clusters.size()<<std::endl;
    // std::cout<<"tem_all_tracks_<"<<tem_all_tracks_.size()<<std::endl;
    //////////////////////////
    // Run Hungarian Algorithm
    //////////////////////////

    std::vector<std::vector<double>> costMatrix;
    for (int i=0;i<this->clusters.size();i++)
    {
      auto cluster_i = clusters[i];  // for each of the rows
      std::vector<double> costs_cluster_i;
      for (auto& track_j : all_tracks_)  // for each of the columns
      {
        costs_cluster_i.push_back(getCostRowColum(cluster_i, track_j));
      }
      // std::cout<<"c"<<std::endl;
      costMatrix.push_back(costs_cluster_i);  // Add row to the matrix
    }
    // for(int i=0;i<costMatrix.size();i++)
    // {
    //   for (int j=0;j<costMatrix[0].size();j++)
    //   std::cout<<"costMatrix"<< costMatrix[i][j]<<std::endl;
    // }
    
    // std::cout<<"7"<<std::endl;
    // Run the Hungarian Algorithm;
    HungarianAlgorithm HungAlgo;
    std::vector<int> track_assigned_to_cluster;
    // std::cout<<"g"<<std::endl;
    // std::cout << "Calling Hungarian Algorithm now!!" << std::endl;
    double cost = HungAlgo.Solve(costMatrix, track_assigned_to_cluster);  // 和未来预测轨迹之间的匹配。
    // std::cout<<"f"<<std::endl;
    for(int i=0;i<track_assigned_to_cluster.size();i++)
    {
      // std::cout<<" afad"<<track_assigned_to_cluster[i]<<std::endl;
    }
    // std::cout << "Called  Hungarian Algorithm!" << std::endl;
    for (int i = 0; i < costMatrix.size(); i++)  // for each of the rows
    {
      // std::cout<<"o"<<std::endl;
      // If a cluster has been unassigned (can happen if rows>columns), then create a new track for it
      if (track_assigned_to_cluster[i] == -1)
      {
        addNewTrack(clusters[i]);   // 没有匹配上添加新的轨迹。   // 没有匹配上，添加新轨迹
        // std::cout<<"h"<<std::endl;
      }
      else
      { 
        if (all_tracks_[track_assigned_to_cluster[i]].is_new == true)
        {
          // std::cout<<"oppssssp"<<std::endl;
          all_tracks_[track_assigned_to_cluster[i]].is_new = false;  // 连续两次匹配
          // std::cout<<"oppadada"<<std::endl;
          all_tracks_[track_assigned_to_cluster[i]].addToHistory(clusters[i]);
          // std::cout<<"oppp"<<std::endl;

        }
        else
        {
          // std::cout<<"oppafafafafaa"<<std::endl;
          all_tracks_[track_assigned_to_cluster[i]].addToHistory(clusters[i]);
          // std::cout<<"jjjjj"<<std::endl;
        }
        // std::cout<<"k"<<std::endl;
      }
    }
  }
  // 好了，到这里已经完成了轨迹的重要性值。
  // std::cout<<"updatetracka"<<std::endl;
  updatetrack(all_tracks_);  // 新轨迹还是老轨迹
  // std::cout<<"updatetrackb"<<std::endl;
  // auto end = std::chrono::high_resolution_clock::now();
  // auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
  // std::cout << "Elapsed time: " << elapsed_ms.count() << " ms" << std::endl;
  
}

void BoxDetector::updatetrack(std::vector<track> &all_tracks_)
{
    /*没有检测到了，每一个轨迹都需要刷新了*/
    double time =ros::Time::now().toSec();
    std::vector<track> temp_tracks_;  // 现在的所有的轨迹
    for (int i=0;i<all_tracks_.size();i++)
    { 
      if(!all_tracks_[i].shouldDestory(time))
      {
        if(all_tracks_[i].have_add == false&&all_tracks_[i].init_fit == true)
        {
          all_tracks_[i].addhandhistory(time);
        }
        all_tracks_[i].have_add = false;
        temp_tracks_.push_back(all_tracks_[i]);
      }
    }
    all_tracks_ = temp_tracks_;
}

double BoxDetector::getCostRowColum(cluster& a, track& b)
{ 
  // std::cout<<a.centroid(0)<<" "<<a.centroid(1)<<" "<<b.getLatestposition()(0)<<" "<<b.getLatestposition()(1);
  double result = distance(a.centroid(0),a.centroid(1),b.getLatestposition()(0),b.getLatestposition()(1)); //和轨迹的匹配
  // std::cout << "getCostRowColum= " << result << std::endl;
  return result;
}

void BoxDetector::addNewTrack( cluster& c)
{
  // std::cout<<"add track"<<std::endl;
  track tmp(c, 4);
  all_tracks_.push_back(tmp);  // add track
}

inline void BoxDetector::getCameraPose(const nav_msgs::OdometryConstPtr& odom, Eigen::Matrix4d& camPoseMatrix){
	Eigen::Quaterniond quat;
	quat = Eigen::Quaterniond(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);
	Eigen::Matrix3d rot = quat.toRotationMatrix();

	// convert body pose to camera pose
	Eigen::Matrix4d map2body; map2body.setZero();
	map2body.block<3, 3>(0, 0) = rot;
	map2body(0, 3) = odom->pose.pose.position.x; 
	map2body(1, 3) = odom->pose.pose.position.y;
	map2body(2, 3) = odom->pose.pose.position.z;
	map2body(3, 3) = 1.0;

	camPoseMatrix = map2body * this->body2Cam_;
}
inline void BoxDetector::getCameraPose(const geometry_msgs::PoseStampedConstPtr& pose, Eigen::Matrix4d& camPoseMatrix){
	Eigen::Quaterniond quat;
	quat = Eigen::Quaterniond(pose->pose.orientation.w, pose->pose.orientation.x, pose->pose.orientation.y, pose->pose.orientation.z);
	Eigen::Matrix3d rot = quat.toRotationMatrix();

	// convert body pose to camera pose
	Eigen::Matrix4d map2body; map2body.setZero();
	map2body.block<3, 3>(0, 0) = rot;
	map2body(0, 3) = pose->pose.position.x; 
	map2body(1, 3) = pose->pose.position.y;
	map2body(2, 3) = pose->pose.position.z;
	map2body(3, 3) = 1.0;

	camPoseMatrix = map2body * this->body2Cam_;
}

void BoxDetector::imagePoseCB( const cv::Mat& img,const cv::Mat& img1, const geometry_msgs::PoseStampedConstPtr& pose){
	// store current depth image   
  img.copyTo(this->depthImage_);
  if (img.type() == CV_16UC1){
		this->depthImage_.convertTo(this->depthImage_, CV_32FC1, (1/this->depthScale_));
	}
  
  cv::Mat color_image;
  // 检查图像类型
  if (img1.channels() == 1) {
    // 如果是灰度图像，则转换为彩色图
    cv::cvtColor(img1, color_image, cv::COLOR_GRAY2BGR);
  } else 
  {
    color_image = img1.clone();
  }

	color_image.copyTo(this->rgbImage_); 

	// store current position and orientation (camera)
	Eigen::Matrix4d camPoseMatrix;
	this->getCameraPose(pose, camPoseMatrix);

	this->position_(0) = camPoseMatrix(0, 3);
	this->position_(1) = camPoseMatrix(1, 3);
	this->position_(2) = camPoseMatrix(2, 3);
	this->orientation_ = camPoseMatrix.block<3, 3>(0, 0);

	updatebox();

  this->header = pose->header;
  // this->time = header.stamp.toSec();
  // std::cout<<"time"<<time<<std::endl;
  this->time = ros::Time::now().toSec();
}

void BoxDetector::imageOdomCB(const cv::Mat& img,const cv::Mat& img1, const nav_msgs::OdometryConstPtr& odom){
	// store current depth image
  img.copyTo(this->depthImage_);
  if (img.type() == CV_16UC1){
		this->depthImage_.convertTo(this->depthImage_, CV_32FC1, (1/this->depthScale_));
	}
  
  cv::Mat color_image;
  // 检查图像类型
  if (img1.channels() == 1) {
    // 如果是灰度图像，则转换为彩色图
    cv::cvtColor(img1, color_image, cv::COLOR_GRAY2BGR);
  } else 
  {
    color_image = img1.clone();
  }

	color_image.copyTo(this->rgbImage_); 

	// store current position and orientation (camera)
	Eigen::Matrix4d camPoseMatrix;
	this->getCameraPose(odom, camPoseMatrix);

	this->position_(0) = camPoseMatrix(0, 3);
	this->position_(1) = camPoseMatrix(1, 3);
	this->position_(2) = camPoseMatrix(2, 3);
	this->orientation_ = camPoseMatrix.block<3, 3>(0, 0);
  // std::cout<<"position:"<<this->position_(0)<<" "<<this->position_(1)<<" "<<this->position_(2)<<std::endl;
  
  // std::cout<<"detector1"<<std::endl;
  // cv::imshow("1",this->rgbImage_);
  // cv::waitKey(0);
	updatebox();
  // std::cout<<"detector2"<<std::endl;
  
  this->header = odom->header;
  // this->time = header.stamp.toSec();
  this->time = ros::Time::now().toSec();
}

// void BoxDetector::imagePoseCB(const sensor_msgs::ImageConstPtr& img,const sensor_msgs::ImageConstPtr& img1, const geometry_msgs::PoseStampedConstPtr& pose){
// 	// store current depth image
// 	cv_bridge::CvImagePtr imgPtr = cv_bridge::toCvCopy(img, img->encoding);
// 	if (img->encoding == sensor_msgs::image_encodings::TYPE_16UC1){
// 		(imgPtr->image).convertTo(imgPtr->image, CV_32FC1, (1/this->depthScale_));
// 	}
//   imgPtr->image.copyTo(this->depthImage_);
// 	cv_bridge::CvImagePtr imgPtr1 = cv_bridge::toCvCopy(img1, img1->encoding);
// 	imgPtr1->image.copyTo(this->rgbImage_);    

// 	// store current position and orientation (camera)
// 	Eigen::Matrix4d camPoseMatrix;
// 	this->getCameraPose(pose, camPoseMatrix);

// 	this->position_(0) = camPoseMatrix(0, 3);
// 	this->position_(1) = camPoseMatrix(1, 3);
// 	this->position_(2) = camPoseMatrix(2, 3);
// 	this->orientation_ = camPoseMatrix.block<3, 3>(0, 0);

// 	updatebox();
//   this->header = img1->header;
//   this->time = header.stamp.toSec();
//   // std::cout<<"time"<<time<<std::endl;
// }

// void BoxDetector::imageOdomCB(const sensor_msgs::ImageConstPtr& img,const sensor_msgs::ImageConstPtr& img1, const nav_msgs::OdometryConstPtr& odom){
// 	// store current depth image
// 	cv_bridge::CvImagePtr imgPtr = cv_bridge::toCvCopy(img, img->encoding);
// 	if (img->encoding == sensor_msgs::image_encodings::TYPE_16UC1){
// 		(imgPtr->image).convertTo(imgPtr->image, CV_32FC1, (1/this->depthScale_));
// 	}
//   // std::cout<<img->encoding<<std::endl;
//   imgPtr->image.copyTo(this->depthImage_);
//       // for ( int y = 0; y < 4; ++ y ) {
//       //   for ( int x = 0; x < 4; ++ x ) {
//       //     std::cout<<this->depthImage_.at<float>(y, x) <<std::endl; 
//       //   }
//       // }
//   cv_bridge::CvImagePtr imgPtr1 = cv_bridge::toCvCopy(img1, img1->encoding);
// 	imgPtr1->image.copyTo(this->rgbImage_); 

// 	// store current position and orientation (camera)
// 	Eigen::Matrix4d camPoseMatrix;
// 	this->getCameraPose(odom, camPoseMatrix);

// 	this->position_(0) = camPoseMatrix(0, 3);
// 	this->position_(1) = camPoseMatrix(1, 3);
// 	this->position_(2) = camPoseMatrix(2, 3);
// 	this->orientation_ = camPoseMatrix.block<3, 3>(0, 0);
//   // std::cout<<"position:"<<this->position_(0)<<" "<<this->position_(1)<<" "<<this->position_(2)<<std::endl;
  
//   // std::cout<<"detector1"<<std::endl;
//   // cv::imshow("1",this->rgbImage_);
//   // cv::waitKey(0);
// 	updatebox();
//   // std::cout<<"detector2"<<std::endl;
  
//   this->header = img1->header;
//   this->time = header.stamp.toSec();
// }

int BoxDetector::getnumtrack()
{
  return this->tem_All_tracks_.size();
}


// ConvexHullsOfCurve_Std BoxDetector::getIntervalHull(double t_init,double ts)
// {   
//     ConvexHullsOfCurve hull_curve;
//     for (int i =0;i<this->tem_All_tracks_.size();i++)
//     {
//       int segment = 3;
//       std::vector<Eigen::Vector3d> positions;
//       auto start_time1 = std::chrono::high_resolution_clock::now();
//       for(double time =t_init;time<t_init+ts;time =time +double(ts/segment))
//       {
//         Eigen::Vector3d position = tem_All_tracks_[i].getOneTimePosition(time);
//         positions.push_back(position);
//       }
//       auto end_time1 = std::chrono::high_resolution_clock::now();
//       auto duration1 = std::chrono::duration_cast<std::chrono::microseconds>(end_time1 - start_time1);
//       std::cout << "执行时间gettrackpoint: " << duration1.count() << " wei秒" << std::endl;

//       Eigen::Vector3d max_box = tem_All_tracks_[i].getMaxBbox();
//       std::vector<Point_3> points;
//       auto start_time = std::chrono::high_resolution_clock::now();
//       for(int j=0 ;j<positions.size();j++)
//       {
//         Eigen::Vector3d pt;
//         pt = positions[j];
//         double x = pt.x();
//         double y = pt.y();
//         double z = pt.z();
//         points.push_back(Point_3(x - max_box.x()/ 2.0, y - max_box.y()/ 2.0, z - max_box.z()/ 2.0));
//         points.push_back(Point_3(x - max_box.x()/ 2.0, y - max_box.y()/ 2.0, z + max_box.z()/ 2.0));
//         points.push_back(Point_3(x - max_box.x()/ 2.0, y + max_box.y()/ 2.0, z + max_box.z()/ 2.0));
//         points.push_back(Point_3(x - max_box.x()/ 2.0, y + max_box.y()/ 2.0, z - max_box.z()/ 2.0));

//         points.push_back(Point_3(x + max_box.x()/ 2.0, y + max_box.y()/ 2.0, z + max_box.z()/ 2.0));
//         points.push_back(Point_3(x + max_box.x()/ 2.0, y + max_box.y()/ 2.0, z - max_box.z()/ 2.0));
//         points.push_back(Point_3(x + max_box.x()/ 2.0, y - max_box.y()/ 2.0, z + max_box.z()/ 2.0));
//         points.push_back(Point_3(x + max_box.x()/ 2.0, y - max_box.y()/ 2.0, z - max_box.z()/ 2.0));
//       }
//       auto end_time = std::chrono::high_resolution_clock::now();
//       auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
//       std::cout << "执行时间getpoint: " << duration.count() << " wei秒" << std::endl;

//       start_time = std::chrono::high_resolution_clock::now();
//       CGAL_Polyhedron_3 hull_interval = convexHullOfPoints(points);
//       hull_curve.push_back(hull_interval);
//       end_time = std::chrono::high_resolution_clock::now();
//       duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
//       std::cout << "执行时间convexHull: " << duration.count() << " wei秒" << std::endl;     
//     }
//     return singlevectorGCALPol2vectorStdEigen(hull_curve);  // 每一个障碍物这一段时间内的点
// }


// ConvexHullsOfCurves_Std BoxDetector::getAllIntervalnHull(double t_init,double ts,int num_segment)
// {   
//     ConvexHullsOfCurves hulls_curve;
//     ConvexHullsOfCurve hull_curve;

//     for (int i =0;i<this->tem_All_tracks_.size();i++)
//     {
//       Eigen::Vector3d max_box = tem_All_tracks_[i].getMaxBbox();

//       for(int j=0;j<num_segment;j++)
//       {

//         int segment = 3;
//         std::vector<Eigen::Vector3d> positions;
//         for(double time =t_init+j*ts;time<t_init+ts+j*ts;time =time +double(ts/segment))
//         {
//             Eigen::Vector3d position = tem_All_tracks_[i].getOneTimePosition(time);
//             positions.push_back(position);
//         }

//         std::vector<Point_3> points;
//         for(int k=0 ;k<positions.size();k++)
//         {
//           Eigen::Vector3d pt;
//           pt = positions[k];
//           double x = pt.x();
//           double y = pt.y();
//           double z = pt.z();
//           points.push_back(Point_3(x - max_box.x()/ 2.0, y - max_box.y()/ 2.0, z - max_box.z()/ 2.0));
//           points.push_back(Point_3(x - max_box.x()/ 2.0, y - max_box.y()/ 2.0, z + max_box.z()/ 2.0));
//           points.push_back(Point_3(x - max_box.x()/ 2.0, y + max_box.y()/ 2.0, z + max_box.z()/ 2.0));
//           points.push_back(Point_3(x - max_box.x()/ 2.0, y + max_box.y()/ 2.0, z - max_box.z()/ 2.0));

//           points.push_back(Point_3(x + max_box.x()/ 2.0, y + max_box.y()/ 2.0, z + max_box.z()/ 2.0));
//           points.push_back(Point_3(x + max_box.x()/ 2.0, y + max_box.y()/ 2.0, z - max_box.z()/ 2.0));
//           points.push_back(Point_3(x + max_box.x()/ 2.0, y - max_box.y()/ 2.0, z + max_box.z()/ 2.0));
//           points.push_back(Point_3(x + max_box.x()/ 2.0, y - max_box.y()/ 2.0, z - max_box.z()/ 2.0));
//         }
//         CGAL_Polyhedron_3 hull_interval = convexHullOfPoints(points);
//         hull_curve.push_back(hull_interval);
//       }
//       hulls_curve.push_back(hull_curve);
//     }
//     return  vectorGCALPol2vectorStdEigen(hulls_curve);  // 每一个障碍物这一段时间内的点
// }

//pcl::PointXYZ pt_tem;
// ConvexHullsOfCurve_Std BoxDetector::getIntervalHull(double t_init,double ts)
// {   
//     ConvexHullsOfCurve_Std hulls_curve;
//     for (int i =0;i<this->tem_All_tracks_.size();i++)
//     {
//       int segment = 3;
//       std::vector<Eigen::Vector3d> positions;
//       auto start_time1 = std::chrono::high_resolution_clock::now();
//       for(double time =t_init;time<t_init+ts;time =time +double(ts/segment))
//       {
//         Eigen::Vector3d position = tem_All_tracks_[i].getOneTimePosition(time);
//         positions.push_back(position);
//       }
//       auto end_time1 = std::chrono::high_resolution_clock::now();
//       auto duration1 = std::chrono::duration_cast<std::chrono::microseconds>(end_time1 - start_time1);
//       std::cout << "执行时间gettrackpoint: " << duration1.count() << " wei秒" << std::endl;

//       Eigen::Vector3d max_box = tem_All_tracks_[i].getMaxBbox();
//       // std::vector<Point_3> points;
//       pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//       // pcl::PointXYZ pt;
//       // cloud_ptr->points.push_back( pt );
//       auto start_time = std::chrono::high_resolution_clock::now();
//       for(int j=0 ;j<positions.size();j++)
//       {
//         Eigen::Vector3d pt;
//         pt = positions[j];
//         double x = pt.x();
//         double y = pt.y();
//         double z = pt.z();
//         cloud->points.push_back(pcl::PointXYZ(x - max_box.x()/ 2.0, y - max_box.y()/ 2.0, z - max_box.z()/ 2.0));
//         cloud->points.push_back(pcl::PointXYZ(x - max_box.x()/ 2.0, y - max_box.y()/ 2.0, z + max_box.z()/ 2.0));
//         cloud->points.push_back(pcl::PointXYZ(x - max_box.x()/ 2.0, y + max_box.y()/ 2.0, z + max_box.z()/ 2.0));
//         cloud->points.push_back(pcl::PointXYZ(x - max_box.x()/ 2.0, y + max_box.y()/ 2.0, z - max_box.z()/ 2.0));

//         cloud->points.push_back(pcl::PointXYZ(x + max_box.x()/ 2.0, y + max_box.y()/ 2.0, z + max_box.z()/ 2.0));
//         cloud->points.push_back(pcl::PointXYZ(x + max_box.x()/ 2.0, y + max_box.y()/ 2.0, z - max_box.z()/ 2.0));
//         cloud->points.push_back(pcl::PointXYZ(x + max_box.x()/ 2.0, y - max_box.y()/ 2.0, z + max_box.z()/ 2.0));
//         cloud->points.push_back(pcl::PointXYZ(x + max_box.x()/ 2.0, y - max_box.y()/ 2.0, z - max_box.z()/ 2.0));
//       }
//       auto end_time = std::chrono::high_resolution_clock::now();
//       auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
//       std::cout << "执行时间getpoint: " << duration.count() << " wei秒" << std::endl;

//       start_time = std::chrono::high_resolution_clock::now();
//       pcl::ConvexHull<pcl::PointXYZ> hull;                  
//       hull.setInputCloud(cloud);                   
//       hull.setDimension(3);  // 设置凸包维度
//       hull.setComputeAreaVolume(false);
//       std::vector<pcl::Vertices> polygons;    
//       // polygons保存的是所有凸包多边形的顶点在surface_hull中的下标
//       pcl::PointCloud<pcl::PointXYZ>::Ptr surface_hull(new pcl::PointCloud<pcl::PointXYZ>);
//       // surface_hull是所有凸包多边形的顶点
//       hull.reconstruct(*surface_hull, polygons);
//       std::cout<<"surface_hull"<<surface_hull->size()<<std::endl;
//       // 遍历点云中的点
//       Polyhedron_Std hull_curve(3,surface_hull->size());
//       int count_pt =0;
//       for (pcl::PointCloud<pcl::PointXYZ>::iterator it = surface_hull->begin(); it != surface_hull->end(); ++it) {
//          // 访问当前点的坐标
//          double x = it->x;
//          double y = it->y;
//          double z = it->z;
//          hull_curve.col(count_pt) = Eigen::Vector3d(x,y,z);
//          count_pt ++; 
//       }

//       hulls_curve.push_back(hull_curve);

//       end_time = std::chrono::high_resolution_clock::now();
//       duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
//       std::cout << "执行时间convexHull: " << duration.count() << " wei秒" << std::endl;     
//     }
//     return hulls_curve;  // 每一个障碍物这一段时间内的点
// }

// ConvexHullsOfCurves_Std BoxDetector::getAllIntervalnHull(double t_init,double ts,int num_segment)
// {   
//     ConvexHullsOfCurves hulls_curve;
//     ConvexHullsOfCurve hull_curve;

//     for (int i =0;i<this->tem_All_tracks_.size();i++)
//     {
//       Eigen::Vector3d max_box = tem_All_tracks_[i].getMaxBbox();

//       for(int j=0;j<num_segment;j++)
//       {

//         int segment = 3;
//         std::vector<Eigen::Vector3d> positions;
//         for(double time =t_init+j*ts;time<t_init+ts+j*ts;time =time +double(ts/segment))
//         {
//             Eigen::Vector3d position = tem_All_tracks_[i].getOneTimePosition(time);
//             positions.push_back(position);
//         }

//         std::vector<Point_3> points;
//         for(int k=0 ;k<positions.size();k++)
//         {
//           Eigen::Vector3d pt;
//           pt = positions[k];
//           double x = pt.x();
//           double y = pt.y();
//           double z = pt.z();
//           points.push_back(Point_3(x - max_box.x()/ 2.0, y - max_box.y()/ 2.0, z - max_box.z()/ 2.0));
//           points.push_back(Point_3(x - max_box.x()/ 2.0, y - max_box.y()/ 2.0, z + max_box.z()/ 2.0));
//           points.push_back(Point_3(x - max_box.x()/ 2.0, y + max_box.y()/ 2.0, z + max_box.z()/ 2.0));
//           points.push_back(Point_3(x - max_box.x()/ 2.0, y + max_box.y()/ 2.0, z - max_box.z()/ 2.0));

//           points.push_back(Point_3(x + max_box.x()/ 2.0, y + max_box.y()/ 2.0, z + max_box.z()/ 2.0));
//           points.push_back(Point_3(x + max_box.x()/ 2.0, y + max_box.y()/ 2.0, z - max_box.z()/ 2.0));
//           points.push_back(Point_3(x + max_box.x()/ 2.0, y - max_box.y()/ 2.0, z + max_box.z()/ 2.0));
//           points.push_back(Point_3(x + max_box.x()/ 2.0, y - max_box.y()/ 2.0, z - max_box.z()/ 2.0));
//         }
//         CGAL_Polyhedron_3 hull_interval = convexHullOfPoints(points);
//         hull_curve.push_back(hull_interval);
//       }
//       hulls_curve.push_back(hull_curve);
//     }
//     return  vectorGCALPol2vectorStdEigen(hulls_curve);  // 每一个障碍物这一段时间内的点
// }

ConvexHullsOfCurve_Std BoxDetector::getIntervalHull(int mode,double t_init,double ts)
{   
    ConvexHullsOfCurve_Std hulls_curve;
    std::vector<track> temp_track_hull;
    if(mode ==0)
    {
      temp_track_hull = this->tem_All_tracks_;
    }
    else
    {
      temp_track_hull = this->all_tracks_;
    }

    for (int i =0;i<temp_track_hull.size();i++)
    { 
      // std::cout<<"evalzz"<<std::endl;
      if(temp_track_hull[i].init_fit == false)//||temp_track_hull[i].isdynammic() == false)
      {
        continue;
      }
      int segment = 3;
      std::vector<Eigen::Vector3d> positions;
      for(double time =t_init;time<t_init+ts;time =time +double(ts/segment))
      {
        Eigen::Vector3d position = temp_track_hull[i].getOneTimePosition(time);
        positions.push_back(position);
      }

      Eigen::Vector3d max_box = temp_track_hull[i].getMaxBbox();
      // std::vector<Point_3> points;
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
      // pcl::PointXYZ pt;
      // cloud_ptr->points.push_back( pt );
      for(int j=0 ;j<positions.size();j++)
      {
        Eigen::Vector3d pt;
        pt = positions[j];
        double x = pt.x();
        double y = pt.y();
        double z = pt.z();
        double distance = 0.25;
        cloud->points.push_back(pcl::PointXYZ(x - max_box.x()/ 2.0-distance, y - max_box.y()/ 2.0-distance, z - max_box.z()/ 2.0-distance));
        cloud->points.push_back(pcl::PointXYZ(x - max_box.x()/ 2.0-distance, y - max_box.y()/ 2.0-distance, z + max_box.z()/ 2.0+distance));
        cloud->points.push_back(pcl::PointXYZ(x - max_box.x()/ 2.0-distance, y + max_box.y()/ 2.0+distance, z + max_box.z()/ 2.0+distance));
        cloud->points.push_back(pcl::PointXYZ(x - max_box.x()/ 2.0-distance, y + max_box.y()/ 2.0+distance, z - max_box.z()/ 2.0-distance));

        cloud->points.push_back(pcl::PointXYZ(x + max_box.x()/ 2.0+distance, y + max_box.y()/ 2.0+distance, z + max_box.z()/ 2.0+distance));
        cloud->points.push_back(pcl::PointXYZ(x + max_box.x()/ 2.0+distance, y + max_box.y()/ 2.0+distance, z - max_box.z()/ 2.0-distance));
        cloud->points.push_back(pcl::PointXYZ(x + max_box.x()/ 2.0+distance, y - max_box.y()/ 2.0-distance, z + max_box.z()/ 2.0+distance));
        cloud->points.push_back(pcl::PointXYZ(x + max_box.x()/ 2.0+distance, y - max_box.y()/ 2.0-distance, z - max_box.z()/ 2.0-distance));
      }

      pcl::ConvexHull<pcl::PointXYZ> hull;                  
      hull.setInputCloud(cloud);                   
      hull.setDimension(3);  // 设置凸包维度
      hull.setComputeAreaVolume(false);
      std::vector<pcl::Vertices> polygons;    
      // polygons保存的是所有凸包多边形的顶点在surface_hull中的下标
      pcl::PointCloud<pcl::PointXYZ>::Ptr surface_hull(new pcl::PointCloud<pcl::PointXYZ>);
      // surface_hull是所有凸包多边形的顶点
      hull.reconstruct(*surface_hull, polygons);

      // 遍历点云中的点
      Polyhedron_Std hull_curve(3,surface_hull->size());
      int count_pt =0;
      for (pcl::PointCloud<pcl::PointXYZ>::iterator it = surface_hull->begin(); it != surface_hull->end(); ++it) {
         // 访问当前点的坐标
         double x = it->x;
         double y = it->y;
         double z = it->z;
         hull_curve.col(count_pt) = Eigen::Vector3d(x,y,z);
         count_pt ++; 
      }

      hulls_curve.push_back(hull_curve);    
    }
    return hulls_curve;  // 每一个障碍物这一段时间内的点
}

//pcl::PointXYZ pt_tem;
ConvexHullsOfCurve_Std BoxDetector::getIntervalHull(int segment)// 0-29
{   
    ConvexHullsOfCurve_Std hulls_curve;
    if(segment>=30)
    {
      hulls_curve = getIntervalHull(0,this->time_st_+this->ts_*segment,this->ts_);
    }
    else
    {
      for(int i =0;i < this->hulls_curves_.size();i++)
      {
        Polyhedron_Std hull_curve = this->hulls_curves_[i][segment];
        hulls_curve.push_back(hull_curve);
      }
    }
    
    return hulls_curve;  // 每一个障碍物这一段时间内的点
}

ConvexHullsOfCurves_Std BoxDetector::getAllIntervalnHull(double t_init,double ts,int num_segment)
{   
    ConvexHullsOfCurves_Std hulls_curves;
    
    for (int i =0;i<this->tem_All_tracks_.size();i++)
    {
      Eigen::Vector3d max_box = tem_All_tracks_[i].getMaxBbox();
      ConvexHullsOfCurve_Std hulls_curve;
      for(int j=0;j<num_segment;j++)
      {

        int segment = 3;
        std::vector<Eigen::Vector3d> positions;
        for(double time =t_init+j*ts;time<t_init+ts+j*ts;time =time +double(ts/segment))
        {
            Eigen::Vector3d position = tem_All_tracks_[i].getOneTimePosition(time);
            positions.push_back(position);
        }
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        // std::vector<Point_3> points;
        for(int k=0 ;k<positions.size();k++)
        {
          Eigen::Vector3d pt;
          pt = positions[k];
          double x = pt.x();
          double y = pt.y();
          double z = pt.z();

          double distance = 0.25;
          cloud->points.push_back(pcl::PointXYZ(x - max_box.x()/ 2.0-distance, y - max_box.y()/ 2.0-distance, z - max_box.z()/ 2.0-distance));
          cloud->points.push_back(pcl::PointXYZ(x - max_box.x()/ 2.0-distance, y - max_box.y()/ 2.0-distance, z + max_box.z()/ 2.0+distance));
          cloud->points.push_back(pcl::PointXYZ(x - max_box.x()/ 2.0-distance, y + max_box.y()/ 2.0+distance, z + max_box.z()/ 2.0+distance));
          cloud->points.push_back(pcl::PointXYZ(x - max_box.x()/ 2.0-distance, y + max_box.y()/ 2.0+distance, z - max_box.z()/ 2.0-distance));

          cloud->points.push_back(pcl::PointXYZ(x + max_box.x()/ 2.0+distance, y + max_box.y()/ 2.0+distance, z + max_box.z()/ 2.0+distance));
          cloud->points.push_back(pcl::PointXYZ(x + max_box.x()/ 2.0+distance, y + max_box.y()/ 2.0+distance, z - max_box.z()/ 2.0-distance));
          cloud->points.push_back(pcl::PointXYZ(x + max_box.x()/ 2.0+distance, y - max_box.y()/ 2.0-distance, z + max_box.z()/ 2.0+distance));
          cloud->points.push_back(pcl::PointXYZ(x + max_box.x()/ 2.0+distance, y - max_box.y()/ 2.0-distance, z - max_box.z()/ 2.0-distance));
          // cloud->points.push_back(pcl::PointXYZ(x - max_box.x()/ 2.0, y - max_box.y()/ 2.0, z - max_box.z()/ 2.0));
          // cloud->points.push_back(pcl::PointXYZ(x - max_box.x()/ 2.0, y - max_box.y()/ 2.0, z + max_box.z()/ 2.0));
          // cloud->points.push_back(pcl::PointXYZ(x - max_box.x()/ 2.0, y + max_box.y()/ 2.0, z + max_box.z()/ 2.0));
          // cloud->points.push_back(pcl::PointXYZ(x - max_box.x()/ 2.0, y + max_box.y()/ 2.0, z - max_box.z()/ 2.0));

          // cloud->points.push_back(pcl::PointXYZ(x + max_box.x()/ 2.0, y + max_box.y()/ 2.0, z + max_box.z()/ 2.0));
          // cloud->points.push_back(pcl::PointXYZ(x + max_box.x()/ 2.0, y + max_box.y()/ 2.0, z - max_box.z()/ 2.0));
          // cloud->points.push_back(pcl::PointXYZ(x + max_box.x()/ 2.0, y - max_box.y()/ 2.0, z + max_box.z()/ 2.0));
          // cloud->points.push_back(pcl::PointXYZ(x + max_box.x()/ 2.0, y - max_box.y()/ 2.0, z - max_box.z()/ 2.0));


        }
        
        pcl::ConvexHull<pcl::PointXYZ> hull; 
        hull.setInputCloud(cloud);                   
        hull.setDimension(3);  // 设置凸包维度
        hull.setComputeAreaVolume(false);
        std::vector<pcl::Vertices> polygons;    
        // polygons保存的是所有凸包多边形的顶点在surface_hull中的下标
        pcl::PointCloud<pcl::PointXYZ>::Ptr surface_hull(new pcl::PointCloud<pcl::PointXYZ>);
        // surface_hull是所有凸包多边形的顶点
        hull.reconstruct(*surface_hull, polygons);
        // 遍历点云中的点
        Polyhedron_Std hull_curve(3,surface_hull->size());
         int count_pt =0;
        for (pcl::PointCloud<pcl::PointXYZ>::iterator it = surface_hull->begin(); it != surface_hull->end(); ++it) {
         // 访问当前点的坐标
         double x = it->x;
         double y = it->y;
         double z = it->z;
         hull_curve.col(count_pt) = Eigen::Vector3d(x,y,z);
         count_pt ++; 
        }
        hulls_curve.push_back(hull_curve);
      }
      hulls_curves.push_back(hulls_curve);
    }
    return  hulls_curves;  // 每一个障碍物这一段时间内的点
}

std::vector<std::vector<double>> BoxDetector::getboxes()
{
  std::vector<std::vector<double>> result;
  for(int i=0;i<this->clusters_temp.size();i++)
  {
    cluster b = clusters_temp[i];
    std::vector<double> a = {b.centroid(0),b.centroid(1),b.centroid(2),b.bbox(0),b.bbox(1),b.bbox(2)};
    result.push_back(a);
  }
  return result;
}

void BoxDetector::StoreAllTrack(double time_st,double ts,std::vector<std::vector<Eigen::Vector3d>> &positionss)
{
  tem_All_tracks_.clear();
  std::vector<track> tem_All_tracks_use = this->all_tracks_;  // 现在的所有的轨迹
  for(int i=0;i<tem_All_tracks_use.size();i++)
  { 
    // std::cout<<"evalxx"<<std::endl;
    if(tem_All_tracks_use[i].init_fit == true)//&&tem_All_tracks_use[i].isdynammic() == true)
    {
       this->tem_All_tracks_.push_back(tem_All_tracks_use[i]);
    }
  }
  std::cout<<"sstem_All_tracks_:"<<tem_All_tracks_.size()<<std::endl;

  ConvexHullsOfCurves_Std hulls_curves;
  hulls_curves = this->getAllIntervalnHull(time_st,ts,30);
  this->time_st_ = time_st;
  this->ts_ = ts_;
  this->hulls_curves_ = hulls_curves;


  for(int i =0;i<tem_All_tracks_.size();i++)
  {
    std::vector<Eigen::Vector3d> positions;
    for(int k=0;k<60;k++)
    {
      Eigen::Vector3d position = tem_All_tracks_[i].getOneTimePosition(time_st+((double)k+0.5)*ts);
      positions.push_back(position);
    }
    positionss.push_back(positions);
  }

}




ConvexHullsOfCurves_Std BoxDetector::gettemphull(int q_segment)
{
  ConvexHullsOfCurves_Std hulls_curves_return;
  if(q_segment>=30)
  {
    ConvexHullsOfCurves_Std hulls_curves;
    hulls_curves = this->getAllIntervalnHull(this->time_st_,this->ts_,q_segment);
    hulls_curves_return = hulls_curves;
  }
  else
  {
    for(int i = 0;i<this->hulls_curves_.size();i++)
    {
      ConvexHullsOfCurve_Std hulls_curve;
      for(int j=0;j<q_segment;j++)
      {
        hulls_curve.push_back(this->hulls_curves_[i][j]);
      }
      hulls_curves_return.push_back(hulls_curve);
    }
  }
  return hulls_curves_return;
}

bool BoxDetector::checkReplan(Eigen::Vector3d pose)
{
  // std::cout<<"checkReplan"<<std::endl;
  for(int i=0;i<this->all_tracks_.size();i++)
  {
    Eigen::Vector3d track_pose = this->all_tracks_[i].getLatestposition();
    if((pose - track_pose).norm()<=0.8)
    {
      return true;
    }
  }
  return false;

}

bool BoxDetector::checkReplan1(Eigen::Matrix<double, 4, 3>  Q)
{
  // std::cout<<"checkReplan"<<std::endl;
  Eigen::Matrix<double,4,4> trans;
  trans<<1,4,1,0,
         -3,0,3,0,
        3,-6,3,0,
        -1,3,-3,1;
  trans = trans/6;

  for(double time =0;time<=1;time+=0.2)
  {
    Eigen::VectorXd t_pow(4);
    t_pow<<1,pow(time,1),pow(time,2),pow(time,3);
    Eigen::Vector3d pose = t_pow.transpose()*trans*Q;
    for(int i=0;i<this->all_tracks_.size();i++)
    {
      Eigen::Vector3d track_pose = this->all_tracks_[i].getLatestposition();
      if((pose - track_pose).norm()<=0.5)
      {
        return true;
      }
    }
  }
  return false;

}



int BoxDetector::getallnumtrack()
{
  return all_tracks_.size();
}




