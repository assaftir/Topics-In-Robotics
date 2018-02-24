#include <pcl-1.7/pcl/point_cloud.h>
#include "main_node.h"

char* cmdvel_topic = "/mobile_base_controller/cmd_vel";
char* odom_topic = "/mobile_base_controller/odom";

//GLOBAL
float pCenterX, pCenterY;
vector<vector<Point> > contours;
Mat src; Mat src_gray;
int thresh = 100;
int max_thresh = 255;
RNG rng(12345);

int main(int argc, char **argv) {
    ros::init(argc, argv, "main_node");
    ros::NodeHandle n;
    int option = 0;
    while (n.ok()) {
        cout << ("Please choose an option. '0' for available commands, '-1' to exit") << endl << ">>";
        cin >> option;
        switch (option) {
            case -1:
                cout << "bye bye" << endl;
                exit(-1);
	    case 1:
                float button_dist;
		button_dist = 0.0;
                cout << "Driving to the elevator: " << endl;
                if (move_to_elevator(argc, argv)){
                    while((button_dist = distance_to_red_object(n)) <= 0){
                        ROS_INFO("Searching for the button");
                    }
                    cout << "The button is " << button_dist << " meters away" << endl;
		    if(come_back_home())
		    	cout << "I'm at Home!" << endl;
                }else{
                    cout << "Can't drive to the elevator for some reason" << endl;
                }
                break;
	    case 2:
                if(dist_and_home(n))
		    cout << "YAY!" << endl;
		else
		    cout << "Crap!" << endl;
                break;
            case 3:
                cout << "Min distance: " << min_laser_dist(n) << endl;
                break;          
            default:
                cout << "Wrong input, try again" << endl;
        }
    }
}

bool dist_and_home(ros::NodeHandle n){
    float button_dist;
    button_dist = 0.0;
    while((button_dist = distance_to_red_object(n)) <= 0 || isnan(button_dist))
    	ROS_INFO("Searching for the button");

    cout << "The button is " << button_dist << " meters away" << endl;
    if(come_back_home()){
        cout << "I'm at Home!" << endl;
        return true;
    }
    return false;
}

float distance_to_red_object(ros::NodeHandle n) {
    PointCloudCallbackWrapper pclCb;
    // "/kinect2/qhd/points" for armdillo
    // "/torso_camera/depth_registered/points" for komodo
    ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2> ("/torso_camera/depth_registered/points", 10, &PointCloudCallbackWrapper::cloudCallback, &pclCb);
    pclCb.running = true;
    //Will be set to false by the PCL callback when it's done
    while (n.ok() && pclCb.running)
        ros::spinOnce();
    return pclCb.red_object_distance;
}

float min_laser_dist(ros::NodeHandle n){
    ros::Rate r(10);
    LaserCallbackWrapper laserCb;
    ros::Subscriber scan_subscriber = n.subscribe<sensor_msgs::LaserScan>("/scan", 10, &LaserCallbackWrapper::laserCallback, &laserCb);
    //Will be changed to false from outside, when the laser callback has finished executing
    laserCb.laser_callback_is_active = true;
    while (laserCb.laser_callback_is_active) {
        ros::spinOnce();
        r.sleep();
    }
    return laserCb.min_distance;
}

//Take the mid range and check if it's > this->dist
void LaserCallbackWrapper::laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
    this->laser_callback_is_active = true;
    this->obstacles_in_dist = false;
    this->min_distance = scan->ranges[0];
    for(int i = 0 ; i < scan->ranges.size() ; i++){
	if(scan->ranges[i] < this->min_distance)
		this->min_distance = scan->ranges[i];
        if (scan->ranges[i] < this->dist){
            this->obstacles_in_dist = true;
            break;
        }
    }
    this->laser_callback_is_active = false;
}

//Do some coordinates transformation and store {x,y, theta} in current_pose object

void OdomCallbackWrapper::odomCallback(const nav_msgs::OdometryConstPtr& msg) {
    this->current_pose.x = msg->pose.pose.position.x;
    this->current_pose.y = msg->pose.pose.position.y;
    //Quaternion to RPY conversion
    tf::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
    tf::Matrix3x3 mat(q);
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);
    this->current_pose.theta = yaw;
    this->new_odom_data_avail = true;
}

void PointCloudCallbackWrapper::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
try{
    //Convert between sensor_msgs/PointCloud2 to PointXYZRGB vector
    if ((msg->width * msg->height) == 0) {
        cout << "Failed: cloud not dense" <<endl;
        return; //return if the cloud is not dense!
    }
    sensor_msgs::Image image_;
    try {
        pcl::toROSMsg(*msg, image_); //convert the cloud
    }    catch (std::runtime_error e) {
        ROS_ERROR_STREAM("Error in converting cloud to image message: " << e.what());
    }
    cv::Mat bgr_image = cv_bridge::toCvCopy(image_, "bgr8")->image;
    //Without the median blur, we have some false positives
    cv::medianBlur(bgr_image, bgr_image, 3);
    cv::Mat original_image = bgr_image.clone();
    //Convert the bgr image to HSV encoding
    cv::Mat hsv_image;
    cv::cvtColor(bgr_image, hsv_image, cv::COLOR_BGR2HSV);
    //Filter the image, take only the desired red hue ranges
    cv::Mat lower_range;
    cv::Mat upper_range;
    cv::inRange(hsv_image, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), lower_range);
    cv::inRange(hsv_image, cv::Scalar(160, 100, 100), cv::Scalar(180, 255, 255), upper_range);
    //Combine both red ranges to one image and blur it
    cv::Mat gray_image = lower_range + upper_range;
    cv::GaussianBlur(gray_image, gray_image, cv::Size(9, 9), 2, 2);
    src_gray = gray_image;
    //
    cout << "ok up to here" << endl;
    cv::namedWindow("Src:", cv::WINDOW_AUTOSIZE);
    cv::imshow("Src:", gray_image);
    cout << "ok up to here2" << endl;
    createTrackbar(" Canny thresh:", "Source", &thresh, max_thresh, thresh_callback);
    cout << "ok up to here3" << endl;
    thresh_callback(0, 0);
    cout << "ok up to here4" << endl;
    //Now contours array is filled up with contours
    //cout << contours.size() << endl;
    geometry_msgs::Point pCenter;
    pixelTo3DPoint(*msg, pCenterX, pCenterY, pCenter);
    cout << "ok up to here5" << endl;
    cout << "dist to elevator: " << pCenter.z << endl;
    cout << "ok up to here6" << endl;
    cv::waitKey(60);

    //
    this->red_object_distance = pCenter.z;
    this->running = false;
}
catch(exception& e){
    this->red_object_distance = 0;
    this->running = false;
}
}

void get_mat_from_pcl(cv::Mat& image, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){
    for( int y = 0; y < image.rows; y++ ) {
        for( int x = 0; x < image.cols; x++ ) {        
            pcl::PointXYZRGB point = cloud->at( x, y );
            image.at<cv::Vec3f>( y, x )[0] = point.b;
            image.at<cv::Vec3f>( y, x )[1] = point.g;
            image.at<cv::Vec3f>( y, x )[2] = point.r;
        }
    }
}

void print_options_list() {
    cout << "(1) Move forward moves the robot forward 50cm if there is no obstacle that is closer than 50 cm. Otherwise, the robot does not move." << endl;
    cout << "(2) Prompts for an angle alpha and turns around the robot in place alpha degrees clockwise. " << endl;
    cout << "(3) Distance to red object return the distance to a red object in the robot's current frame. If there are no red objects is return NULL." << endl;
    cout << "(4) Find red object turns around while searching for a red object. It stops after it finds a red object and returns its distance. If there is no red object, it stops after doing a full circle. " << endl;
}

//Let the odom callback run

void wait_for_odom_data(OdomCallbackWrapper& odomCb) {
    ros::Rate r(10);
    odomCb.new_odom_data_avail = false;
    while (!odomCb.new_odom_data_avail) {
        ros::spinOnce();
        r.sleep();
    }
}

void pixelTo3DPoint(const sensor_msgs::PointCloud2 pCloud, const int u, const int v, geometry_msgs::Point &p){
      // get width and height of 2D point cloud data
      int width = pCloud.width;
      int height = pCloud.height;

      // Convert from u (column / width), v (row/height) to position in array
      // where X,Y,Z data starts
      int arrayPosition = v*pCloud.row_step + u*pCloud.point_step;

      // compute position in array where x,y,z data start
      int arrayPosX = arrayPosition + pCloud.fields[0].offset; // X has an offset of 0
      int arrayPosY = arrayPosition + pCloud.fields[1].offset; // Y has an offset of 4
      int arrayPosZ = arrayPosition + pCloud.fields[2].offset; // Z has an offset of 8

      float X = 0.0;
      float Y = 0.0;
      float Z = 0.0;

      memcpy(&X, &pCloud.data[arrayPosX], sizeof(float));
      memcpy(&Y, &pCloud.data[arrayPosY], sizeof(float));
      memcpy(&Z, &pCloud.data[arrayPosZ], sizeof(float));

     // put data into the point p
      p.x = X;
      p.y = Y;
      p.z = Z;

    }

//Move using the navigation stack
bool move_to_elevator(int argc, char **argv){
  //Init client
  MoveBaseClient ac("move_base", true);
  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  move_base_msgs::MoveBaseGoal goal;
  //Set the desired (x,y) location in map frame and the (x,y,z,w) quat orientation
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = 12.003;
  goal.target_pose.pose.position.y = -0.826;
  goal.target_pose.pose.orientation.x = 0.0;
  goal.target_pose.pose.orientation.y = 0.0;
  goal.target_pose.pose.orientation.z = 0.541;
  goal.target_pose.pose.orientation.w = 0.841;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();
  //ROS_INFO(ac.getState());
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("I made it!");
  else
    ROS_INFO("Something went wrong :(");

  return ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
}

//Move using the navigation stack
bool come_back_home(){
  //Init client
  MoveBaseClient ac("move_base", true);
  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  move_base_msgs::MoveBaseGoal goal;
  //Set the desired (x,y) location in map frame and the (x,y,z,w) quat orientation
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = -1.174;
  goal.target_pose.pose.position.y = -0.700;
  goal.target_pose.pose.orientation.x = 0.0;
  goal.target_pose.pose.orientation.y = 0.0;
  goal.target_pose.pose.orientation.z = 0.0;
  goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("I made it!");
  else
    ROS_INFO("Something went wrong :(");

  return ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
}
//TODO: Seg fault
void thresh_callback(int, void*){
try{
  Mat canny_output;
  vector<Vec4i> hierarchy;
  int largest_area=0;
  int largest_contour_index=0;
  Rect bounding_rect;  
  /// Detect edges using canny
  Canny( src_gray, canny_output, thresh, thresh*2, 3 );
  /// Find contours
  findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
  /// Draw contours
  Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
  Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
  for( int i = 0; i < contours.size(); i++ ){
       //drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
       double a = contourArea(contours[i], false);  //  Find the area of contour
       if(a > largest_area){
           largest_area = a;
           largest_contour_index = i;                //Store the index of largest contour
           bounding_rect = boundingRect(contours[i]); // Find the bounding rectangle for biggest contour     
	}
  }
  for(int i = 0 ; i < contours[largest_contour_index].size() ; i++){
	cout << contours[largest_contour_index][i] << endl;
  }
  pCenterX = bounding_rect.x + bounding_rect.width/2;
  pCenterY = bounding_rect.y + bounding_rect.height/2;
  cout << "cX: " << pCenterX << " cY:" << pCenterY << endl;
  drawContours(drawing, contours, largest_contour_index, color, CV_FILLED, 8, hierarchy); // Draw the largest contour using previously stored index.
  //rectangle(src, bounding_rect,  Scalar(0,255,0),1, 8,0); 
  /// Show in a window
  namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
  imshow( "Contours", drawing );
  waitKey(60);
}
catch(exception& e){ throw e; }
}
