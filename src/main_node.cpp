#include <pcl-1.7/pcl/point_cloud.h>
#include "main_node.h"

char* cmdvel_topic = "/mobile_base_controller/cmd_vel";
char* odom_topic = "/mobile_base_controller/odom";

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
            case 0:
                print_options_list();
                break;
            case 1:
                cout << "distance?" << endl;
                float dist;
                cin >> dist;
                if (!check_for_obstacles(n, dist)) {
                    cout << "Moving forward " << dist << " meters." << endl;
                    driveOdom(dist, n, 1);
                } else
                    cout << "Obstacles, staying in place." << endl;
                break;
            case 2:
                cout << "degrees?" << endl;
                float alpha_;
                cin >> alpha_;
                int clockwise;
                cout << "type 1 for clockwise, 0 for anti-clockwise" << endl;
                cin >> clockwise;
                cout << "Turning " << alpha_ << " degrees" << endl;
                turnOdom(clockwise, degreesToRadians(alpha_), n, false);                              
                break;
            case 3:
                cout << "Measuring distance" << endl;
                cout << "Distance: " << distance_to_red_object(n) << endl;
                break;
            case 4:
                cout << "Looking for the red object" << endl;
                cout << "Distance: " << turnOdom(1, 2 * M_PI, n, true) << endl;
                break;
	    case 5:
                cout << "Moving with actionlib: " << move_one(argc, argv) << endl;
                break;
            case 6:
                cout << "Min distance: " << min_laser_dist(n) << endl;
                break;
            case 7:
                cout << "degrees?" << endl;
                float alpha;
                cin >> alpha;
                cout << "Turning " << alpha << " degrees clockwise" << endl;
                turn_around_alpha(alpha, n);
                break;
            case 8:
                cout << "distance?" << endl;
                float dist_;
                cin >> dist_;
                cout << "Moving backwards " << dist_ << " meters." << endl;
                driveOdom(dist_, n, -1);
                break;
            default:
                cout << "Wrong input, try again" << endl;
        }
    }
}

bool driveOdom(double distance, ros::NodeHandle n, int forward) {
    //! We will be publishing to the "cmd_vel" topic to issue commands
    ros::Publisher cmd_vel_pub_ = n.advertise<geometry_msgs::Twist>(cmdvel_topic, 10);
    //! We will be listening to TF transforms as well
    tf::TransformListener listener_;
    //wait for the listener to get the first message
    listener_.waitForTransform("base_footprint", "odom", ros::Time(0), ros::Duration(1.0));
    //we will record transforms here
    tf::StampedTransform start_transform;
    tf::StampedTransform current_transform;
    //record the starting transform from the odometry to the base frame
    listener_.lookupTransform("base_footprint", "odom", ros::Time(0), start_transform);
    //we will be sending commands of type "twist"
    geometry_msgs::Twist move_cmd;
    //the command will be to go forward at 0.25 m/s
    double passed = 0;
    move_cmd.linear.y = move_cmd.angular.z = 0;
    ros::Rate rate(10.0);
    bool done = false;
    while (!done && n.ok()) {
        cout << "Passed: " << passed << endl;
        //Update speed according to the correct distance from goal
        move_cmd.linear.x = forward * 0.25 * move_speed_factor(abs(distance) - passed);
        //send the drive command
        cmd_vel_pub_.publish(move_cmd);
        rate.sleep();
        //get the current transform
        try {
            listener_.lookupTransform("base_footprint", "odom", ros::Time(0), current_transform);
        } catch (tf::TransformException ex) {
            ROS_ERROR("%s", ex.what());
            break;
        }
        //see how far we've traveled
        tf::Transform relative_transform = start_transform.inverse() * current_transform;
        passed = relative_transform.getOrigin().length();
        if (passed > abs(distance)){
	    //Stop
	    move_cmd.linear.x = 0;
	    cmd_vel_pub_.publish(move_cmd);
	    done = true;
	}
    }
    cout << "done" << endl;
    return done;
}

float turnOdom(int clockwise, double radians, ros::NodeHandle n, bool searching_red_object) {  
    if (searching_red_object){
    	float dist;
    	if ((dist = distance_to_red_object(n)) != 0)
    	    return dist;
    }
    while (radians < 0) radians += 2 * M_PI;
    while (radians > 2 * M_PI) radians -= 2 * M_PI;
    //! We will be publishing to the "cmd_vel" topic to issue commands
    ros::Publisher cmd_vel_pub_ = n.advertise<geometry_msgs::Twist>(cmdvel_topic, 10);
    //! We will be listening to TF transforms as well
    tf::TransformListener listener_;
    //wait for the listener to get the first message
    listener_.waitForTransform("base_footprint", "odom", ros::Time(0), ros::Duration(1.0));
    //we will record transforms here
    tf::StampedTransform start_transform;
    tf::StampedTransform current_transform;
    //record the starting transform from the odometry to the base frame
    listener_.lookupTransform("base_footprint", "odom", ros::Time(0), start_transform);
    //we will be sending commands of type "twist"
    geometry_msgs::Twist base_cmd;
    //the command will be to turn at 0.75 rad/s
    base_cmd.linear.x = base_cmd.linear.y = 0.0;
    //the axis we want to be rotating by
    tf::Vector3 desired_turn_axis(0, 0, 1);
    if (!clockwise) desired_turn_axis = -desired_turn_axis;
    ros::Rate rate(10.0);
    bool done = false;
    double angle_turned = 0;
    while (!done && n.ok()) {
        base_cmd.angular.z = 0.2 * turn_speed_factor(angle_turned, radians);
        if (clockwise) base_cmd.angular.z = -base_cmd.angular.z;
        //send the drive command
        cout << "tunred: " << angle_turned << endl;
        cout << "factor: " << turn_speed_factor(angle_turned, radians) << endl;
        cmd_vel_pub_.publish(base_cmd);
        rate.sleep();
        //get the current transform
        try {
            listener_.lookupTransform("base_footprint", "odom", ros::Time(0), current_transform);
        } catch (tf::TransformException ex) {
            ROS_ERROR("%s", ex.what());
            break;
        }
        tf::Transform relative_transform =
                start_transform.inverse() * current_transform;
        tf::Vector3 actual_turn_axis =
                relative_transform.getRotation().getAxis();
        angle_turned = relative_transform.getRotation().getAngle();
        if (fabs(angle_turned) < 1.0e-2) continue;
        if (actual_turn_axis.dot(desired_turn_axis) < 0)
            angle_turned = 2 * M_PI - angle_turned;
	if (searching_red_object){
    		float dist;
        	if ((dist = distance_to_red_object(n)) != 0) {
            	    publish_move_command(cmd_vel_pub_, 0.0, 0.0);
            	    return dist;
		}
	}
        if (angle_turned > radians) done = true;
    }
    if (searching_red_object)
    	cout << "Couldn't find a red object around" << endl;
    else
	cout << "Done spinning" << endl;
    return 0.0;
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
    //Convert between sensor_msgs/PointCloud2 to PointXYZRGB vector
   // ros::Rate r(10000);
if ((msg->width * msg->height) == 0) {
      cout << "Failed: cloud not dense" <<endl;
      return; //return if the cloud is not dense!
}
    sensor_msgs::Image image_;
    try{
      pcl::toROSMsg (*msg, image_); //convert the cloud
    }
    catch (std::runtime_error e){
      ROS_ERROR_STREAM("Error in converting cloud to image message: " << e.what());
    }
    cv::Mat bgr_image = cv_bridge::toCvCopy(image_, "bgr8")->image;  
	//Without the median blur, we get an unwanted circle
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
	cv::Mat red_image = lower_range + upper_range;
	cv::GaussianBlur(red_image, red_image, cv::Size(9, 9), 2, 2);
	std::vector<cv::Vec3f> circles;
	/*Apply Circle Hough Transformation, returns a vector of circles.
	 Each circle is a vector of the form {x, y, radius}.*/
	 
	 cv::HoughCircles(red_image, circles, CV_HOUGH_GRADIENT, 1, red_image.rows/8, 200, 20, 0, 0);
	 cv::namedWindow("Detected", cv::WINDOW_AUTOSIZE);
    	 cv::imshow("Detected", red_image);
	 cv::waitKey(60);

        bool found_red_object = false;

	if(circles.size() == 0){
	    cout << "I couldn't find the red apple :(" << endl;
	} else {

		std_msgs::Float32MultiArray apple_coordinates;
		for(size_t i = 0 ; i < 3 ; i++){
		    apple_coordinates.data.push_back(round(circles[0][i]));
		}
		geometry_msgs::Point p;
		pixelTo3DPoint(*msg, apple_coordinates.data[0], apple_coordinates.data[1], p);

		//Publish the coordinates : {x, y, radius}
		cout << "coordiantes: " << "x: " << apple_coordinates.data[0] << "y: " << apple_coordinates.data[1] << "radius: " << apple_coordinates.data[2] << endl;
		cout << "distance: " << p.z << endl;

	    

	    if (p.z > 0) {
		this->red_object_distance = p.z;
		found_red_object = true;
	    }
	  
	}
        if (!found_red_object)
		this->red_object_distance = 0;
	this->running = false;
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

bool pixel_is_red(int r, int g, int b){
    return g < 80 && b < 80 && r > 120;
}

/*Check for obstacles within @dist meters,
The answer will be stored in lsaerCb object by the laser callback function*/
bool check_for_obstacles(ros::NodeHandle n, float dist) {
    ros::Rate r(10);
    LaserCallbackWrapper laserCb;
    laserCb.dist = dist;
    ros::Subscriber scan_subscriber = n.subscribe<sensor_msgs::LaserScan>("/scan", 10, &LaserCallbackWrapper::laserCallback, &laserCb);
    //Will be changed to false from outside, when the laser callback has finished executing
    laserCb.laser_callback_is_active = true;
    while (laserCb.laser_callback_is_active) {
        ros::spinOnce();
        r.sleep();
    }
    return laserCb.obstacles_in_dist;
}


float turn_speed_factor(float passed, float radians){
    return ((0.8*(radians - passed)) / radians) + 0.1;
}

void publish_move_command(ros::Publisher move_pub, float x, float z) {
    geometry_msgs::Twist move;
    move.linear.x = x;
    move.angular.z = z;
    move_pub.publish(move);
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

float move_speed_factor(float x){
    if(x > 0.5) return 1;
    return 0.1 + 2.8*x - 2.4*pow(x, 2);
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

//Turn around alpha degrees clockwise, Listen to the odometry topic to reach the desired angle.
//Speed will be reduced as we approach the goal
void turn_around_alpha(float alpha, ros::NodeHandle n) {
    float radians = fmod(degreesToRadians(alpha), M_PI * 2);
    ros::Rate r(10);
    OdomCallbackWrapper odomCb;
    ros::Subscriber odom_sub = n.subscribe(odom_topic, 10, &OdomCallbackWrapper::odomCallback, &odomCb);
    ros::Publisher move_pub = n.advertise<geometry_msgs::Twist>(cmdvel_topic, 10);
    //Wait for the first callback to executed
    wait_for_odom_data(odomCb);
    //Now we have the initial position in odomCb.current_pose (updated by the odom callback)
    geometry_msgs::Pose2D init_pose(odomCb.current_pose);
    //We are going to cross the PI point (where angle goes from -PI to +PI), so split the turn into 2 loops
    //TODO: BETTER SOLUTION
    float remaining_angle = radians;
    if (init_pose.theta - radians <= -M_PI) {
        //Turn up to -PI
        radians = M_PI + init_pose.theta;
        remaining_angle -= radians;
	float passed;
        while ((passed = abs(odomCb.current_pose.theta - init_pose.theta)) < radians) {
	    cout << "current: " << odomCb.current_pose.theta << " init: " << init_pose.theta << endl;
            publish_move_command(move_pub, 0.0, -0.1*turn_speed_factor(passed, radians));
            //spin in order to update current_pose by the odomCallback
            ros::spinOnce();
            r.sleep();
        }
    }
    //Wait for the odom callback to be executed and update the initial position
    wait_for_odom_data(odomCb);
    //Get new init_pose
    init_pose.theta = odomCb.current_pose.theta;
    //Now turn the remaining angle
    float passed;
    while ((passed = abs(odomCb.current_pose.theta - init_pose.theta)) < remaining_angle) {
        cout << "current: " << odomCb.current_pose.theta << " init: " << init_pose.theta << endl;
        publish_move_command(move_pub, 0.0, -0.1*turn_speed_factor(passed, remaining_angle));
        //spin in order to update current_pose by the odomCallback
        ros::spinOnce();
        r.sleep();
    }
    //ROS_INFO("Turned " + alpha + " degrees clockwise\n");
    publish_move_command(move_pub, 0.0, 0.0);
}

//Move using the navigation stack
bool move_one(int argc, char **argv){

  ros::init(argc, argv, "send_goals_node");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("/move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = 1.0;
  goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base moved 1 meter forward");
  else
    ROS_INFO("The base failed to move forward 1 meter for some reason");

  return ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED;

}
