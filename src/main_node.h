#ifndef MAIN_NODE_H
#define MAIN_NODE_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <iostream>
#include <vector>
#include <sensor_msgs/LaserScan.h>
#include <tf/tf.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl-1.7/pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Float32MultiArray.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#define degreesToRadians(degrees) (degrees * M_PI / 180.0)

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class LaserCallbackWrapper {
public:
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    bool laser_callback_is_active;
    bool check_for_obstacles();
    bool obstacles_in_dist;
    float min_distance;
    float max_distance;
    float dist;
private:

};

class OdomCallbackWrapper {
public:
    void odomCallback(const nav_msgs::OdometryConstPtr& msg);
    geometry_msgs::Pose2D current_pose;
    bool new_odom_data_avail;
};

class PointCloudCallbackWrapper {
public:
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
    float red_object_distance;
    bool running;
};

bool driveOdom(double distance, ros::NodeHandle n, int forward);
bool move_one(int argc, char **argv);
bool pixel_is_red(int r, int g, int b);
float min_laser_dist(ros::NodeHandle n);
void print_options_list();
bool check_for_obstacles(ros::NodeHandle n, float dist);
float distance_to_red_object(ros::NodeHandle n);
void turn_around_alpha(float alpha, ros::NodeHandle n);
void publish_move_command(ros::Publisher move_pub, float x, float z);
void wait_for_odom_data(OdomCallbackWrapper& odomCb);
float move_speed_factor(float x);
float turn_speed_factor(float passed, float radians);
void get_mat_from_pcl(cv::Mat& image, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
void pixelTo3DPoint(const sensor_msgs::PointCloud2 pCloud, const int u, const int v, geometry_msgs::Point &p);
float turnOdom(int clockwise, double radians, ros::NodeHandle n, bool searching_red_object);
float getDistance(float x, float y, float z);

#endif	// MAIN_NODE_H
