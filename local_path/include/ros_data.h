#ifndef ROS_DATA_H
#define ROS_DATA_H
#include<ros/ros.h>
#include<iostream>
#include<math.h>
#include<cmath>
#include<vector>
#include<queue>
#include<tf/tf.h>

#include<std_msgs/String.h>
#include<std_msgs/Int32.h>

#include<nav_msgs/Path.h>
#include<nav_msgs/Odometry.h>
#include<nav_msgs/OccupancyGrid.h>

#include<geometry_msgs/Point.h>
#include<geometry_msgs/PointStamped.h>
#include<geometry_msgs/PoseWithCovarianceStamped.h>

#include<visualization_msgs/MarkerArray.h>

#include<ackermann_msgs/AckermannDriveStamped.h>
#include<ackermann_msgs/AckermannDrive.h>

#include<morai_msgs/CtrlCmd.h>

class ros_data
{
public:
    ros_data();

    int speed;
    double vehicle_yaw, vehicle_roll, vehicle_pitch;
    double UTM_OFFSET_X, UTM_OFFSET_Y;
    bool flag;

    std::string state;

    nav_msgs::Path path;
    nav_msgs::Path temp_path;
    nav_msgs::Path trajectory_path;
    nav_msgs::Path tracking_path;
    nav_msgs::Odometry odometry;
    nav_msgs::OccupancyGrid small_map;
    nav_msgs::OccupancyGrid global_map;

    geometry_msgs::PoseWithCovariance pose;
    geometry_msgs::PoseStamped temp_pose;
    geometry_msgs::Point prev_point;
    geometry_msgs::Twist cmd_vel;


    //Publish
    ros::Publisher cmd_pub;
    ros::Publisher path_pub;
    ros::Publisher pose_pub;
    ros::Publisher marker_vel;
    ros::Publisher small_map_pub;
    //Subscriber
    ros::Subscriber pose_sub;
    ros::Subscriber path_sub;
    ros::Subscriber trajectory;
    ros::Subscriber Odom;
    ros::Subscriber speed_sub;
    ros::Subscriber map_sub;
    ros::Subscriber state_sub;

    //Callback
    void pathcallback(const nav_msgs::Path& msg);
    void trajectorycallback(const nav_msgs::Path& msg);
    void posecallback(const geometry_msgs::PoseWithCovariance& msg);
    void speed_callback(const std_msgs::Int32 &msg);
    void map_callback(const nav_msgs::OccupancyGrid &msg);
    void state_callback(const std_msgs::String &msg);
};

#endif // ROS_DATA_H
