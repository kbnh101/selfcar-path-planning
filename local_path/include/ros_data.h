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
#include<geometry_msgs/Polygon.h>

#include<visualization_msgs/MarkerArray.h>

#include<ackermann_msgs/AckermannDriveStamped.h>
#include<ackermann_msgs/AckermannDrive.h>

#include<sensor_msgs/PointCloud.h>

#include<morai_msgs/CtrlCmd.h>
#include<data_transfer_msg/data_transfer.h>

class ros_data
{
public:
    ros_data();

    int speed;
    double vehicle_yaw, vehicle_roll, vehicle_pitch;
    double UTM_OFFSET_X, UTM_OFFSET_Y;
    bool flag, parking_state;

    std::string state;

    nav_msgs::Path path;
    nav_msgs::Path temp_path;
    nav_msgs::Path trajectory_path;
    nav_msgs::Path tracking_path;
    nav_msgs::Path parking_path;
    nav_msgs::Odometry odometry;
    nav_msgs::OccupancyGrid small_map;
    nav_msgs::OccupancyGrid global_map;

    geometry_msgs::PoseWithCovariance pose;
    geometry_msgs::PoseStamped temp_pose;
    geometry_msgs::Point prev_point;
    geometry_msgs::Twist cmd_vel;
    geometry_msgs::Polygon node_list;
    geometry_msgs::Polygon parking_stop_list;

    sensor_msgs::PointCloud object_point;

    data_transfer_msg::data_transfer camera_data;

    //Publish
    ros::Publisher cmd_pub;
    ros::Publisher cmd_pub_;
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
    ros::Subscriber sim_pose_sub;
    ros::Subscriber object_sub;
    ros::Subscriber camera_sub;
    ros::Subscriber node_sub;
    ros::Subscriber parking_path_sub;
    ros::Subscriber parking_stop_sub;

    //Callback
    void pathcallback(const nav_msgs::Path& msg);
    void trajectorycallback(const nav_msgs::Path& msg);
    void posecallback(const geometry_msgs::PoseWithCovariance& msg);
    void speed_callback(const std_msgs::Int32 &msg);
    void map_callback(const nav_msgs::OccupancyGrid &msg);
    void state_callback(const std_msgs::String &msg);
    void sim_pose_callback(const nav_msgs::Odometry &msg);
    void object_callback(const sensor_msgs::PointCloud &msg);
    void camera_callback(const data_transfer_msg::data_transfer &msg);
    void node_callback(const geometry_msgs::Polygon &msg);
    void parking_path_callback(const nav_msgs::Path &msg);
    void parking_stop_callback(const geometry_msgs::Polygon &msg);
};

#endif // ROS_DATA_H
