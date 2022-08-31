#ifndef PARKING_PATH_H
#define PARKING_PATH_H

#include<iostream>
#include<string>
#include<ros/ros.h>
#include<tf/tf.h>
#include<vector>
#include<fstream>
#include<istream>
#include<stack>

#include<std_msgs/String.h>

#include<nav_msgs/Path.h>
#include<nav_msgs/Odometry.h>

#include<sensor_msgs/PointCloud.h>

#include<geometry_msgs/PoseStamped.h>
#include<geometry_msgs/PoseWithCovariance.h>
#include<geometry_msgs/Polygon.h>

struct path_data
{
    nav_msgs::Path path;
    int cost;
};

class parking_path
{
public:
    parking_path();

    double offset_x, offset_y;
    double vehicle_pitch, vehicle_roll, vehicle_yaw;

    std::vector<path_data> path_withcost;

    std::string state;

    sensor_msgs::PointCloud object;

    nav_msgs::Path path;
    nav_msgs::Path diagonal_path1;
    nav_msgs::Path diagonal_path2;
    nav_msgs::Path diagonal_path3;
    nav_msgs::Path diagonal_path4;
    nav_msgs::Path diagonal_path5;
    nav_msgs::Path diagonal_path6;
    nav_msgs::Path parallel_path1;
    nav_msgs::Path parallel_path2;
    nav_msgs::Path temp_node;

    geometry_msgs::PoseWithCovariance pose;
    geometry_msgs::Polygon stop_area;

    //Publish
    ros::Publisher parking_path_pub;
    ros::Publisher stop_area_pub;

    //Subscriber
    ros::Subscriber object_sub;
    ros::Subscriber state_sub;
    ros::Subscriber pose_sub;
    ros::Subscriber sim_pose_sub;

    //Callback
    void object_callback(const sensor_msgs::PointCloud &msg);
    void state_callback(const std_msgs::String &msg);
    void pose_callback(const geometry_msgs::PoseWithCovariance &msg);
    void sim_pose_callback(const nav_msgs::Odometry &msg);

    //Function
    void parsing_data(std::ifstream& fin, nav_msgs::Path &path);
    bool find_rightpath(nav_msgs::Path path, sensor_msgs::PointCloud object);
    void process();
};

#endif // PARKING_PATH_H
