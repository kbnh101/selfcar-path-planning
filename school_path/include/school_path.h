#ifndef SCHOOL_PATH_H
#define SCHOOL_PATH_H
#include<ros/ros.h>
#include<iostream>
#include<string.h>
#include<vector>
#include <fstream>
#include <istream>

#include<nav_msgs/Path.h>
#include<nav_msgs/Odometry.h>

#include<geometry_msgs/PointStamped.h>
#include<geometry_msgs/PoseWithCovarianceStamped.h>

#include<std_msgs/Int32.h>
#include<std_msgs/String.h>

class school_path
{
    bool flag;
    double offset_x, offset_y;
public:
    school_path();
    //Data
    nav_msgs::Path global_path;

    std_msgs::String state;

    std::vector<double> wp_x;
    std::vector<double> wp_y;
    //Callback
    void StartPointCallback(const geometry_msgs::PoseWithCovarianceStamped& msg);
    //Function
    void x_data(std::ifstream& fin);
    void y_data(std::ifstream& fin);
    void process();
    //Publish
    ros::Publisher path_pub;
    ros::Publisher state_pub;
    //Subscribe
    ros::Subscriber startPoint_sub;
};

#endif // SCHOOL_PATH_H
