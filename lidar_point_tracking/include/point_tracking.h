#include<ros/ros.h>
#include<iostream>
#include<math.h>
#include<cmath>
#include<vector>
#include<string>

#include<geometry_msgs/Point.h>
#include<geometry_msgs/PointStamped.h>
#include<geometry_msgs/Twist.h>

#include<visualization_msgs/MarkerArray.h>

#include<std_msgs/Int32.h>
#include<std_msgs/String.h>

#include<sensor_msgs/PointCloud.h>

#include<pcl_ros/point_cloud.h>

#include<morai_msgs/CtrlCmd.h>

class point_tracking
{
    double max_lfd, min_lfd, back_x, back_y, vel_param, VL;
    int velocity;
    bool is_look_foward_point;
public:
    point_tracking();

    morai_msgs::CtrlCmd cmd_vel_;

    geometry_msgs::Twist cmd_vel;

    sensor_msgs::PointCloud center_point;

    double steering_angle(sensor_msgs::PointCloud way_pt);
    void process();
    //callback
    void way_pt_sub(const sensor_msgs::PointCloud &msg);
    //subscriber
    ros::Subscriber lidar_waypoint;
    //publisher
    ros::Publisher cmd_pub;
    ros::Publisher ctrl_cmd;
};
