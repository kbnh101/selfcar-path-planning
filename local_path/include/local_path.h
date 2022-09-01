#include<ros/ros.h>
#include<iostream>
#include<math.h>
#include<cmath>
#include<vector>
#include<queue>
#include<tf/tf.h>
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

#include "pure_pursuit.h"
#include "ros_data.h"

#include<data_transfer_msg/data_transfer.h>

class local_path : public pure_pursuit, ros_data{
public:
    local_path();
    double rate = 10;
    int prev_temp_num;
    int num;
    double avoid_path_param, avoid_mindist, avoid_objdist, dynamic_mindist, dynamic_objdist;

    bool avoid;
    bool dynamic;

    morai_msgs::CtrlCmd cmd_vel_;

    //Function
    void local_map();
    void path_tracking(nav_msgs::Path global_path);
    void static_object(sensor_msgs::PointCloud object, nav_msgs::Path path);
    void dynamic_object(sensor_msgs::PointCloud object);
    double node_distance();
    bool parking_stop(geometry_msgs::Polygon stop);
    void process();
};
