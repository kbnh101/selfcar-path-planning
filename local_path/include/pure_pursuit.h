#include<ros/ros.h>
#include<math.h>
#include<cmath>
#include<vector>
#include<queue>
#include<tf/tf.h>
#include<visualization_msgs/MarkerArray.h>
#include<std_msgs/Int32.h>

#include<nav_msgs/Path.h>
#include<nav_msgs/Odometry.h>

#include"pid.h"

class pure_pursuit : public pid{
    double max_lfd, min_lfd, VL, L;
public:
    pure_pursuit();
    bool is_look_foward_point;

    //Publisher
    ros::Publisher marker_lfd;
    //Function
    void lfd_visualiztion(geometry_msgs::Pose index);
    double steering_angle(geometry_msgs::PoseWithCovariance pose, nav_msgs::Path tracking_path, double vehicle_yaw, double velocity);
};
