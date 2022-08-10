#include "ros_data.h"

ros_data::ros_data()
{
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    pnh.param("UTM_OFFSET_X",UTM_OFFSET_X,361001.412425);
    pnh.param("UTM_OFFSET_Y",UTM_OFFSET_Y,4065821.07176);

    //publish
    cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",10);
    path_pub = nh.advertise<nav_msgs::Path>("tracking_path",10);
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("current_pose",10);
    marker_vel = nh.advertise<visualization_msgs::MarkerArray>("/marker_vel",10);
    small_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("local_map",10);
    //subscibe
    pose_sub = nh.subscribe("/Perception/Localization/LocalPose",10,&ros_data::posecallback,this);
    path_sub = nh.subscribe("/global_path",10,&ros_data::pathcallback,this);
    trajectory = nh.subscribe("/trajectory",10,&ros_data::trajectorycallback,this);
    speed_sub = nh.subscribe("/speed",10,&ros_data::speed_callback,this);
    map_sub = nh.subscribe("map",10,&ros_data::map_callback,this);
    state_sub = nh.subscribe("state",10,&ros_data::state_callback,this);
}

void ros_data::trajectorycallback(const nav_msgs::Path& msg)
{
    trajectory_path = msg;
}

void ros_data::map_callback(const nav_msgs::OccupancyGrid &msg)
{
    global_map = msg;
}

void ros_data::pathcallback(const nav_msgs::Path& msg)
{
    path = msg;
    temp_path = msg;
    flag = true;
}

void ros_data::speed_callback(const std_msgs::Int32 &msg)
{
    speed = msg.data;
}

void ros_data::state_callback(const std_msgs::String &msg)
{
    state = msg.data;
}

void ros_data::posecallback(const geometry_msgs::PoseWithCovariance& msg)
{
    pose = msg;
    pose.pose.position.x = (msg.pose.position.x) - UTM_OFFSET_X;
    pose.pose.position.y = (msg.pose.position.y) - UTM_OFFSET_Y;

    tf::Quaternion car_orientation(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
    tf::Matrix3x3 matrix(car_orientation);
    matrix.getRPY(vehicle_roll, vehicle_pitch, vehicle_yaw);

    temp_pose.header.stamp = ros::Time::now();
    temp_pose.header.frame_id = "odom";
    temp_pose.pose.position.x = 0;
    temp_pose.pose.position.y = 0;
    temp_pose.pose.position.z = 0;
    temp_pose.pose.orientation.x = msg.pose.orientation.x;
    temp_pose.pose.orientation.y = msg.pose.orientation.y;
    temp_pose.pose.orientation.z = msg.pose.orientation.z;
    temp_pose.pose.orientation.w = msg.pose.orientation.w;

    pose_pub.publish(temp_pose);
}
