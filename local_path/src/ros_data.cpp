#include "ros_data.h"

ros_data::ros_data()
{
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    pnh.param("UTM_OFFSET_X",UTM_OFFSET_X,0.0);
    pnh.param("UTM_OFFSET_Y",UTM_OFFSET_Y,0.0);

    //publish
    cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",10);
    cmd_pub_ = nh.advertise<morai_msgs::CtrlCmd>("/ctrl_cmd",10);
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
    sim_pose_sub = nh.subscribe("odom",10,&ros_data::sim_pose_callback,this);
    object_sub = nh.subscribe("center_points",10,&ros_data::object_callback,this);
    camera_sub = nh.subscribe("/Data_Transger",10,&ros_data::camera_callback,this);
    node_sub = nh.subscribe("node_pose",10,&ros_data::node_callback,this);
    parking_path_sub = nh.subscribe("parking_path",10,&ros_data::parking_path_callback,this);
    parking_stop_sub = nh.subscribe("fucking_stop",10,&ros_data::parking_stop_callback,this);

    parking_state = false;
}

void ros_data::sim_pose_callback(const nav_msgs::Odometry &msg)
{
    pose = msg.pose;
    pose.pose.position.x = (msg.pose.pose.position.x) - UTM_OFFSET_X;
    pose.pose.position.y = (msg.pose.pose.position.y) - UTM_OFFSET_Y;

    tf::Quaternion car_orientation(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
    tf::Matrix3x3 matrix(car_orientation);
    matrix.getRPY(vehicle_roll, vehicle_pitch, vehicle_yaw);

    temp_pose.header.stamp = ros::Time::now();
    temp_pose.header.frame_id = "map";
    temp_pose.pose.position.x = msg.pose.pose.position.x;
    temp_pose.pose.position.y = msg.pose.pose.position.y;
    temp_pose.pose.position.z = 0;
    temp_pose.pose.orientation.x = msg.pose.pose.orientation.x;
    temp_pose.pose.orientation.y = msg.pose.pose.orientation.y;
    temp_pose.pose.orientation.z = msg.pose.pose.orientation.z;
    temp_pose.pose.orientation.w = msg.pose.pose.orientation.w;

    pose_pub.publish(temp_pose);
}

void ros_data::trajectorycallback(const nav_msgs::Path& msg)
{
    trajectory_path = msg;
}

void ros_data::node_callback(const geometry_msgs::Polygon &msg)
{
    node_list = msg;
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

void ros_data::parking_path_callback(const nav_msgs::Path &msg)
{
    parking_path = msg;
}

void ros_data::parking_stop_callback(const geometry_msgs::Polygon &msg)
{
    parking_stop_list = msg;
}

void ros_data::speed_callback(const std_msgs::Int32 &msg)
{
    speed = msg.data;
}

void ros_data::state_callback(const std_msgs::String &msg)
{
    state = msg.data;
    if(parking_state == true && (state == "diagonal_parking" || state == "parallel_parking"))
    {
        state = "go";
        speed = 15;
    }
    else if(state != "digonal_parking" || state != "parallel_parking")
    {
        parking_state = false;
    }
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

void ros_data::object_callback(const sensor_msgs::PointCloud &msg)
{
    object_point = msg;
}

void ros_data::camera_callback(const data_transfer_msg::traffic_light &msg)
{
    traffic_data = msg;
}
