#include "point_tracking.h"

point_tracking::point_tracking()
{
    ros::NodeHandle nh;
    ros::NodeHandle pnh;

    std::string lidar_point_topic;
    std::string twist_topic;
    std::string sim_topic;

    pnh.param<std::string>("lidar_point_topic", lidar_point_topic, "center_point");
    pnh.param<std::string>("twist_topic", twist_topic, "cmd_vel");
    pnh.param<std::string>("sim_topic", sim_topic, "ctrl_cmd");

    pnh.param<double>("VL", VL, 1.0);
    pnh.param<int>("velocity", velocity, 50);
    pnh.param<double>("back_x", back_x, -1.7);
    pnh.param<double>("back_y", back_y, 0.0);
    pnh.param<double>("max_lfd", max_lfd, 10.0);
    pnh.param<double>("min_lfd", min_lfd, 5.0);
    pnh.param<double>("vel_param", vel_param, 5.0);

    //publish
    cmd_pub = nh.advertise<geometry_msgs::Twist>(twist_topic,10);
    ctrl_cmd = nh.advertise<morai_msgs::CtrlCmd>(sim_topic,10);

    //subscriber
    lidar_waypoint = nh.subscribe(lidar_point_topic,10,&point_tracking::way_pt_sub,this);

    is_look_foward_point = false;
}


void point_tracking::way_pt_sub(const sensor_msgs::PointCloud &msg)
{
    center_point = msg;
}

double point_tracking::steering_angle(sensor_msgs::PointCloud way_pt)
{
    double min_dist = 10;
    double min_index = 0;
    double steering = 0;
    double dis = 0;
    int lfd = 1;

    for(int i = 0; i<way_pt.points.size(); i++)
    {
        double dx = back_x - way_pt.points.at(i).x;
        double dy = back_y - way_pt.points.at(i).y;

        double dist = sqrt(dx*dx + dy*dy);
        if(dist<min_dist)
        {
            min_dist = dist;
            min_index = i;
        }
    }

    lfd = cmd_vel.linear.x/vel_param;

    if(lfd < min_lfd)
    {
        lfd = min_lfd;
    }
    else if(lfd > max_lfd)
    {
        lfd = max_lfd;
    }

    double dx,dy = 0;

    for(int i = 0; i<way_pt.points.size(); i++)
    {
        dx = way_pt.points.at(i).x - 1.7;
        dy = way_pt.points.at(i).y;

        if(dx > 0)
        {
            dis = sqrt(pow(dx,2) + pow(dy,2));
            if(dis>=lfd)
            {
                is_look_foward_point = true;
                break;
            }
        }
    }

    double theta = atan2(dy,dx);

    if(is_look_foward_point == true)
    {
        double eta = atan2((2*VL*sin(theta)),lfd);
        steering = eta;
    }
    else
    {
        ROS_INFO("no found forwad point");
    }

    return steering;
}

void point_tracking::process()
{
    cmd_vel.angular.z = steering_angle(center_point);
    cmd_vel.linear.x = velocity;

    cmd_vel_.longlCmdType = 2;
    cmd_vel_.velocity = velocity;
    cmd_vel_.steering = steering_angle(center_point);

    cmd_pub.publish(cmd_vel);
    ctrl_cmd.publish(cmd_vel_);
}
