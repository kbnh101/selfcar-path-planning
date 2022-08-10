#include "local_path.h"

local_path::local_path()
{
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    pnh.param("num",num1,30);
}

void local_path::path_tracking(nav_msgs::Path global_path)
{
    if(flag == true)
    {
        double least_dist = 10;
        int temp_num = 0;
        int num = this->num;

        for(int i = 0; i<global_path.poses.size(); i++)
        {
            double dx = pose.pose.position.x - global_path.poses.at(i).pose.position.x;
            double dy = pose.pose.position.y - global_path.poses.at(i).pose.position.y;

            double dist = sqrt(dx*dx + dy*dy);
            if(dist<least_dist)
            {
                least_dist = dist;
                temp_num = i;
            }
        }

        if(prev_temp_num < 0)
        {
            temp_num = temp_num;
        }
        else if(temp_num >= prev_temp_num + 20 || temp_num <= prev_temp_num - 20)
        {
            temp_num = prev_temp_num;
        }

        tracking_path.header.stamp = ros::Time::now();
        tracking_path.header.frame_id = "map";
        tracking_path.poses.clear();

        geometry_msgs::PoseStamped temp_pose;
        temp_pose.header.stamp = ros::Time::now();
        temp_pose.header.frame_id = "map";

        int local_path_size = temp_num + num1;

        if(local_path_size >= global_path.poses.size()-1)
        {
            num1 -= 1;
        }

        if(temp_num + num1 <= global_path.poses.size())
        {
            for(int i = temp_num; i< temp_num + num1; i++)
            {
                temp_pose.pose.position.x = global_path.poses.at(i).pose.position.x;
                temp_pose.pose.position.y = global_path.poses.at(i).pose.position.y;
                temp_pose.pose.position.z = 0;
                tracking_path.poses.push_back(temp_pose);
            }
        }
        else
        {
            flag = false;
        }
        prev_temp_num = temp_num;
        path_pub.publish(tracking_path);
    }
    else
    {
        //ROS_INFO("DONE");
    }
}

void local_path::local_map()
{
    double x = 40;
    double y = 40;
    small_map.header.stamp = ros::Time::now();
    small_map.header.frame_id = "local";
    small_map.info.map_load_time = ros::Time::now();
    small_map.info.resolution = 1;
    small_map.info.width = x;
    small_map.info.height = y;
    small_map.info.origin.position.x = -20;
    small_map.info.origin.position.y = -20;
    small_map.info.origin.orientation.w = 1;
    small_map.data.assign(x*y,0);

    small_map_pub.publish(small_map);
}

void local_path::process()
{
    path_tracking(path);
    if(flag == true)
    {
        for(int i = 0; i < prev_temp_num; i++)
        {
            path.poses.at(i).pose.position.x = 10000.0;
            path.poses.at(i).pose.position.y = 10000.0;
        }
        speed = speed;
        cmd_vel.angular.z = -1 * steering_angle(pose, tracking_path, vehicle_yaw, speed);
        if(is_look_foward_point == true)
        {
            cmd_vel.linear.x = speed;
        }
        else
        {
            cmd_vel.linear.x = 0;
        }
    }
    else if(flag==false)
    {
        cmd_vel.angular.z = 0;
        cmd_vel.linear.x = 0;
        prev_temp_num = -1;
	path = temp_path;
    }
    cmd_pub.publish(cmd_vel);
}

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "local_path");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    ros_data data;
    local_path path;
    data.flag = false;

    ros::Rate loop_rate(path.rate);
    while(ros::ok())
    {
        path.process();
        loop_rate.sleep();
        ros::spinOnce();
    }

    ros::spin();
    return 0;
}
