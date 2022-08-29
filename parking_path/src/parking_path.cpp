#include "parking_path.h"

parking_path::parking_path()
{
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    pnh.param("offset_x", offset_x, 302459.942);
    pnh.param("offset_y", offset_y, 4122635.537);

    //Publish
    parking_path_pub = nh.advertise<nav_msgs::Path>("parking_path",10);

    //Subscriber
    object_sub = nh.subscribe("center_points",10,&parking_path::object_callback,this);
    state_sub = nh.subscribe("state",10,&parking_path::state_callback,this);
    pose_sub = nh.subscribe("/Perception/Localization/LocalPose",10,&parking_path::pose_callback,this);
    sim_pose_sub = nh.subscribe("/odom",10,&parking_path::sim_pose_callback,this);
}

void parking_path::object_callback(const sensor_msgs::PointCloud &msg)
{
    object = msg;
}

void parking_path::state_callback(const std_msgs::String &msg)
{
    state = msg.data;
}

void parking_path::pose_callback(const geometry_msgs::PoseWithCovariance &msg)
{
    pose = msg;
    pose.pose.position.x = (msg.pose.position.x) - offset_x;
    pose.pose.position.y = (msg.pose.position.y) - offset_y;

    tf::Quaternion car_orientation(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
    tf::Matrix3x3 matrix(car_orientation);
    matrix.getRPY(vehicle_roll, vehicle_pitch, vehicle_yaw);
}

void parking_path::sim_pose_callback(const nav_msgs::Odometry &msg)
{
    pose = msg.pose;
    pose.pose.position.x = (msg.pose.pose.position.x) - 0.0;
    pose.pose.position.y = (msg.pose.pose.position.y) - 0.0;

    tf::Quaternion car_orientation(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
    tf::Matrix3x3 matrix(car_orientation);
    matrix.getRPY(vehicle_roll, vehicle_pitch, vehicle_yaw);
}

void parking_path::parsing_data(std::ifstream& fin, nav_msgs::Path& path)
{
    std::string line;

    int start;
    int end;

    path_data temp_data;

    if(fin.is_open())
    {
        path.header.stamp = ros::Time::now();
        path.header.frame_id = "map";

        while(getline(fin, line))
        {
            geometry_msgs::PoseStamped temp_pose;

            start = line.find("\t", 0);
            end = line.size() - 1;

            std::string utm_x = line.substr(0, start);
            std::string utm_y = line.substr(start + 1, end);

            temp_pose.pose.position.x = std::stod(utm_x.c_str()) - offset_x;
            temp_pose.pose.position.y = std::stod(utm_y.c_str()) - offset_y;

            path.poses.push_back(temp_pose);
            temp_data.path = path;
            temp_data.cost = 1;
        }
        path_withcost.push_back(temp_data);
    }
    else
    {
        std::cout << "Unable to open file" << std::endl;
    }
}

bool parking_path::find_rightpath(nav_msgs::Path path, sensor_msgs::PointCloud object)
{
    if(object.points.size() > 0)
    {
        double min_dist = 100.0;
        int closest_index = 0;

        for(int i = 0; i < path.poses.size(); i++)
        {
            for(int j = 0; j < object.points.size(); j++)
            {
                double path_x = path.poses.at(i).pose.position.x;
                double path_y = path.poses.at(i).pose.position.y;

                double rotated_x = cos(vehicle_yaw)*object.points.at(j).x - sin(vehicle_yaw)*object.points.at(j).y;
                double rotated_y = sin(vehicle_yaw)*object.points.at(j).x + cos(vehicle_yaw)*object.points.at(j).y;

                double obj_x = rotated_x + pose.pose.position.x;
                double obj_y = rotated_y + pose.pose.position.y;

                double dx = path_x - obj_x;
                double dy = path_y - obj_y;

                double dist = sqrt(dx*dx + dy*dy);

                if(dist<min_dist)
                {
                    min_dist = dist;
                    closest_index = j;
                }
            }
        }

        if(min_dist < 1)
            return false;
        else
            return true;
    }
    else
    {
        return true;
    }
}

void parking_path::process()
{
    if(state == "diagonal_parking")
    {
        int index = 0;
        if(find_rightpath(diagonal_path1, object) == false && path_withcost.at(0).cost == 1)
        {
            path_withcost.at(0).cost = 100;
        }
        else if(find_rightpath(diagonal_path2, object) == false && path_withcost.at(1).cost == 1)
        {
            path_withcost.at(1).cost = 100;
        }
        else if(find_rightpath(diagonal_path3, object) == false && path_withcost.at(2).cost == 1)
        {
            path_withcost.at(2).cost = 100;
        }
        else if(find_rightpath(diagonal_path4, object) == false && path_withcost.at(3).cost == 1)
        {
            path_withcost.at(3).cost = 100;
        }
        else if(find_rightpath(diagonal_path5, object) == false && path_withcost.at(4).cost == 1)
        {
            path_withcost.at(4).cost = 100;
        }
        else if(find_rightpath(diagonal_path6, object) == false && path_withcost.at(5).cost == 1)
        {
            path_withcost.at(5).cost = 100;
        }
        for(int i = 0; i<path_withcost.size(); i++)
        {
            if(path_withcost.at(i).cost == 1)
            {
                index = i;
                break;
            }
        }
        parking_path_pub.publish(path_withcost.at(index).path);
    }
    else if(state == "parallel_parking")
    {
        if(find_rightpath(parallel_path1, object) == true)
        {
            parking_path_pub.publish(parallel_path1);
        }
        else if(find_rightpath(parallel_path2, object) == true)
        {
            parking_path_pub.publish(parallel_path2);
        }
    }
    else
    {
        nav_msgs::Path default_path;
        parking_path_pub.publish(default_path);
    }
}
