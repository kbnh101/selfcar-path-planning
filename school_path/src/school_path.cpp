#include "school_path.h"

school_path::school_path()
{
    ros::NodeHandle nh;
    ros::NodeHandle pnh;

    pnh.param("offset_x", offset_x, 362169.6526);
    pnh.param("offset_y", offset_y, 4054407.07);

    //publish
    path_pub = nh.advertise<nav_msgs::Path>("global_path",10);
    state_pub = nh.advertise<std_msgs::String>("state",10);
    //subscribe
    startPoint_sub = nh.subscribe("initialpose", 10, &school_path::StartPointCallback,this);

    flag = false;
}

void school_path::StartPointCallback(const geometry_msgs::PoseWithCovarianceStamped &msg)
{
    flag = true;
}

void school_path::x_data(std::ifstream &fin)
{
    std::string line;

    if(fin.is_open()){
        while(getline(fin, line))
        {
            double data = std::stod(line.c_str()) - offset_x;
            wp_x.push_back(data);
        }
    }
    else
    {
        std::cout << "Unable to open file" << std::endl;
    }
}

void school_path::y_data(std::ifstream &fin)
{
    std::string line;

    if(fin.is_open()){
        while(getline(fin, line))
        {
            double data = std::stod(line.c_str()) - offset_y;
            wp_y.push_back(data);
        }
    }
    else
    {
        std::cout << "Unable to open file" << std::endl;
    }
}

void school_path::process()
{
    if(flag == true)
    {
        global_path.header.stamp = ros::Time::now();
        global_path.header.frame_id = "map";
        global_path.poses.clear();

        geometry_msgs::PoseStamped temp_pose;
        temp_pose.header.stamp = ros::Time::now();
        temp_pose.header.frame_id = "map";

        for(int i = 0; i < wp_x.size(); i++)
        {
            temp_pose.pose.position.x = wp_x.at(i);
            temp_pose.pose.position.y = wp_y.at(i);
            temp_pose.pose.position.z = 0;

            global_path.poses.push_back(temp_pose);
        }
        path_pub.publish(global_path);
        global_path.poses.clear();

        flag = false;
    }
    std::stringstream state_stream;
    std::string temp_state;

    temp_state = "no";

    state_stream << temp_state;
    state.data = state_stream.str();

    state_pub.publish(state);
}
