#include "local_path.h"

local_path::local_path()
{
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    pnh.param<int>("num",num,25);
    pnh.param<double>("path_param",avoid_path_param, 4.5);
    pnh.param<double>("avoid_mindist",avoid_mindist, 1.0);
    pnh.param<double>("avoid_objdist",avoid_objdist, 10.0);
    pnh.param<double>("dynamic_mindist",dynamic_mindist, 1.0);
    pnh.param<double>("dynamic_objdist",dynamic_objdist, 10.0);

    avoid = false;
    dynamic = false;
}

std::pair<int, double> local_path::calc_dist(sensor_msgs::PointCloud object, nav_msgs::Path path)
{
    std::pair<int, double> index_dist;

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

    index_dist.first = closest_index;
    index_dist.second = min_dist;

    return index_dist;
}

void local_path::dynamic_object(sensor_msgs::PointCloud object)
{
    if(object.points.size() > 0)
    {
        std::pair<int, double> index_dist = calc_dist(object, path);

        double min_dist = index_dist.second;
        int closest_index = index_dist.first;

        double x = object.points.at(closest_index).x;
        double y = object.points.at(closest_index).y;

        double obj_distance = sqrt(x*x + y*y);

        if(min_dist < dynamic_mindist && obj_distance < dynamic_objdist)
            dynamic = true;
        else
            dynamic = false;
    }
}

void local_path::static_object(sensor_msgs::PointCloud object, nav_msgs::Path path)
{
    if(object.points.size() > 0 && avoid == false)
    {
        std::pair<int, double> index_dist = calc_dist(object, path);

        double min_dist = index_dist.second;
        int closest_index = index_dist.first;

        double x = object.points.at(closest_index).x;
        double y = object.points.at(closest_index).y;

        double obj_distance = sqrt(x*x + y*y);

        if(min_dist < avoid_mindist && obj_distance < avoid_objdist)
            avoid = true;
    }
    else if(object.points.size() > 0 && avoid == true)
    {
        std::pair<int, double> index_dist = calc_dist(object, path);

        double min_dist = index_dist.second;
        int closest_index = index_dist.first;

        double x = object.points.at(closest_index).x;
        double y = object.points.at(closest_index).y;

        double obj_distance = sqrt(x*x + y*y);

        if(min_dist < avoid_mindist && obj_distance < avoid_objdist)
            avoid = false;
    }
}

double local_path::node_distance()
{
    if(node_list.points.size() > 0)
    {
        std::vector<geometry_msgs::Point32> temp_point;

        temp_point.push_back(node_list.points.at(2));//node3
        temp_point.push_back(node_list.points.at(8));//node9
        temp_point.push_back(node_list.points.at(10));//node11
        temp_point.push_back(node_list.points.at(29));//node30
        //traffic light node pushback

        double index = 0;
        double min_dis = 50;

        for(int i = 0; i<temp_point.size(); i++)
        {
            double dx = temp_point.at(i).x - temp_pose.pose.position.x;
            double dy = temp_point.at(i).y - temp_pose.pose.position.y;

            double dis = sqrt(dx*dx + dy*dy);

            if(min_dis > dis)
            {
                min_dis = dis;
                index = i;
            }
        } // 신호등 노드 중에 현재 자차량 위치와 가장 가까운 점 탐색
        double tx = temp_point.at(index).x - temp_pose.pose.position.x;
        double ty = temp_point.at(index).y - temp_pose.pose.position.y;

        double dist = sqrt(tx*tx + ty*ty);

        return dist;// 신호등 노드와의 거리 return
    }
}

void local_path::path_tracking(nav_msgs::Path global_path)
{
    if(flag == true)
    {
        double least_dist = 10;
        int temp_num = 0;
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
        tracking_path.header.stamp = ros::Time::now();
        tracking_path.header.frame_id = "map";
        tracking_path.poses.clear();

        geometry_msgs::PoseStamped temp_pose;
        temp_pose.header.stamp = ros::Time::now();
        temp_pose.header.frame_id = "map";

        int local_path_size = temp_num + num;

        if(local_path_size >= global_path.poses.size()-1)
        {
            num -= 1;
        }
        if(temp_num + num <= global_path.poses.size())
        {
            for(int i = temp_num; i< temp_num + num; i++)
            {
                if(avoid == false)
                {
                    temp_pose.pose.position.x = global_path.poses.at(i).pose.position.x;
                    temp_pose.pose.position.y = global_path.poses.at(i).pose.position.y;
                    temp_pose.pose.position.z = 0;
                    tracking_path.poses.push_back(temp_pose);
                }
                else if(avoid == true)
                {
                    temp_pose.pose.position.x = global_path.poses.at(i).pose.position.x + avoid_path_param*sin(vehicle_yaw);
                    temp_pose.pose.position.y = global_path.poses.at(i).pose.position.y - avoid_path_param*cos(vehicle_yaw);
                    temp_pose.pose.position.z = 0;
                    tracking_path.poses.push_back(temp_pose);
                }
            }

        }

        prev_temp_num = temp_num;
        path_pub.publish(tracking_path);
    }
    else
    {
        //ROS_INFO("DONE");
    }
}

bool local_path::parking_stop(geometry_msgs::Polygon stop)
{
    double min_dist = 100;
    int index = 0;

    for(int i = 0; i < stop.points.size(); i++)
    {
        double dx = pose.pose.position.x - stop.points.at(i).x;
        double dy = pose.pose.position.y - stop.points.at(i).y;

        double dist = sqrt(dx*dx + dy*dy);

        if(min_dist > dist)
        {
            index = i;
            min_dist = dist;
        }
    }

    if(min_dist <= 0.5)
        return true;
    else
        return false;
}

void local_path::local_map()
{
//    double x = 40;
//    double y = 40;
//    small_map.header.stamp = ros::Time::now();
//    small_map.header.frame_id = "local";
//    small_map.info.map_load_time = ros::Time::now();
//    small_map.info.resolution = 1;
//    small_map.info.width = x;
//    small_map.info.height = y;
//    small_map.info.origin.position.x = -20;
//    small_map.info.origin.position.y = -20;
//    small_map.info.origin.orientation.w = 1;
//    small_map.data.assign(x*y,0);

//    small_map_pub.publish(small_map);
}

void local_path::process()
{
    if(state == "diagonal_parking" || state == "parallel_parking")
    {
        flag = true;
        num = 10;
        path_tracking(parking_path);
    }
    else
    {
        path_tracking(path);
    }
    if(flag == true)
    {
        //simulator
        cmd_vel_.longlCmdType = 2;
        cmd_vel_.steering = steering_angle(pose, tracking_path, vehicle_yaw, speed);
        cmd_vel_.brake = 0;
        //fucking erp42
        cmd_vel.angular.z = -1 * steering_angle(pose, tracking_path, vehicle_yaw, speed);
        if(camera_data.traffic_light == "RED")
        {
            std::cout<<"oh good"<<std::endl;
        }

        if(is_look_foward_point == true)
        {
            cmd_vel.linear.x = speed;
            cmd_vel_.velocity = speed;
            ///////////////////traffic light///////////////////
            if(camera_data.traffic_light == "YELLOW" )
            {
                cmd_vel.linear.x = 5;
                cmd_vel_.velocity = 5;
            }
            else if(camera_data.traffic_light == "RED" && node_distance() <10 && node_distance()>=5)
            {
                cmd_vel.linear.x = 5;
                cmd_vel_.velocity = 5;
            }
            else if(camera_data.traffic_light == "RED")
            {
                cmd_vel.linear.x = 0;
                cmd_vel_.velocity = 0;
            }
            ///////////////////avoid dynamic & static object///////////////////
            if(state == "static")
            {
                static_object(object_point, tracking_path);
            }
            else if(state == "dynamic")
            {
                dynamic_object(object_point);
            }
            ///////////////////stop dynamic object///////////////////
            if(dynamic == true)
            {
                cmd_vel_.velocity = 0;
                cmd_vel_.brake = 1;
                cmd_vel.linear.x = 0;
            }
            ///////////////////parking///////////////////
            if(parking_stop(parking_stop_list) == true && parking_state == false)
            {
                cmd_vel_.velocity = 0;
                cmd_vel_.brake = 1;
                cmd_vel.linear.x = 0;

                cmd_pub_.publish(cmd_vel_);
                cmd_pub.publish(cmd_vel);

                ros::Duration(5.0).sleep();
                parking_state = true;
                num = 20;
            }
        }
        else
        {
            cmd_vel.linear.x = 0;
            cmd_vel_.velocity = 0;
            flag = false;
        }
    }
    else if(flag==false)
    {
        cmd_vel.angular.z = 0;
        cmd_vel.linear.x = 0;
        cmd_vel_.velocity = 0;
        cmd_vel_.steering = 0;
        cmd_vel_.brake = 1;
    }
    cmd_pub.publish(cmd_vel);
    cmd_pub_.publish(cmd_vel_);
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
