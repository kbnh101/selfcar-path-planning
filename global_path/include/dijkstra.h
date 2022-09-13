#ifndef DIJKSTRA_H
#define DIJKSTRA_H
#include<ros/ros.h>
#include<nav_msgs/Path.h>
#include<nav_msgs/OccupancyGrid.h>
#include<nav_msgs/GetMap.h>
#include<nav_msgs/Odometry.h>

#include<geometry_msgs/PointStamped.h>
#include<geometry_msgs/PoseWithCovarianceStamped.h>
#include<geometry_msgs/Polygon.h>

#include<visualization_msgs/MarkerArray.h>

#include<std_msgs/Int32.h>
#include<std_msgs/String.h>

#include<iostream>
#include<opencv2/opencv.hpp>
#include<math.h>
#include<cmath>
#include"csv.h"
#include<queue>
#include<bitset>
#include<vector>


struct Node
{
    int id;
    double x;
    double y;
};

struct Edge
{
    int link_id;
    vector<double> way_point_x;
    vector<double> way_point_y;
    int from_node;
    int to_node;
    int cost;
    double speed;    
    std::string state;
};

struct path_info
{
    double x;
    double y;
    int vel;
    string state;
};

struct Graph
{
    int node_id;
    vector<Edge> E;
};

class dijkstra
{
public:
    dijkstra();
    cv::Point2d start_point, dst_point, pose_point;
    vector<double> speedv;
    vector<path_info> pathinfo;
    vector<Node> node;
    vector<Edge> edge;
    vector<Graph> G;
    int width, height;
    int exstart, exend;
    int near_start, near_end;
    bool flag;
    bool Done, fuckyou;
    double resolution;
    double OFFSET_X, OFFSET_Y;
    string edge_location;
    string node_location;
    queue<int> Queue;
    vector<Edge> spath;
    vector<int> shortest_path;

    vector<int>* dijkstra1(int start, int V, vector<pair<int,int> > adj[]);

    nav_msgs::OccupancyGrid map;

    geometry_msgs::Pose pose;
    geometry_msgs::Polygon node_pose;

    std_msgs::Int32 speed;

    //Function
    void node_parse(istream& file);
    void edge_parse(istream& file);
    void draw_path();
    void find_path(cv::Point2d start, cv::Point2d dst);
    void trace_path(int s, int e, vector<int>* from);
    void print_path(int s, int e, vector<int>* from);
    double nomalize(double num);

    //Callback
    void MapCallback(const nav_msgs::OccupancyGrid& msg);
    void TargetPointtCallback(const geometry_msgs::PoseStamped& msg);
    void StartPointCallback(const geometry_msgs::PoseWithCovarianceStamped& msg);
    void posecallback(const geometry_msgs::PoseWithCovariance& msg);
    void sim_posecallback(const nav_msgs::Odometry &msg);

    //Publish
    ros::Publisher path_pub;
    ros::Publisher marker_pub;
    ros::Publisher speed_pub;
    ros::Publisher state_pub;
    ros::Publisher node_pub;

    //Subscribe
    ros::Subscriber map_sub;
    ros::Subscriber startPoint_sub;
    ros::Subscriber targetPoint_sub;
    ros::Subscriber pose_sub;
    ros::Subscriber path_sub;
    ros::Subscriber sim_pose_sub;
};

#endif // DIJKSTRA_H
