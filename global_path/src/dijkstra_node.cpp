#include "dijkstra.h"

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "dijkstra");
    dijkstra path;

    ifstream file_edge("/home/a/test_ws/src/Dijkstra/path/edge.csv");
    ifstream file_node("/home/a/test_ws/src/Dijkstra/path/node.csv");

    path.node_parse(file_node);
    path.edge_parse(file_edge);

    file_edge.close();
    file_node.close();

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
      loop_rate.sleep();
      ros::spinOnce();
    }
    ros::spin();
    return 0;
}
