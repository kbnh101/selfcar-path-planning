#include "parking_path.h"

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "parking_path");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    std::string location = "/home/a/morai_ws/src/morai/parking_path/path_data/";

    std::ifstream diagonal_path1;
    std::ifstream diagonal_path2;
    std::ifstream diagonal_path3;
    std::ifstream diagonal_path4;
    std::ifstream diagonal_path5;
    std::ifstream diagonal_path6;
    std::ifstream parallel_path1;
    std::ifstream parallel_path2;

    diagonal_path1.open(location + "diagonal_path1.txt");
    diagonal_path2.open(location + "diagonal_path2.txt");
    diagonal_path3.open(location + "diagonal_path3.txt");
    diagonal_path4.open(location + "diagonal_path4.txt");
    diagonal_path5.open(location + "diagonal_path5.txt");
    diagonal_path6.open(location + "diagonal_path6.txt");
    parallel_path1.open(location + "parallel_path1.txt");
    parallel_path2.open(location + "parallel_path2.txt");

    parking_path path;

    path.parsing_data(diagonal_path1, path.diagonal_path1);
    path.parsing_data(diagonal_path2, path.diagonal_path2);
    path.parsing_data(diagonal_path3, path.diagonal_path3);
    path.parsing_data(diagonal_path4, path.diagonal_path4);
    path.parsing_data(diagonal_path5, path.diagonal_path5);
    path.parsing_data(diagonal_path6, path.diagonal_path6);
    path.parsing_data(parallel_path1, path.parallel_path1);
    path.parsing_data(parallel_path2, path.parallel_path2);

    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        path.process();
        loop_rate.sleep();
        ros::spinOnce();
    }

    ros::spin();
    return 0;
}
