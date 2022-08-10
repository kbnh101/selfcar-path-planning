#include "school_path.h"

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "school_path_node");

    std::ifstream fin_x;
    std::ifstream fin_y;

    fin_x.open("/home/a/selfcar_ws/src/school_path/path_data/utm_x.txt");
    fin_y.open("/home/a/selfcar_ws/src/school_path/path_data/utm_y.txt");

    school_path path;

    path.x_data(fin_x);
    path.y_data(fin_y);

    fin_x.close();
    fin_y.close();

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
