#include "point_tracking.h"

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "point_tracking");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    point_tracking tracking;

    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        tracking.process();
        loop_rate.sleep();
        ros::spinOnce();
    }

    ros::spin();
    return 0;
}
