
#include "pcl_test_core.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcl_test");

    ros::NodeHandle nh;

    PclTestCore core(nh);
    nh.param<std::string>("input_topic", input_topic_, std::string("/camera/depth/points"));
    nh.param<std::string>("output_topic", output_topic_, std::string("/camera/depth/filtered_points"));
    nh.param<float>("resolution", resolution_, 0.4);
    return 0;
}

