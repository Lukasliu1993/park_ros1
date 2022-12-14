#include "My_Filter.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "my_pcl_node");

    My_Filter filter;

    ros::spin();

    return 0;
}
