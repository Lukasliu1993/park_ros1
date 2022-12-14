#include "pcl_test_core.h"

PclTestCore::PclTestCore(){}
void PclTestCore::run() {
    ros::NodeHandle node;
    ros::NodeHandle private_node("~");
    private_node.param<std::string>("input_topic", input_topic_, std::string("/camera/depth/points"));
    private_node.param<std::string>("output_topic", output_topic_, std::string("/camera/depth/filtered_points"));
    private_node.param<float>("resolution", resolution_, 0.4);
    sub_point_cloud_ = node.subscribe(input_topic_,10, &PclTestCore::point_cb, this);
    pub_filtered_points_ = node.advertise<sensor_msgs::PointCloud2>(output_topic_, 10);
    ros::spin();
}
PclTestCore::~PclTestCore(){}

void PclTestCore::Spin(){
    
}

void PclTestCore::point_cb(const sensor_msgs::PointCloud2ConstPtr & in_cloud_ptr){
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromROSMsg(*in_cloud_ptr, *current_pc_ptr);

    pcl::VoxelGrid<pcl::PointXYZ> vg;

    vg.setInputCloud(current_pc_ptr);
    vg.setLeafSize(resolution_, resolution_, resolution_);
    vg.filter(*filtered_pc_ptr);

    sensor_msgs::PointCloud2 pub_pc;
    pcl::toROSMsg(*filtered_pc_ptr, pub_pc);

    pub_pc.header = in_cloud_ptr->header;

    pub_filtered_points_.publish(pub_pc);
}
int main(int argc, char **argv) {
    ros::init(argc, argv, "pcl_test");
    PclTestCore driver;
    driver.run();
    return 0;
}