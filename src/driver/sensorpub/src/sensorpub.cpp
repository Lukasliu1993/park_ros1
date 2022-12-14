#include <sensorpub/sensorpub.h>
namespace dzt {
    sensorpub::sensorpub(){
    }

    void sensorpub::imuCallback(const sensor_msgs::Imu::ConstPtr &msg) {
        sensor_msgs::Imu imu_msg;
        imu_msg = *msg;
        imu_msg.angular_velocity_covariance = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1e-6};
        imu_msg.linear_acceleration_covariance = {1e-6, 0.0, 0.0, 0.0, 1e-6, 0.0, 0.0, 0.0, 1e-6};
        pub_imu.publish(imu_msg);
    }

    void sensorpub::odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
        nav_msgs::Odometry odom_msg;
        odom_msg = *msg;
        odom_msg.pose.covariance = {1e-3, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                    0.0, 1e-3, 0.0, 0.0, 0.0, 0.0, 
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                    0.0, 0.0, 0.0, 0.0, 0.0, 1e-3};
        odom_msg.twist.covariance = {1e-3, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                    0.0, 1e-3, 0.0, 0.0, 0.0, 0.0, 
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                    0.0, 0.0, 0.0, 0.0, 0.0, 1e-3};
        pub_odom.publish(odom_msg);
    }
    
    void sensorpub::posCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
        geometry_msgs::PoseWithCovarianceStamped pose_msg;
        pose_msg.header = msg->header;
        pose_msg.pose.pose = msg->pose;
        pose_msg.pose.covariance = {1e1, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                    0.0, 1e1, 0.0, 0.0, 0.0, 0.0, 
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.1};
        pub_pos.publish(pose_msg);
    }

    void sensorpub::slamCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
        nav_msgs::Odometry slam_msg;
        slam_msg.header.stamp = msg->header.stamp;
        slam_msg.header.frame_id = "odom";
        slam_msg.child_frame_id = "base_link";
        slam_msg.pose.pose = msg->pose;
        slam_msg.pose.covariance = {0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                    0.0, 0.05, 0.0, 0.0, 0.0, 0.0, 
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.05};
        pub_slam.publish(slam_msg);

        geometry_msgs::PoseWithCovarianceStamped pose_msg;
        pose_msg.header = msg->header;
        pose_msg.pose.pose = msg->pose;
        pose_msg.pose.covariance = {0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                    0.0, 0.05, 0.0, 0.0, 0.0, 0.0, 
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.05};
        pub_slam2.publish(pose_msg);
    }
    void sensorpub::run() {
        ros::NodeHandle node;
        ros::NodeHandle private_node("~");

        pub_imu = node.advertise<sensor_msgs::Imu>("/imu_cor", 100);
        pub_odom = node.advertise<nav_msgs::Odometry>("/odom_cor", 100);
        pub_pos = node.advertise<geometry_msgs::PoseWithCovarianceStamped>("gps_cor", 100);
        pub_slam = node.advertise<nav_msgs::Odometry>("slam_cor", 100);
        pub_slam2 = node.advertise<geometry_msgs::PoseWithCovarianceStamped>("slam_cor2", 100);
        sub_imu = node.subscribe("/fixposition/rawimu", 100, &sensorpub::imuCallback, this);
        sub_odom = node.subscribe("/odom", 100, &sensorpub::odomCallback, this);
        sub_pos = node.subscribe("/tracked_pose", 100, &sensorpub::posCallback, this);
        sub_slam = node.subscribe("/tracked_slam", 100, &sensorpub::slamCallback, this);
        ros::spin();
    }
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "sensorpub");
    dzt::sensorpub driver;
    driver.run();
    return 0;
}
