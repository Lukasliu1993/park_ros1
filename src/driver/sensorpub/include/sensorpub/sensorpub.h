
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include "sensor_msgs/Imu.h"
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
namespace dzt {
    class sensorpub {
        public:
            sensorpub();

            ~sensorpub() = default;

            void run();

        private:

            void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
            void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);
            void posCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
            void slamCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
        private:
            ros::Subscriber sub_imu, sub_odom, sub_pos, sub_slam;
            ros::Publisher pub_imu, pub_odom, pub_pos, pub_slam, pub_slam2;
    };
}


