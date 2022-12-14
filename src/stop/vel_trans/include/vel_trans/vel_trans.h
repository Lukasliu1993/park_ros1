#ifndef PROJECT_VEL_TRANS_NODE_H
#define PROJECT_VEL_TRANS_NODE_H

#include <string>
#include <ros/ros.h>
#include <cmath>

// #include "geometry_msgs/PoseWithCovarianceStamped.h"
// #include "geometry_msgs/PoseStamped.h"
// #include <tf/transform_listener.h>
// #include <tf/transform_datatypes.h>
#include "geometry_msgs/Twist.h"
#include "std_msgs/Bool.h"
namespace dtail_simulation {

    class vel_trans {

    public:
        vel_trans();

        ~vel_trans() = default;

        void run();

    private:
        void vel_callback(const geometry_msgs::Twist::ConstPtr &msg);
        void stop_callback(const std_msgs::Bool::ConstPtr &msg);
        void emg_callback(const std_msgs::Bool::ConstPtr &msg);
    private:
        std::string sub_topic, pub_topic,pause_topic,emgency_topic;
        bool model_,stop_,emergency_;
        // std::string gps_odom_pub_topic;
        ros::Subscriber vel_subscribe_ ,stop_subscribe_, emg_subscribe_;
        ros::Publisher vel_pub_;
        // ros::Publisher gps_odom_pub_;
        ros::NodeHandle nh_;
    };
}


#endif //PROJECT_SIMULATION_LOCATION_NODE_H
