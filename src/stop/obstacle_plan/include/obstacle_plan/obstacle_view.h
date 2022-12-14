#ifndef PROJECT_SIMULATION_LOCATION_NODE_H
#define PROJECT_SIMULATION_LOCATION_NODE_H

#include <string>
#include <ros/ros.h>
#include <cmath>
#include "autoware_msgs/DetectedObject.h"
#include "autoware_msgs/DetectedObjectArray.h"
#include "obstacle_plan/Pause.h"
#include "std_msgs/Bool.h"

namespace dtail_simulation {

    class Obstacleview {

    public:
        Obstacleview();

        ~Obstacleview() = default;

        void run();

    private:
        void obs_callback(const autoware_msgs::DetectedObjectArray::ConstPtr &msg);
    private:
        std::string sub_topic, pub_topic;
        // std::string gps_odom_pub_topic;
        ros::Subscriber ob_sub_;
        ros::Publisher pause_pub_;
        // ros::Publisher gps_odom_pub_;
        ros::NodeHandle nh_;
        bool pause_;
        double max_dis,min_v,min_time,min_ang,len,high,r,x_max,x_min;
        int model;
        ros::Time stop;
        ros::Time start;
    };
}


#endif //PROJECT_SIMULATION_LOCATION_NODE_H
