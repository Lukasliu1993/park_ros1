#ifndef AUTOLABOR_CANBUS_DRIVER_PM1_DRIVER_H
#define AUTOLABOR_CANBUS_DRIVER_PM1_DRIVER_H

#include <climits>

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf2_ros/transform_broadcaster.h>

#include "can_bus/CanBusMessage.h"
#include "can_bus/CanBusService.h"
#include "can_bus/cmd.h"
#include <cmath>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
// extern "C" {
// #include "control_model/model.h"
// #include "control_model/motor_map.h"
// #include "control_model/optimization.h"
// }

static double speedtable[11] = {
  0, 0.06, 0.149, 0.237, 0.324, 0.410, 0.499, 0.585, 0.673, 0.760, 0.842};
static double pushtable[11] = {6.0,6.0,7.45,7.90,8.1,8.2,8.32,8.36,8.41,8.44,8.42};
const int CANBUS_NODETYPE_ECU = 1;
union HEX {
    float num;
    unsigned char hex_num[4];
};

namespace autolabor_driver {


    class Pm1Driver {
    public:
        Pm1Driver();

        ~Pm1Driver() = default;

        void run();

    private:

        void handle_canbus_msg(const can_bus::CanBusMessage::ConstPtr &msg);

        void handle_twist_msg(const geometry_msgs::Twist::ConstPtr &msg);

        void parse_msg(const ros::TimerEvent &);


    private:
        geometry_msgs::Twist twist_cache_;
        double twist_timeout_;
        std::string odom_frame_, base_frame_;
        int rate_;
        double sync_timeout_;
        double max_speed_;
        double current_vel_l, current_vel_r, last_a, last_v;
        float current_angle;
        float wheelbase, wheeltread;
        float speed_cor, angle_cor;
        bool publish_tf_;
        bool first_send_odom_flag_;
        double accumulation_x_, accumulation_y_, accumulation_yaw_;
        ros::Timer odom_timer;
        ros::Time last_send_odom_time_;
        tf2_ros::TransformBroadcaster br_;

        ros::Subscriber canbus_msg_subscriber_;
        ros::Subscriber twist_subscriber_;
        ros::ServiceClient canbus_client_;
        ros::Publisher odom_pub_,v_receive_pub_,a_receive_pub_,v_send_pub_,a_send_pub_,vel_send_pub_;
 
    };

}


#endif //AUTOLABOR_CANBUS_DRIVER_PM1_DRIVER_H
