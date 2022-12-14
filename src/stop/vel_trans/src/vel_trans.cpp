#include <vel_trans/vel_trans.h>
namespace dtail_simulation  {
    vel_trans::vel_trans() {};

    void vel_trans::vel_callback(const geometry_msgs::Twist::ConstPtr &msg) {
        // ROS_INFO_STREAM("!!!!!!!!!!!!!!!!!!!!!ecommad=" << command);
        if(!stop_  && !emergency_ ){
            // ROS_INFO_STREAM("@@@@@@@@@@@@@@@@@@ecommad=" << command);
            geometry_msgs::Twist vel_msg;
            vel_msg = *msg;
            vel_pub_.publish(vel_msg); 
        }
        else{
            geometry_msgs::Twist vel_msg;
            vel_msg.linear.x = 0;
            vel_msg.linear.y = 0;
            vel_msg.linear.z = 0;
            if(model_){
            vel_msg.angular.x = 0;
            vel_msg.angular.y = 0;
            vel_msg.angular.z = 0;
            }
            else{
            vel_msg.angular.x = msg->angular.x;
            vel_msg.angular.y = msg->angular.y;
            vel_msg.angular.z = msg->angular.z;
            }
            vel_pub_.publish(vel_msg); 

        }
        
    }
    void vel_trans::stop_callback(const std_msgs::Bool::ConstPtr &msg) {
        std_msgs::Bool stop_msg;
        stop_msg = *msg;
        if(stop_msg.data == true){
            stop_ = true;
        }
        else if(stop_msg.data == false){
            stop_ = false;
        }   
    }

    void vel_trans::emg_callback(const std_msgs::Bool::ConstPtr &msg) {
        std_msgs::Bool emg_msg;
        emg_msg = *msg;
        if(emg_msg.data == true){
            emergency_ = true;
        }
        else if(emg_msg.data == false){
            emergency_ = false;
        }  
        
    }
    void vel_trans::run() {
        ros::NodeHandle private_node("~");
        private_node.param<std::string>("sub_topic", sub_topic, "cmd_vel");
        private_node.param<std::string>("pub_topic", pub_topic, "cmd_vel2");
        private_node.param<std::string>("pause_topic", pause_topic, "pause_msg");
        private_node.param<std::string>("emgency_topic", emgency_topic, "emgency_stop");
        private_node.param<bool>("vel_model", model_, true);
        stop_ = false;
        emergency_ = false;
        vel_subscribe_ = nh_.subscribe<geometry_msgs::Twist>(sub_topic,
            10,
            &vel_trans::vel_callback,
            this);
        stop_subscribe_ = nh_.subscribe<std_msgs::Bool>(pause_topic,
            10,
            &vel_trans::stop_callback,
            this);
        emg_subscribe_= nh_.subscribe<std_msgs::Bool>(emgency_topic,
            10,
            &vel_trans::emg_callback,
            this);
        vel_pub_ = nh_.advertise<geometry_msgs::Twist>(pub_topic, 100);
        ros::spin();
    }
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "vel_trans");
    dtail_simulation::vel_trans vel_trans_;
    vel_trans_.run();
    return 0;
}