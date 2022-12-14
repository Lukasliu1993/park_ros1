#include "can_bus/dzt_p1_driver.h"
namespace autolabor_driver {


    Pm1Driver::Pm1Driver() : current_vel_l(0.0),current_vel_r(0.0),first_send_odom_flag_(true){
    }
    void Pm1Driver::handle_canbus_msg(const can_bus::CanBusMessage::ConstPtr &msg) {
        if((msg->node_type == CANBUS_NODETYPE_ECU) && (!msg->payload.empty())){
            union{
            float speed;
            char  buf[4];
            } agv_speed_l,agv_speed_r;
            for (int i = 0; i < 4; i++){
                agv_speed_l.buf[i] = msg->payload[i];
            }
            for (int i = 4; i < 8; i++){
                agv_speed_r.buf[i-4] = msg->payload[i];
            }
            current_vel_l = agv_speed_l.speed;
            current_vel_r = -1 * agv_speed_r.speed;
            // ROS_INFO("rec:vl: %f\n",current_vel_l);
            // ROS_INFO("rec:vr: %f\n",current_vel_r);
        }    
    }

    void Pm1Driver::handle_twist_msg(const geometry_msgs::Twist::ConstPtr &msg) {
        twist_cache_ = *msg;
        if (twist_cache_.linear.x >= 0){
            twist_cache_.linear.x = std::min(twist_cache_.linear.x , max_speed_);
        }
        else{
            twist_cache_.linear.x = std::max(twist_cache_.linear.x , -max_speed_);
        }
        // double baselink_v_anle = tan(twist_cache_.angular.z) * twist_cache_.linear.x / wheelbase;
        double baselink_v_anle = twist_cache_.angular.z;
        double vl = speed_cor * (twist_cache_.linear.x - baselink_v_anle * wheeltread * 0.5);
        double vr = speed_cor * (twist_cache_.linear.x + baselink_v_anle * wheeltread * 0.5);
        can_bus::CanBusService srv;
        srv.request.requests.clear();
        can_bus::CanBusMessage target_speed;
        target_speed.node_type = CANBUS_NODETYPE_ECU;
        target_speed.payload.clear();
        for(int i = 0;i < 8;i++){
            target_speed.payload.push_back(0);   
        }
        int id_l, id_r;
        double ks_l, ks_r;
        for(id_l = 0;id_l < sizeof(pushtable);id_l++){
            if(speedtable[id_l] >= std::fabs(vl)){
                break;
            }
        }
        switch (id_l)
        {
        case 0:  
            ks_l = pushtable[0];
            break;
        case sizeof(pushtable):
            ks_l = pushtable[sizeof(pushtable)-1];
            break;
        default:
            ks_l = pushtable[id_l - 1] + (pushtable[id_l] - pushtable[id_l - 1]) * (std::fabs(vl) - speedtable[id_l - 1]) / (speedtable[id_l] - speedtable[id_l - 1]);
            break;
        }
        // vl = vl / ks_l;
        for(id_r = 0;id_r < 8;id_r++){
            if(speedtable[id_r] >= std::fabs(vr)){
                break;
            }
        }
        switch (id_r)
        {
        case 0:  
            ks_r = pushtable[0];
            break;
        case sizeof(pushtable):
            ks_r = pushtable[sizeof(pushtable)-1];
            break;
        default:
            ks_r = pushtable[id_r - 1] + (pushtable[id_r] - pushtable[id_r - 1]) * (std::fabs(vr) - speedtable[id_r - 1]) / (speedtable[id_r] - speedtable[id_r - 1]);
            break;
        }
        union HEX float_num_l, float_num_r;
        float_num_l.num = vl * 1;
        for(size_t l = 0; l < 4; l++){	//大端模式顺着来0-4，小端模式逆着来4-0
            target_speed.payload[l] = float_num_l.hex_num[l];	
        }
        float_num_r.num = vr * -1;
        for(size_t r = 0; r < 4; r++){	//大端模式顺着来0-4，小端模式逆着来4-0
            target_speed.payload[r+4] = float_num_r.hex_num[r];	
        }
        srv.request.requests.push_back(target_speed);
        canbus_client_.call(srv);             
    }

    void Pm1Driver::parse_msg(const ros::TimerEvent &){
        ros::Time now = ros::Time::now();
            if (first_send_odom_flag_) {
                last_send_odom_time_ = now;
                accumulation_x_ = 0.0;
                accumulation_y_ = 0.0;
                accumulation_yaw_ = 0.0;
                last_a = 0.0;
                last_v = 0.0;
                first_send_odom_flag_ = false;
            } else {
                double current_vel = (current_vel_l + current_vel_r) * 0.5; 
                double delta_time = (now - last_send_odom_time_).toSec();
                // ROS_INFO("delta_time %f\n",delta_time);
                double delta_dis = current_vel * delta_time;
                double w = (current_vel_r- current_vel_l) / wheeltread;
                double delta_theta = w * delta_time;
                double deltaX, deltaY;
                if (delta_theta == 0) {
                    deltaX = delta_dis;
                    deltaY = 0.0;
                } else {
                    deltaX = delta_dis * (sin(delta_theta) / delta_theta);
                    deltaY = delta_dis * ((1 - cos(delta_theta)) / delta_theta);
                }
                accumulation_x_ += (cos(accumulation_yaw_) * deltaX - sin(accumulation_yaw_) * deltaY);
                accumulation_y_ += (sin(accumulation_yaw_) * deltaX + cos(accumulation_yaw_) * deltaY);
                accumulation_yaw_ += delta_theta;
                tf2::Quaternion q;
                q.setRPY(0, 0, accumulation_yaw_);


                if (publish_tf_) {
                    geometry_msgs::TransformStamped transform_stamped;
                    transform_stamped.header.stamp = now;
                    transform_stamped.header.frame_id = odom_frame_;
                    transform_stamped.child_frame_id = base_frame_;
                    transform_stamped.transform.translation.x = accumulation_x_;
                    transform_stamped.transform.translation.y = accumulation_y_;
                    transform_stamped.transform.translation.z = 0.0;
                    transform_stamped.transform.rotation.x = q.x();
                    transform_stamped.transform.rotation.y = q.y();
                    transform_stamped.transform.rotation.z = q.z();
                    transform_stamped.transform.rotation.w = q.w();
                    br_.sendTransform(transform_stamped);
                }

                nav_msgs::Odometry odom_msg;
                odom_msg.header.frame_id = odom_frame_;
                odom_msg.child_frame_id = base_frame_;
                odom_msg.header.stamp = now;
                odom_msg.pose.pose.position.x = accumulation_x_;
                odom_msg.pose.pose.position.y = accumulation_y_;
                odom_msg.pose.pose.position.z = 0;
                odom_msg.pose.pose.orientation.x = q.getX();
                odom_msg.pose.pose.orientation.y = q.getY();
                odom_msg.pose.pose.orientation.z = q.getZ();
                odom_msg.pose.pose.orientation.w = q.getW();
                odom_msg.twist.twist.linear.x = delta_dis / delta_time;
                odom_msg.twist.twist.linear.y = 0;
                odom_msg.twist.twist.angular.z = delta_theta / delta_time;
                odom_pub_.publish(odom_msg);
                last_send_odom_time_ = now;
            }
    }

    void Pm1Driver::run() {
        ros::NodeHandle node;
        ros::NodeHandle private_node("~");
        private_node.param<std::string>("odom_frame", odom_frame_, std::string("odom"));
        private_node.param<std::string>("base_frame", base_frame_, std::string("base_link"));
        private_node.param<int>("rate", rate_, 10);
        private_node.param<float>("speed_cor", speed_cor, 1);
        private_node.param<double>("max_speed", max_speed_, 2.0);
        private_node.param<bool>("publish_tf", publish_tf_, true);
        private_node.param<float>("wheelbase", wheelbase, 0.34);
        private_node.param<float>("wheeltread", wheeltread, 0.568);
        odom_pub_ = node.advertise<nav_msgs::Odometry>("/odom", 10);
        canbus_msg_subscriber_ = node.subscribe("/canbus_msg", 100, &Pm1Driver::handle_canbus_msg, this);
        twist_subscriber_ = node.subscribe("cmd_vel", 10, &Pm1Driver::handle_twist_msg, this);
        canbus_client_ = node.serviceClient<can_bus::CanBusService>("canbus_server");
        ros::Timer odom_timer = node.createTimer(ros::Duration(1.0 / rate_), &Pm1Driver::parse_msg, this);
        ros::spin();
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "pm1_driver");
    autolabor_driver::Pm1Driver driver;
    driver.run();
    return 0;
}