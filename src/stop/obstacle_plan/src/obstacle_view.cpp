#include <obstacle_plan/obstacle_view.h>
namespace dtail_simulation  {
    Obstacleview::Obstacleview() {};

    void Obstacleview::obs_callback(const autoware_msgs::DetectedObjectArray::ConstPtr &msg) {
        bool tag = true;
        if(model == 1){
            for (auto const &object: msg->objects){
                float_t ox = object.pose.position.x;
                float_t oy = object.pose.position.y;
                float_t od = sqrt((object.pose.position.x * object.pose.position.x) +
                                        (object.pose.position.y * object.pose.position.y));
                float_t ov = sqrt((object.velocity.linear.x * object.velocity.linear.x ) +
                                        (object.velocity.linear.y * object.velocity.linear.y));
                // if(od < max_dis && ox > 0 && ov > min_v && atan(fabs(oy) / (fabs(ox) + r)) < min_ang * M_PI / 180 ){
                if(ox > x_min && ov > min_v && fabs(oy) < len && fabs(ox) < x_max){
                    tag = false;
                    start = msg->header.stamp;
                    stop = msg->header.stamp; 
                    if(!pause_ ){
                        pause_ = true;
                        std_msgs::Bool data_push;
                        data_push.data = true;
                        pause_pub_.publish(data_push); 
                        // ros::param::set("stop_switch",1); 
                        break;
                    }
                }
            } 
            if(tag){
                start = msg->header.stamp;
            }
            if((start.sec - stop.sec) > min_time){
                if(pause_ ){
                    std_msgs::Bool data_push;
                    data_push.data = false;
                    pause_pub_.publish(data_push);  
                    // ros::param::set("stop_switch",0); 
                    pause_ = false;
                }
            }
        }
        if(model == 2){
            for (auto const &object: msg->objects){
                float_t ox = object.pose.position.x;
                float_t oy = object.pose.position.y;
                float_t od = sqrt((object.pose.position.x * object.pose.position.x) +
                                        (object.pose.position.y * object.pose.position.y));
                float_t ov = sqrt((object.velocity.linear.x * object.velocity.linear.x ) +
                                        (object.velocity.linear.y * object.velocity.linear.y));
                // if(od < max_dis && ov > min_v && atan(fabs(oy) / (fabs(ox) + r)) < min_ang * M_PI / 180 ){
                if(ov > min_v && fabs(oy) < len && fabs(ox) < x_max && fabs(ox) > x_min){
                    tag = false;
                    start = msg->header.stamp;
                    stop = msg->header.stamp; 
                    if(!pause_ ){
                        pause_ = true;
                        std_msgs::Bool data_push;
                        data_push.data = true;
                        pause_pub_.publish(data_push); 
                        // ros::param::set("stop_switch",1);  
                        break;
                    }
                }
            } 
            if(tag){
                start = msg->header.stamp;
            }
            if((start.sec - stop.sec) > min_time){
                if(pause_ ){
                    std_msgs::Bool data_push;
                    data_push.data = false;
                    pause_pub_.publish(data_push);  
                    // ros::param::set("stop_switch",0); 
                    pause_ = false;
                }
            }
        }
    }



    void Obstacleview::run() {
        ros::NodeHandle private_node("~");
        private_node.param<std::string>("sub_topic", sub_topic, "/detection/object_tracker/objects");
        private_node.param<std::string>("pub_topic", pub_topic, "/pause_msg");
        private_node.param<double>("max_dis", max_dis, 2.0);
        private_node.param<int>("model", model, 1);
        private_node.param<double>("min_v", min_v, 0.1);
        private_node.param<double>("min_time", min_time, 3);
        private_node.param<double>("min_ang", min_ang, 30);
        private_node.param<double>("len", len, 0.6);
        private_node.param<double>("high", high, 0.8);
        private_node.param<double>("x_min", x_min, 0.0);
        private_node.param<double>("x_max", x_max, 0.8);
        r = (len / 2) / tan(min_ang * M_PI /180);
        ROS_WARN_STREAM("started view obstacle");
        ros::param::set("stop_switch",0); 
        pause_ = false;
        ob_sub_ = nh_.subscribe<autoware_msgs::DetectedObjectArray>(sub_topic,10,&Obstacleview::obs_callback,this);
        pause_pub_ = nh_.advertise<std_msgs::Bool>(pub_topic, 100);
        ros::spin();
    }
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "pause_pub");
    dtail_simulation::Obstacleview Obstack_view;
    Obstack_view.run();
    return 0;
}