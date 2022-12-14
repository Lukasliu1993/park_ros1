#include <fcntl.h>
#include <dirent.h>
#include <linux/input.h>
#include <sys/stat.h>

#include <boost/bind.hpp>
#include <boost/thread.hpp>

#include "dt_keyboard_control/keyboard_control_node.h"

namespace dt_tool {

KeyboardControl::KeyboardControl():linear_state_(0), angular_state_(0), port_name_(""){
  ros::NodeHandle private_node("~");

  private_node.param("linear_min", linear_min_, 0.2);
  private_node.param("linear_max", linear_max_, 2.0);
  private_node.param("linear_step", linear_step_, 0.2);

  private_node.param("angular_min", angular_min_, 0.5);
  private_node.param("angular_max", angular_max_, 4.0);
  private_node.param("angular_step", angular_step_, 0.2);

  private_node.param("smooth_step", smooth_step_, 0.1); //平滑每档速度
  private_node.param("smooth_switch", smooth_switch_, false); //平滑开关
  private_node.param("brush_switch", brush_switch_, false); //清扫刷控制开关
  private_node.param("wash_switch", wash_switch_, false); //清扫刷控制开关
  private_node.param("rate", rate_, 10.0);

  linear_scale_ = linear_min_;
  angular_scale_ = angular_min_;
  send_flag_ = true;
}

KeyboardControl::~KeyboardControl(){
  close(fd_);
}

//开启刷子
void KeyboardControl::brushstart(){
ros::param::set("brush_start_",1);
ROS_INFO_STREAM("开启干刷");
}

//关闭刷子
void KeyboardControl::brushstop(){
ros::param::set("brush_start_",0);
ROS_INFO_STREAM("关闭干刷");
}

void KeyboardControl::washstart(){
ros::param::set("wash_start_",1);
ROS_INFO_STREAM("开启水刷");
}

//关闭刷子
void KeyboardControl::washstop(){
ros::param::set("wash_start_",0);
ROS_INFO_STREAM("关闭水刷");
}
//急停开
void KeyboardControl::emgstop(){
std_msgs::Bool data_push;
data_push.data = true;
stop_pub_.publish(data_push); 
ROS_INFO_STREAM("急停开");
}

//急停关
void KeyboardControl::emgcontinue(){
std_msgs::Bool data_push;
data_push.data = false;
stop_pub_.publish(data_push); 
ROS_INFO_STREAM("急停关");
}

void KeyboardControl::buttonTwistCheck(int value, int& state, int down, int up){
  if (value == 1){
    state += down;
  }else if (value == 0){
    state += up;
  }
}

void KeyboardControl::buttonScaleCheck(int value, double &scale, double step, double limit){
  if (value == 1){
    if (step > 0){
      scale = std::min(scale + step, limit);
    }else{
      scale = std::max(scale + step, limit);
    }
  }
}

void KeyboardControl::parseKeyboard(){
  while (true) {
    read(fd_, &ev_, sizeof(struct input_event));
    if (ev_.type == EV_KEY){
      ROS_DEBUG_STREAM("INFO: [key]: " << ev_.code << ", [value]: " << ev_.value);
      switch (ev_.code) {
      case KEYBOARD_UP:
        buttonTwistCheck(ev_.value, linear_state_, 1, -1);
        break;
      case KEYBOARD_DOWN:
        buttonTwistCheck(ev_.value, linear_state_, -1, 1);
        break;
      case KEYBOARD_LEFT:
        buttonTwistCheck(ev_.value, angular_state_, 1, -1);
        break;
      case KEYBOARD_RIGHT:
        buttonTwistCheck(ev_.value, angular_state_, -1, 1);
        break;
      case BRUSH_START:     //开启刷子
        if(brush_switch_){
          brushstart();
        }
        break;
      case BRUSH_STOP:      //关闭刷子
        if(brush_switch_){
          brushstop();
        }
        break;
      case WASH_START:     //开启刷子
        if(brush_switch_){
          washstart();
        }
        break;
      case WASH_STOP:      //关闭刷子
        if(brush_switch_){
          washstop();
        }
        break;
      case EMERGENCY_STOP:      //急停开
        emgstop();
        break;
      case EMERGENCY_GO:      //急停关
        emgcontinue();
        break;
      case KEYBOARD_1:
        buttonScaleCheck(ev_.value, linear_scale_, linear_step_, linear_max_);
        break;
      case KEYBOARD_2:
        buttonScaleCheck(ev_.value, linear_scale_, -linear_step_, linear_min_);
        break;
      case KEYBOARD_3:
        buttonScaleCheck(ev_.value, angular_scale_, angular_step_, angular_max_);
        break;
      case KEYBOARD_4:
        buttonScaleCheck(ev_.value, angular_scale_, -angular_step_, angular_min_);
        break;
      case KEYBOARD_7:
          brush_switch_ = true;
        break;
      case KEYBOARD_8:
          brush_switch_ = false;
        break;
      case KEYBOARD_9:
        if (ev_.value == 1){
          send_flag_ = true;
        }
        break;
      case KEYBOARD_0:
        if (ev_.value == 1){
          send_flag_ = false;
        }
        break;
      default:
        break;
      }
    }
  }
}

void KeyboardControl::twistCallback(const ros::TimerEvent &){
  if (send_flag_){
    geometry_msgs::Twist twist;
    if (!smooth_switch_)
    {
    twist.linear.x = linear_state_ * linear_scale_;
    twist.angular.z = angular_state_ * angular_scale_;
    twist_pub_.publish(twist);
    ROS_DEBUG_STREAM("linear: " << twist.linear.x << " angular: " << twist.angular.z);
    }
    else
    {
    
    //vel_now_, vel_last_;

    twist.linear.x = linear_state_ * linear_scale_;
    twist.angular.z = angular_state_ * angular_scale_;
    vel_now_ = twist.linear.x;
    vel_last_ = twist.linear.x;
    twist_pub_.publish(twist);
    ROS_DEBUG_STREAM("linear: " << twist.linear.x << " angular: " << twist.angular.z);


    }
  }
}


bool KeyboardControl::init(){
  const char path[] = "/dev/input/by-path";
  DIR *dev_dir = opendir(path);
  struct dirent *entry;
  if (dev_dir == NULL){
    return false;
  }

  while ((entry = readdir(dev_dir)) != NULL){
    std::string dir_str = entry->d_name;
    if (dir_str.find("event-kbd") < dir_str.length()){
      port_name_ = std::string(path) + "/" + dir_str;
      ROS_INFO_STREAM("INFO: The keyboard port is :" << port_name_);
      break;
    }
  }
  closedir(dev_dir);

  if (port_name_ != ""){
    fd_ = open(port_name_.c_str(), O_RDONLY);
    if (fd_ < 0){
      ROS_ERROR_STREAM("ERROR: Can't Open The Port :" << port_name_);
      return false;
    }else{
      ROS_INFO_STREAM("INFO: Open The Port :" << port_name_);
      return true;
    }
  }else{
    return false;
  }
}



void KeyboardControl::run(){
  if (init()){
    twist_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    stop_pub_ = nh_.advertise<std_msgs::Bool>("emgency_stop", 100);
    twist_pub_timer_ = nh_.createTimer(ros::Duration(1.0/rate_), &KeyboardControl::twistCallback, this);
    boost::thread parse_thread(boost::bind(&KeyboardControl::parseKeyboard, this));
    ros::spin();
  }
}
}

int main(int argc, char *argv[]){
  setlocale(LC_ALL,"");
  ros::init(argc, argv, "keyboard_control_node");
  dt_tool::KeyboardControl keyboard_control;
  keyboard_control.run();
  return 0;
}


































