#include <gps_driver/gps_driver.h>
using namespace std;
union HEX {
    uint32_t num;
    unsigned char hex_num[4];
};
namespace gps_driver {

  gps_serial::gps_serial(){
  }
  void gps_serial::run(){
    ros::NodeHandle nh;
    ros::NodeHandle private_node("~");
    private_node.param<std::string>("port", port, "/dev/ttyUSB0");
    private_node.param<int>("baud", baud, 460800);
    private_node.param<int>("rate", rate, 100);
    serial::Serial serial(port, baud, serial::Timeout::simpleTimeout(5000));
    gps_pub_ = nh.advertise<nav_msgs::Odometry>("/gps", 10);
    ros::Rate loop_rate(rate);
    ros::Time now = ros::Time::now();
    while (ros::ok()){
      ROS_INFO("here");
      nav_msgs::Odometry msg;
      uint8_t payload[155];
      fetch_payload(serial, payload);
      convert_to_msg(msg, payload);
      gps_pub_.publish(msg);
      loop_rate.sleep();
    }
    serial.close();
  }

  void gps_serial::fetch_payload(serial::Serial& serial, uint8_t* payload){
    uint8_t payload_[158];
    unsigned char state = 0;
    if (serial.isOpen()){
      while(1){
        switch (state){
          case 0:{
            state = check_eq(serial,  0xAA) ? 1 : 0;
            break;
          }case 1:{ // Header 2
            ROS_INFO("find aa");
            state = check_eq(serial,  0x44) ? 2 : 0;
            break;
          }case 2:{ // Header 2
            ROS_INFO("find 44");
            state = check_eq(serial,  0x12) ? 3 : 0;
            break;
          }case 3:{ // PAYLOAD
            ROS_INFO("find 12");
            size_t read_payload_size = serial.read(payload, (int)155);
            state = read_payload_size == 155 ? 4 : 0;
            break;
          }case 4:{ // PAYLOAD
            ROS_INFO("find payload");
            payload_[0] = 0xAA;
            payload_[1] = 0x44;
            payload_[2] = 0x12;
            for(int i = 0; i < 155; i++){
              payload_[i+3] = payload[i];
            }
            state = crc32(payload_, 158) ? 5 : 0;
            ROS_INFO("state %d \n", state);
            break;
          }
          case 5:{
            state = 0;
            return;
          }default:{
            state = 0;
            break;
          }
        }
      }
    }
  }

  bool gps_serial::check_eq(serial::Serial& serial, uint8_t num){
    uint8_t buffer;
    serial.read(&buffer, 1);
    ROS_INFO("search %d \n", buffer);
    if (buffer == num){
      return true;
    }else{
      return false;
    }
  }
  uint32_t gps_serial::crc32(const uint8_t *data, const int size) {
      uint32_t crc = 0;
      for (int i = 0; i < size - 4; i++) {
          ROS_INFO("i %d \n", i);
          ROS_INFO("data[i] %d \n", data[i]);
          crc ^= data[i];
          for (int j = 0; j < 8; j++) {
              if (crc & 1) {
                  crc = (crc >> 1) ^ 0xedb88320u;
              } else {
                  crc >>= 1;
              }
          }
      }
      union HEX data_check;
      data_check.hex_num[0] = data[size-4];
      ROS_INFO("data[-4] %u \n", data[size-4]);
      ROS_INFO("data_check.hex_num[0] %u \n", data_check.hex_num[0]);
      data_check.hex_num[1] = data[size-3];
      ROS_INFO("data[-3] %u \n", data[size-3]);
      ROS_INFO("data_check.hex_num[1] %u \n", data_check.hex_num[1]);
      data_check.hex_num[2] = data[size-2];
      ROS_INFO("data[-2] %u \n", data[size-2]);
      ROS_INFO("data_check.hex_num[2] %u \n", data_check.hex_num[2]);
      data_check.hex_num[3] = data[size-1];
      ROS_INFO("data[-1] %u \n", data[size-1]);
      ROS_INFO("data_check.hex_num[3] %u \n", data_check.hex_num[3]);
      ROS_INFO("crc %u \n", crc);
      ROS_INFO("data_check.num  %u \n", data_check.num );
      return (data_check.num == crc);
  }
  double gps_serial::convert_double(uint8_t * ptr, u_int num){
    double f;
    memcpy(&f, ptr, num);
    return f;
  }
  void gps_serial::convert_to_msg(nav_msgs::Odometry& msg, uint8_t * payload){
    msg.header.frame_id = "gps_link";
    msg.header.stamp = ros::Time::now();
    msg.pose.pose.position.x = convert_double(payload+33, 8);
    msg.pose.pose.position.y = convert_double(payload+41, 8);
    msg.pose.pose.position.z = convert_double(payload+49, 8);
    tf2::Quaternion q;
    q.setRPY(0, 0, convert_double(payload+101, 8));
    msg.pose.pose.orientation.x = q.getX();
    msg.pose.pose.orientation.y = q.getY();
    msg.pose.pose.orientation.z = q.getZ();
    msg.pose.pose.orientation.w = q.getW();
    union HEX signal_check;
    signal_check.hex_num[0] = payload[145];
    signal_check.hex_num[1] = payload[146];
    signal_check.hex_num[2] = payload[147];
    signal_check.hex_num[3] = payload[148];
    msg.twist.twist.linear.x = payload[29];
    msg.twist.twist.linear.y = signal_check.num;
  }
}
int main(int argc, char **argv) {
    ros::init(argc, argv, "gps_serial");
    gps_driver::gps_serial driver;
    driver.run();
    return 0;
}