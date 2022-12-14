#include <unistd.h>

#include "ros/ros.h"
#include "serial/serial.h"
#include "sensor_msgs/Imu.h"
#include "tf2/LinearMath/Quaternion.h"

#define DL_HEADER1                        0xAA
#define DL_HEADER2                        0x55
#define DL_HEADER3                        0x26
#define DL_HEADER4                        0x00


#define COEF_DEG_TO_RAD                   180 / 3.141592653589793
#define g                                 9.8

union HEX {
  int16_t num;
  unsigned char hex_num[2];
};

using namespace std;


bool check_eq(serial::Serial& serial, uint8_t num){
  uint8_t buffer;
  serial.read(&buffer, 1);
  if (buffer == num){
    return true;
  }else{
    return false;
  }
}


bool imu_checksum(uint8_t * payload){
  unsigned char arr[4] = {0xAA, 0x55, 0x26, 0x00};
  int checksum = 0;
  for (int i=0; i<4; i=i+2){
    checksum += (arr[i+1] << 8) | arr[i];
  }
  for (int i=0; i<32; i=i+2){
    checksum += (payload[i+1] << 8) | payload[i];
  }
  checksum = checksum&0xffff;
  return checksum == ((payload[33] << 8) | payload[32]);
}



void convert_to_msg(sensor_msgs::Imu& msg, uint8_t * payload){
  msg.header.frame_id = "imu_link";
  msg.header.stamp = ros::Time::now();
  msg.orientation.x = 0;
  msg.orientation.y = 0;
  msg.orientation.z = 0;
  msg.orientation.w = 1;
  union HEX data_hex;
  data_hex.hex_num[0] = payload[12];
  data_hex.hex_num[1] = payload[13];
  msg.linear_acceleration.x = (double) g * data_hex.num  / (32768 * 0.0625);
  data_hex.hex_num[0] = payload[14];
  data_hex.hex_num[1] = payload[15];
  msg.linear_acceleration.y = (double) g * data_hex.num  / (32768 * 0.0625);
  data_hex.hex_num[0] = payload[16];
  data_hex.hex_num[1] = payload[17];
  ROS_INFO("hex_num[0] %d\n", payload[16]);
  ROS_INFO("hex_num[1] %d\n", payload[17]);
  ROS_INFO("num %d\n", data_hex.num);
  msg.linear_acceleration.z = (double) g * data_hex.num  / (32768 * 0.0625);
  data_hex.hex_num[0] = payload[24];
  data_hex.hex_num[1] = payload[25];
  msg.angular_velocity.x = (double) data_hex.num  / (32768 * (3.141592653589793 / 5760) * COEF_DEG_TO_RAD);
  data_hex.hex_num[0] = payload[26];
  data_hex.hex_num[1] = payload[27];
  msg.angular_velocity.y = (double) data_hex.num  / (32768 * (3.141592653589793 / 5760) * COEF_DEG_TO_RAD);
  data_hex.hex_num[0] = payload[28];
  data_hex.hex_num[1] = payload[29];
  msg.angular_velocity.z = (double) data_hex.num  / (32768 * (3.141592653589793 / 5760)) * COEF_DEG_TO_RAD;
}

void fetch_payload(serial::Serial& serial, uint8_t* payload){
  unsigned char state = 0;
  if (serial.isOpen()){
    while(1){
      switch (state){
      case 0:{ // Header 1
        state = check_eq(serial, DL_HEADER1) ? 1 : 0;
        break;
      }case 1:{ // Header 2
        state = check_eq(serial, DL_HEADER2) ? 2 : 0;
        break;
      }case 2:{ // PAYLOAD
        state = check_eq(serial, DL_HEADER3) ? 3 : 0;
        break;
      }case 3:{ // PAYLOAD
        state = check_eq(serial, DL_HEADER4) ? 4 : 0;
        break;
      }case 4:{ // PAYLOAD
        ROS_INFO("header4\n");
        size_t read_payload_size = serial.read(payload, 34);
        state = read_payload_size == 34 ? 5 : 0;
        break;
      }case 5:{ // CHECKSUM
        ROS_INFO("header5\n");
        state = imu_checksum(payload) ? 6 : 0;
        break;
      }case 6:{
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

int main(int argc, char **argv)
{
  // Set up ROS.
  int baud;
  sensor_msgs::Imu refe;
  bool started = false;
  int num = 0;
  string port;
  ros::init(argc, argv, "imu102n");
  ros::NodeHandle nh;
  ros::NodeHandle private_node("~");
  private_node.param<string>("port", port, "/dev/ttyUSB0");
  private_node.param<int>("baud", baud, 460800);
  serial::Serial serial(port, baud, serial::Timeout::simpleTimeout(5000));
  ros::Publisher publisher = nh.advertise<sensor_msgs::Imu>("imu", 1);
  ros::Rate loop_rate(200);
  ros::Time now = ros::Time::now();
  while (ros::ok()){
    sensor_msgs::Imu msg;
    uint8_t payload[34];
    fetch_payload(serial, payload);
    convert_to_msg(msg, payload);
    publisher.publish(msg);
    // if(!started && (msg.header.stamp.sec - now.sec) < 5 ){
    //   refe = msg;
    //   num ++;
    //   started = true;
    // }
    // else if(started && (msg.header.stamp.sec - now.sec) < 5){
    //   refe.angular_velocity.x = (refe.angular_velocity.x + msg.angular_velocity.x);
    //   refe.angular_velocity.y = (refe.angular_velocity.y + msg.angular_velocity.y);
    //   refe.angular_velocity.z = (refe.angular_velocity.z + msg.angular_velocity.z);
    //   refe.linear_acceleration.x = (refe.linear_acceleration.x + msg.linear_acceleration.x);
    //   refe.linear_acceleration.y = (refe.linear_acceleration.y + msg.linear_acceleration.y);
    //   refe.linear_acceleration.z = (refe.linear_acceleration.z + msg.linear_acceleration.z);
    //   num ++;
    // }
    // else if(started && (msg.header.stamp.sec - now.sec) >= 5){
    // msg.angular_velocity.x = msg.angular_velocity.x - refe.angular_velocity.x / num;
    // msg.angular_velocity.y = msg.angular_velocity.y - refe.angular_velocity.y / num;
    // msg.angular_velocity.z = msg.angular_velocity.z - refe.angular_velocity.z / num;
    // msg.linear_acceleration.x = msg.linear_acceleration.x - refe.linear_acceleration.x / num;
    // msg.linear_acceleration.y = msg.linear_acceleration.y - refe.linear_acceleration.y / num;
    // if(fabs(msg.angular_velocity.x * COEF_DEG_TO_RAD) < 450 && fabs(msg.angular_velocity.y * COEF_DEG_TO_RAD) < 450 && fabs(msg.angular_velocity.z * COEF_DEG_TO_RAD) < 450
    //  && fabs(msg.linear_acceleration.x / g) < 16 && fabs(msg.linear_acceleration.y / g) < 16 && fabs(msg.linear_acceleration.z / g) < 16 && fabs(msg.linear_acceleration.z / g) > 0.001){
    //   publisher.publish(msg);
    //   }
    // }
    loop_rate.sleep();
  }
  serial.close();
  return 0;
}
