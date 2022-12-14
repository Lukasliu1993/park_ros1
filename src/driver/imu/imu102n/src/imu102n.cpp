#include <unistd.h>

#include "ros/ros.h"
#include "serial/serial.h"
#include "sensor_msgs/Imu.h"
#include "tf2/LinearMath/Quaternion.h"

#define DL_HEADER1                        0x5a
#define DL_HEADER2                        0x5a
#define DL_CLASS_MINIAHRS                 0x0f
#define DL_MINIAHRS_ATTITUDE_AND_SENSORS  0x01

#define DL_PAYLOAD_LENGTH                 56
#define DL_CHECK_LENGTH                   57
#define DL_CHECKSUM_LENGTH                0x02

#define DL_NO_ERR                         0x00
#define DL_UNKNOW_MESSAGE                 0x01
#define DL_CHECKSUM_ERR                   0x02
#define DL_PAYLOAD_LENGTH_ERR             0x04

#define COEF_DEG_TO_RAD                   57.29578
#define g                                 9.8
typedef struct imu102n
{
  union{
    float fGx;
    int32_t int_gx;
  }Gx;
    union{
    float fGy;
    int32_t int_gy;
  }Gy;
    union{
    float fGz;
    int32_t int_gz;
  }Gz;
    union{
    float fAx;
    int32_t int_ax;
  }Ax;
    union{
    float fAy;
    int32_t int_ay;
  }Ay;
    union{
    float fAz;
    int32_t int_az;
  }Az;
}imu102n_t;
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

uint8_t convert_byte(uint8_t * ptr){
  uint8_t f;
  memcpy(&f, ptr, 1);
  return f;
}

bool imu_checksum(uint8_t * payload){
  uint8_t checksum = 0;
  for (int i=0; i<DL_PAYLOAD_LENGTH; i++){
    checksum += payload[i];
  }
  return checksum == payload[DL_CHECK_LENGTH-1];
}
float convert_float(uint8_t * ptr){
  float f;
  memcpy(&f, ptr, 4);
  return f;
}


void convert_to_msg(sensor_msgs::Imu& msg, uint8_t * payload){
  imu102n_t imu_bd;
  msg.header.frame_id = "imu_link";
  msg.header.stamp = ros::Time::now();

  msg.orientation.x = 0;
  msg.orientation.y = 0;
  msg.orientation.z = 0;
  msg.orientation.w = 1;
  // msg.orientation_covariance[0] = -1.0;
  imu_bd.Gx.int_gx = payload[3] << 24 | payload[2] << 16 | payload[1] << 8 | payload[0] << 0;
  imu_bd.Gy.int_gy = payload[7] << 24 | payload[6] << 16 | payload[5] << 8 | payload[4] << 0;
  imu_bd.Gz.int_gz = payload[11] << 24 | payload[10] << 16 | payload[9] << 8 | payload[8] << 0;
  imu_bd.Ax.int_ax = payload[15] << 24 | payload[14] << 16 | payload[13] << 8 | payload[12] << 0;
  imu_bd.Ay.int_ay = payload[19] << 24 | payload[18] << 16 | payload[17] << 8 | payload[16] << 0;
  imu_bd.Az.int_az = payload[23] << 24 | payload[22] << 16 | payload[21] << 8 | payload[20] << 0;
  msg.angular_velocity.x = (double)imu_bd.Gx.fGx / COEF_DEG_TO_RAD;
  msg.angular_velocity.y = (double)imu_bd.Gy.fGy / COEF_DEG_TO_RAD;
  msg.angular_velocity.z = (double)imu_bd.Gz.fGz / COEF_DEG_TO_RAD;

  msg.linear_acceleration.x = (double)imu_bd.Ax.fAx * g;
  msg.linear_acceleration.y = (double)imu_bd.Ay.fAy * g;
  msg.linear_acceleration.z = (double)imu_bd.Az.fAz * g;
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
        size_t read_payload_size = serial.read(payload, (int)DL_CHECK_LENGTH);
        state = read_payload_size == DL_CHECK_LENGTH ? 3 : 0;
        break;
      }case 3:{ // CHECKSUM
        state = imu_checksum(payload) ? 4 : 0;
        break;
      }case 4:{
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
  private_node.param<int>("baud", baud, 230400);
  serial::Serial serial(port, baud, serial::Timeout::simpleTimeout(5000));
  ros::Publisher publisher = nh.advertise<sensor_msgs::Imu>("imu", 1);
  ros::Rate loop_rate(300);
  ros::Time now = ros::Time::now();
  while (ros::ok()){
    sensor_msgs::Imu msg;
    uint8_t payload[DL_CHECK_LENGTH];
    fetch_payload(serial, payload);
    convert_to_msg(msg, payload);
    if(!started && (msg.header.stamp.sec - now.sec) < 5 ){
      refe = msg;
      num ++;
      started = true;
    }
    else if(started && (msg.header.stamp.sec - now.sec) < 5){
      refe.angular_velocity.x = (refe.angular_velocity.x + msg.angular_velocity.x);
      refe.angular_velocity.y = (refe.angular_velocity.y + msg.angular_velocity.y);
      refe.angular_velocity.z = (refe.angular_velocity.z + msg.angular_velocity.z);
      refe.linear_acceleration.x = (refe.linear_acceleration.x + msg.linear_acceleration.x);
      refe.linear_acceleration.y = (refe.linear_acceleration.y + msg.linear_acceleration.y);
      refe.linear_acceleration.z = (refe.linear_acceleration.z + msg.linear_acceleration.z);
      num ++;
    }
    else if(started && (msg.header.stamp.sec - now.sec) >= 5){
    msg.angular_velocity.x = msg.angular_velocity.x - refe.angular_velocity.x / num;
    msg.angular_velocity.y = msg.angular_velocity.y - refe.angular_velocity.y / num;
    msg.angular_velocity.z = msg.angular_velocity.z - refe.angular_velocity.z / num;
    msg.linear_acceleration.x = msg.linear_acceleration.x - refe.linear_acceleration.x / num;
    msg.linear_acceleration.y = msg.linear_acceleration.y - refe.linear_acceleration.y / num;
    if(fabs(msg.angular_velocity.x * COEF_DEG_TO_RAD) < 450 && fabs(msg.angular_velocity.y * COEF_DEG_TO_RAD) < 450 && fabs(msg.angular_velocity.z * COEF_DEG_TO_RAD) < 450
     && fabs(msg.linear_acceleration.x / g) < 16 && fabs(msg.linear_acceleration.y / g) < 16 && fabs(msg.linear_acceleration.z / g) < 16 && fabs(msg.linear_acceleration.z / g) > 0.001){
      publisher.publish(msg);
      }
    }
    loop_rate.sleep();
  }
  serial.close();
  return 0;
}
