#include <odompub/odompub.h>
namespace dzt {
    union HEX {
        uint32_t num;
        unsigned char hex_num[4];
    };
    union HEX2 {
        int32_t num;
        unsigned char hex_num[4];
    };
    odompub::odompub(){
    }
    void odompub::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        odom_cache_ = int32_t(msg->twist.twist.linear.x * 1000);
    }
    uint32_t odompub::crc32(const uint8_t *data, const int size) {
        uint32_t crc = 0;
        for (int i = 0; i < size; i++) {
            crc ^= data[i];
            for (int j = 0; j < 8; j++) {
                if (crc & 1) {
                    crc = (crc >> 1) ^ 0xedb88320u;
                } else {
                    crc >>= 1;
                }
            }
        }
        return crc;
    }
    void odompub::pubTimer(const ros::TimerEvent &)
    {
        union HEX2 data_send;
        data_send.num = odom_cache_;
        payload[12] = data_send.hex_num[0];
        payload[13] = data_send.hex_num[1];
        payload[14] = data_send.hex_num[2];
        payload[15] = data_send.hex_num[3];
        union HEX data_check;
        data_check.num = crc32(payload, 32);
        payload[32] = data_check.hex_num[0];
        payload[33] = data_check.hex_num[1];
        payload[34] = data_check.hex_num[2];
        payload[35] = data_check.hex_num[3];
        // serial_.open();
        // printf("open3: %d\n", serial_.isOpen());
        serial_.flushInput();
        serial_.write(payload, sizeof(payload));
        // RCLCPP_INFO(this->get_logger(), "!!!!!!!!!!!!!!!!!\n");
    }
    void odompub::run() {
        ros::NodeHandle node;
        ros::NodeHandle private_node("~");
        private_node.param<std::string>("port", port_, "/dev/ttyUSB0");
        private_node.param<int>("baud", baud_, 115200);
        private_node.param<int>("rate", rate_, 50);
        serial_.setPort(port_);
        serial_.setBaudrate(baud_);
        serial::Timeout to = serial::Timeout::simpleTimeout(5000);
        serial_.setTimeout(to);
        // printf("open1: %d\n", serial_.isOpen());
        serial_.open();
        // printf("open2: %d\n", serial_.isOpen());
        sub = node.subscribe("/odom", 100, &odompub::odomCallback, this);
        ros::Timer odom_timer = node.createTimer(ros::Duration(1.0 / rate_), &odompub::pubTimer, this);
        ros::spin();
    }
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "odompub");
    dzt::odompub driver;
    driver.run();
    return 0;
}
