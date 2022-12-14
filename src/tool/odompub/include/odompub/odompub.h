
#include <ros/ros.h>
#include <serial/serial.h>
#include <nav_msgs/Odometry.h>
namespace dzt {
    class odompub {
        public:
            odompub();

            ~odompub() = default;

            void run();

        private:

            void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);

            void pubTimer(const ros::TimerEvent &);

            uint32_t crc32(const uint8_t *data, const int size);
        private:
            serial::Serial serial_;
            std::string port_;
            int32_t odom_cache_;
            ros::Subscriber sub;
            ros::Timer odom_timer;
            int baud_, rate_;
            uint8_t payload[36] = {0xaa, 0x44, 0x13, 0x14, 0xdd, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
                                    0x00, 0x00, 0x00, 0x00};
    };
}


