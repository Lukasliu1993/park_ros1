#include <unistd.h>
#include <algorithm>
#include "ros/ros.h"
#include <vector>
#include "serial/serial.h"
#include "nav_msgs/Odometry.h"
#include <tf2/LinearMath/Quaternion.h>
using namespace std;


namespace gps_driver {
    class gps_serial {
        public:
            gps_serial();
            ~gps_serial() = default;
            void run();
            void fetch_payload(serial::Serial& serial, uint8_t* payload);
            bool check_eq(serial::Serial& serial, uint8_t num);
            uint32_t crc32(const uint8_t *data, const int size);
            double convert_double(uint8_t * ptr, u_int num);
            void convert_to_msg(nav_msgs::Odometry& msg, uint8_t * payload);
        private:
            ros::Publisher gps_pub_;
            int baud, rate;
            string port;
    };
}

