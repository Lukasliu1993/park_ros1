#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
#include <chasu/dzt_p1_service.h>
#include <stdlib.h>
#include "chasu/cmd2.h"
#define RX_WAIT_TIME  0
#define RX_BUFF_SIZE  1






namespace autolabor_driver {

    CanbusDriver::CanbusDriver() {}

    CanbusDriver::~CanbusDriver() {
        closedev();
    }
    bool CanbusDriver::verify_frame(CAN_OBJ *can,chasu::CanBusMessage *msg)
    {      
        if (can->DataLen > 8) return 0;  // error: data length
        switch (can->ID) {
            case 0x181:
            msg->node_type = 1;
            msg->payload.clear();
            for (int i = 0; i < 8; i++) {
                msg->payload.push_back(can->Data[i]);
            }
            break;
            default:
            return 0;
            break;
        }
        return 1;
    }
    int CanbusDriver::init() {
        bool rec = false;
        while(!rec) {
            startdev();
            ros::Time t1 = ros::Time::now();
            ros::Time t2 = ros::Time::now();
            bool rec = false;
            while((t2.sec  - t1.sec ) < 2) {
                CAN_OBJ can1[1];
                int cnt1;  // current received
                cnt1 = Receive(gDevType1, gDevIdx1, gChMask1, can1, 1, 2);
                if (cnt1 == 1) {
                    rec = true;
                    break;
                }
                t2 = ros::Time::now();
            }
            if(rec) {
                return true;
            } else {    
            }
        }
    }

    bool CanbusDriver::trans_msg(int dev, int num, int ID, int externflag, vector <unsigned char> & data){
        CAN_OBJ can;
        memset(&can, 0, sizeof(CAN_OBJ));
        can.ID = ID;
        can.SendType = 0;
        can.ExternFlag = externflag;
        can.RemoteFlag = 0;
        can.DataLen = 8;  
        for (int i = 0; i < 8; i++) {
            can.Data[i] = data[i];
        }
        if (Transmit(dev, num, 0, &can, 1) == 0) {
            ROS_INFO("transmit failed\n");
            return false;
        }
        return true;
    }

    void CanbusDriver::closedev(){
        if (!ResetCAN(gDevType1, gDevIdx1, 0)) {
            ROS_INFO("ResetCAN1 failed\n");
        }
        if (!CloseDevice(gDevType1, gDevIdx1)) {
            ROS_INFO("CloseCAN1 failed\n");
        }
    }
    bool CanbusDriver::opendev(){
        while (!OpenDevice(gDevType1, gDevIdx1, 0)) {
            ROS_INFO("gDevType1%d\n", gDevType1);
            ROS_INFO("gDevIdx1%d\n", gDevIdx1);
            ROS_INFO("OpenDevice1 failed\n");
            if (!ResetCAN(gDevType1, gDevIdx1, 0)) {
                ROS_INFO("ResetCAN1 failed\n");
            }
            if (!CloseDevice(gDevType1, gDevIdx1)) {
                ROS_INFO("CloseCAN1 failed\n");
            }
        }
        INIT_CONFIG config;
        config.AccCode = 0;
        config.AccMask = 0xffffff;
        config.Filter = 0;
        config.Mode = 0;
        config.Timing0 = gBaud1 & 0xff;
        config.Timing1 = gBaud1 >> 8;
        if (!InitCAN(gDevType1, gDevIdx1, 0, &config)) {
            ROS_INFO("InitDevice1 failed\n");
            return false;
        }
        if (!StartCAN(gDevType1, gDevIdx1, 0)) {
            ROS_INFO("StartCAN1 failed\n");
            return false;
        }
        return true;
    }

    void CanbusDriver::startdev(){
        closedev();
        bool state = opendev();
        while(!state) {
            state = opendev();
        }
    }


    void CanbusDriver::parse_msg1() {
        while (true) {
            CAN_OBJ can[RX_BUFF_SIZE];  // buffer
            int cnt;                    // current received
            int i;
            cnt = Receive(gDevType1, gDevIdx1, gChMask1, can, RX_BUFF_SIZE, RX_WAIT_TIME);
            if (cnt == 0xFFFFFFFF || cnt != RX_BUFF_SIZE) {
            continue;
            }
            for (i = 0; i < cnt; i++) {
                chasu::CanBusMessage msg;
                if (verify_frame(&can[i], &msg)) {
                    canbus_msg_pub1_.publish(msg);
                    if (enable_debug_) {
                        chasu::cmd2 c_msg;
                        c_msg.header.stamp = ros::Time::now();
                        c_msg.node_type = msg.node_type;
                        c_msg.payload = msg.payload;
                        if (c_msg.node_type == 1) {
                            qianjin_rec.publish(c_msg);
                        }
                    }
                } else {
                }
            }
        }
    }

    bool CanbusDriver::canbus_service(chasu::CanBusService::Request &req, chasu::CanBusService::Response &res) {
        if (req.requests.size() > 0) 
        {
            for (int i = 0; i < 1; i++) 
            {
                chasu::CanBusMessage msg = req.requests[i];
                CAN_OBJ can;
                memset(&can, 0, sizeof(CAN_OBJ));
                int is_data = 0;
                switch (msg.node_type) 
                {
                    case 1:
                    if (trans_msg(gDevType1, gDevIdx1, 0x201, 0, msg.payload) == false) 
                    {
                        ROS_INFO("TYPE1 transmit failed\n");
                    } 
                    if (enable_debug_) {
                        chasu::cmd2 c_msg;
                        c_msg.header.stamp = ros::Time::now();
                        c_msg.node_type = msg.node_type;
                        c_msg.payload = msg.payload;
                        qianjin_pub.publish(c_msg);
                    }   
                    break;
                    case 2:
                    if (trans_msg(gDevType1, gDevIdx1, 0x6F, 0, msg.payload) == false) 
                    {
                        ROS_INFO("TYPE1 transmit failed\n");
                    } 
                    if (enable_debug_) {
                        chasu::cmd2 c_msg;
                        c_msg.header.stamp = ros::Time::now();
                        c_msg.node_type = msg.node_type;
                        c_msg.payload = msg.payload;
                        odom_pub.publish(c_msg);
                    }   
                    break;
                    default:
                    break;
                }
            } 
            return true;
        } else {
            return false;
        }
    }

    void CanbusDriver::run() {
        ros::NodeHandle node;
        ros::NodeHandle private_node("~");
        private_node.param<int>("gDevType1", gDevType1, 3);
        private_node.param<int>("gDevIdx1", gDevIdx1, 0);
        private_node.param<int>("gChMask1", gChMask1, 0);
        private_node.param<int>("gBaud1", gBaud1, 7168);
        private_node.param<int>("gTxType1", gTxType1, 0);
        private_node.param<bool>("enable_debug", enable_debug_, false);
        if (init()) {
            ROS_INFO(" start ok\n");
            qianjin_pub = node.advertise<chasu::cmd2>("/qianjin_pub", 10);
            jiaodu_pub = node.advertise<chasu::cmd2>("/jiaodu_pub", 10);
            qianjin_rec = node.advertise<chasu::cmd2>("/qianjin_rec", 10);
            jiaodu_rec = node.advertise<chasu::cmd2>("/jiaodu_rec", 10);
            brush_pub = node.advertise<chasu::cmd2>("/brush_pub", 10);
            odom_pub = node.advertise<chasu::cmd2>("/odom_pub", 10);
            canbus_msg_pub1_ = node.advertise<chasu::CanBusMessage>("canbus_msg", 10);
            canbus_msg_service_ = node.advertiseService("canbus_server", &CanbusDriver::canbus_service, this);
            boost::thread parse_thread1(boost::bind(&CanbusDriver::parse_msg1, this));
        }
        ros::spin(); 
    }
};


int main(int argc, char **argv) {
    ros::init(argc, argv, "autolabor_canbus_driver");
    autolabor_driver::CanbusDriver driver;
    driver.run();
    return 0;
}