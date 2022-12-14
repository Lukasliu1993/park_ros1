/*
 * This file is part of wjlidar_c16 driver.
 *
 * The driver is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The driver is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the driver.  If not, see <http://www.gnu.org/licenses/>.
 */


#include "wanji_driver/input.h"
#include <iomanip>
#include <fstream> 
#include <iostream>
using namespace std;
//extern volatile sig_atomic_t flag;
namespace wanji_driver
{
////////////////////////////////////////////////////////////////////////
// Input base class implementation
////////////////////////////////////////////////////////////////////////

/** @brief constructor
 *
 *  @param private_nh ROS private handle for calling node.
 *  @param port UDP port number.
 */
Input::Input(ros::NodeHandle private_nh, uint16_t port) : private_nh_(private_nh), port_(port)
{
  private_nh.param("device_ip", devip_str_, std::string(""));
   private_nh.param("data_length", packet_size, 1001);
  if (!devip_str_.empty())
    ROS_INFO_STREAM("Only accepting packets from IP address: " << devip_str_);
}

////////////////////////////////////////////////////////////////////////
// InputSocket class implementation
////////////////////////////////////////////////////////////////////////

/** @brief constructor
   *
   *  @param private_nh ROS private handle for calling node.
   *  @param port UDP port number
*/
InputSocket::InputSocket(ros::NodeHandle private_nh, uint16_t port) : Input(private_nh, port)
{
  sockfd_ = -1;
  npackets = 120;
  if (!devip_str_.empty())
  {
    inet_aton(devip_str_.c_str(), &devip_);
  }
  
  requestBaseParam(private_nh);


  ROS_INFO_STREAM("Opening UDP socket: port " << port);
  sockfd_ = socket(PF_INET, SOCK_DGRAM, 0);
  if (sockfd_ == -1)
  {
    perror("socket");  // TODO: ROS_ERROR errno
    return;
  }

  int opt = 1;
  if (setsockopt(sockfd_, SOL_SOCKET, SO_REUSEADDR, (const void*)&opt, sizeof(opt)))
  {
    perror("setsockopt error!\n");
    return;
  }

  sockaddr_in my_addr;                   // my address information
  memset(&my_addr, 0, sizeof(my_addr));  // initialize to zeros
  my_addr.sin_family = AF_INET;          // host byte order
  my_addr.sin_port = htons(port);        // port in network byte order
  my_addr.sin_addr.s_addr = INADDR_ANY;  // automatically fill in my IP

  if (bind(sockfd_, (sockaddr*)&my_addr, sizeof(sockaddr)) == -1)
  {
    perror("bind");  // TODO: ROS_ERROR errno
    return;
  }
  

  if (fcntl(sockfd_, F_SETFL, O_NONBLOCK | FASYNC) < 0)
  {
    perror("non-block");
    return;
  }

}

/** @brief destructor */
InputSocket::~InputSocket(void)
{
  (void)close(sockfd_);
}

/** @brief recv  Request thread. */
int InputSocket::requestBaseParam(ros::NodeHandle private_nh)
{
  int sockfd = socket(PF_INET, SOCK_DGRAM, 0);
  if (sockfd == -1)
  {
    perror("socket");  // TODO: ROS_ERROR errno
    return -1;
  }

  int opt = 1;
  //必须加SO_BROADCAST，否则sendto会显示Permission denied
  if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR/*|SO_BROADCAST*/, (const void*)&opt, sizeof(opt)))
  {
    perror("setsockopt error!\n");
  }

  sockaddr_in my_addr;                   // my address information
  memset(&my_addr, 0, sizeof(my_addr));  // initialize to zeros
  my_addr.sin_family = AF_INET;          // host byte order
  my_addr.sin_port = htons(3001);        // port in network byte order
  my_addr.sin_addr.s_addr = INADDR_ANY;//inet_addr("192.168.2.88");

  if (bind(sockfd, (sockaddr*)&my_addr, sizeof(sockaddr)) == -1)
  {
    perror("bind");  // TODO: ROS_ERROR errno
    return -1;
  }
  sockaddr_in remote_addr;                                    // remote address information
  memset(&remote_addr, 0, sizeof(remote_addr));               // initialize to zeros
  remote_addr.sin_family = AF_INET;                           // host byte order
  remote_addr.sin_port = htons(3333);                         // port in network byte order
  remote_addr.sin_addr.s_addr = inet_addr(devip_str_.c_str());// automatically fill in remote IP

  //create recv thread
  gotRequest=0;
  boost::shared_ptr<boost::thread> recvThreadHandle;
  recvThreadHandle.reset(new boost::thread(boost::bind(&InputSocket::recvThread,this,private_nh,sockfd)));
  ros::Rate loop_rate1(10);//10hz - 100ms
  loop_rate1.sleep();

  for(int i=0;i<3;i++)
  {
    //request sn
    requestDevBaseParam(sockfd);
    loop_rate1.sleep();

    //request sn
    requestSN(sockfd);
    loop_rate1.sleep();

    //request vertical angle resolution
    requestVerAngleResolution(sockfd);
    loop_rate1.sleep();
  }

  //block to wait request reply
  ros::Rate loop_rate(100);//100hz - 10ms
  for(int i=0;i<1100;i++)//11s
  {
    if(gotRequest == 3)
    {
      break;
    }
    loop_rate.sleep();
  }
  close(sockfd);

  fprintf(stderr,"got Request reply:%d\n" ,gotRequest);
  
  return 0;
}

int InputSocket::recvThread(ros::NodeHandle &private_nh,int &sockfd)
{
  // ros::Rate loop_rate(1000);//1000hz - 1ms
  int got_sn=0;
  char sn[20]={0};
  int got_angle=0;
  int got_base_param=0;
  uint8_t rbuf[1500]={0};

  //for(int i=0;i<10000;i++)//10s
  while(1)
  {
    if(gotRequest>=3)
      break;
    sockaddr_in sender_address;
    socklen_t sender_address_len = sizeof(sender_address);
    ssize_t nbytes = recvfrom(sockfd, rbuf, sizeof(rbuf), 0, (sockaddr*)&sender_address, &sender_address_len);

    // if(nbytes!=1001 && nbytes!=-1)
    //   fprintf(stderr,"nbytes ** =%d\n",nbytes);
  
    if(rbuf[22]==0x04 && rbuf[23]==0x01)//sn
    {
      rbuf[22]=0x00;
      rbuf[23]=0x00;
      if(got_sn==0)
      {      
        got_sn=1;
        gotRequest++;

        memcpy(sn,&rbuf[26],16);
        sn[16]=0;
        fprintf(stderr,"got sn=%s\n",sn);
      }
    }
    else if(rbuf[22]==0x05 && rbuf[23]==0x14)//vertical angle 
    {
      rbuf[22]=0x00;
      rbuf[23]=0x00;
      if(got_angle==0)
      {
        got_angle=1;
        gotRequest++;
        fprintf(stderr,"got vertical angle resolution\n");

        float verAngle[19]={0};
        float recvVerAngle[16]={0};
        for(int j=0;j<16;j++)
          recvVerAngle[j] = (rbuf[30+j*4]<<24|rbuf[29+j*4]<<16|rbuf[28+j*4]<<8|rbuf[27+j*4])*1.0/1000;
        //set vertical angle from 16lines to 19lines
        verAngle[0] = verAngle[5] =  verAngle[9] = verAngle[14] = recvVerAngle[7];
        verAngle[1] = recvVerAngle[0];
        verAngle[2] = recvVerAngle[1];
        verAngle[3] = recvVerAngle[2];
        verAngle[4] = recvVerAngle[3];

        verAngle[6] = recvVerAngle[4];
        verAngle[7] = recvVerAngle[5];
        verAngle[8] = recvVerAngle[6];
        
        verAngle[10] = recvVerAngle[8];
        verAngle[11] = recvVerAngle[9];
        verAngle[12] = recvVerAngle[10];
        verAngle[13] = recvVerAngle[11];

        verAngle[15] = recvVerAngle[12];
        verAngle[16] = recvVerAngle[13];
        verAngle[17] = recvVerAngle[14];
        verAngle[18] = recvVerAngle[15];
        
        // for(int i=0;i<19;i++)
        // {
        //   fprintf(stderr,"%02d %.3f\r\n",i+1,verAngle[i]);
        // }

        //save vertical angle resolution to file
        std::string anglePath;
        private_nh.getParam("calibration",anglePath);
        ofstream file;
        file.open(anglePath.c_str());
        if (!file)
        {
          ROS_WARN_STREAM("[cloud][rawdata] " << anglePath << " does not exist");
          fprintf(stderr,"%s\n",anglePath.c_str());
        }
        else
        {
          for(int j=0;j<19;j++)
            file<<fixed<<setprecision(3)<<verAngle[j]<<",0"<<std::endl;
        }
        file.close();

        //set flag to tell pointcloud to get new calibration file
        ros::NodeHandle node;//使用公共节点
        node.setParam("calibration_update","1");


        //save vertical angle resolution to a backup file with sn
        if(sn[0]==0)
          memcpy(sn,"unknow",6);
        size_t pos=anglePath.find(".csv");
        anglePath.insert(pos,sn);
        // fprintf(stderr,"%s\n",anglePath.c_str());
        file.open(anglePath.c_str());
        if (!file)
        {
          ROS_WARN_STREAM("[cloud][rawdata] " << anglePath << " does not exist");
          fprintf(stderr,"%s\n",anglePath.c_str());
        }
        else
        {
          for(int j=0;j<19;j++)
            file<<fixed<<setprecision(3)<<verAngle[j]<<",0"<<std::endl;
        }
        file.close();

        break;

      }
    }
    else if(rbuf[22]==0x05 && rbuf[23]==0x01)//dev base param
    {
      rbuf[22]=0x00;
      rbuf[23]=0x00;
      if(got_base_param==0)
      {
        got_base_param=1;
        gotRequest++;
        fprintf(stderr,"got base param\n");

        int scan_res = (rbuf[27]+1)*10;//0:0.1;1:0.2...->10,20,30,40
        int echo_mode = rbuf[33];
        int protocol_mode = rbuf[39];
        int protocol_compatible = rbuf[40];
        npackets = 0;

        if(protocol_mode == 0)//720f
        {
          if(echo_mode >= 0x30)//dual echo wave:6 dualblocks,every block 20
          {//blocks:6*2 lines:19
            npackets=36000/scan_res/6;//0.2:300
          }
          else//single echo wave
          {
            if(protocol_compatible == 1) //720f-single 0xFFEE
            {//blocks:15 lines:19
              npackets=36000/scan_res/15;//0.2:120
            }
            else //720c-single 0xFFDD
            {
              npackets=36000/scan_res/12;
            }
          }
        }
        else if(protocol_mode == 3)//720c
        {
          if(echo_mode >= 0x30)//dual echo wave:9 dualblocks,every block 20
          {//blocks:6*2 lines:19
            npackets=36000/scan_res/9;//0.2:200
          }
          else
          {//blocks:6*2 lines:19
            npackets=36000/scan_res/18;//0.2:100
          }
        }
        fprintf(stderr,"npackets:%d\n",npackets);

      }
    }
    // else      continue;

    usleep(100);
    static unsigned int count=0;
    count++;
    if(count>=10*1000*10)//10s
      break;
    //loop_rate.sleep();//usleep(10000);
  }

  // fprintf(stderr,"exit thread\n");
  return 0;
}

void InputSocket::refreshPackets(int &packs,unsigned char *pbuf,int rpm)
{
  if(packs != npackets)
  {
    if(pbuf[0] == 0xFF && pbuf[1] == 0xDD)
    {
      npackets = rpm / 30 * (pbuf[4]);
      npackets = 36000 / npackets;
      packs = npackets;
      fprintf(stderr,"refreshPackets1:%d channel:%d npacket:%d rpm:%d\n",npackets,pbuf[3],pbuf[4],rpm);
    }
    else if(pbuf[0] == 0xFF && pbuf[1] == 0xEE)
    {
      npackets = 72000 / rpm;
      packs = npackets;
      fprintf(stderr,"refreshPackets2:%d rpm:%d\n",npackets,rpm);
    }

    packs = npackets;
    fprintf(stderr,"refreshPackets:%d\n",npackets);
  }
}

void InputSocket::resleepPackets(int rpm)
{

}
/** @brief send a Request to get device base param. */
int InputSocket::requestDevBaseParam(int &sockfd)
{
  sockaddr_in remote_addr;                                    // remote address information
  memset(&remote_addr, 0, sizeof(remote_addr));               // initialize to zeros
  remote_addr.sin_family = AF_INET;                           // host byte order
  remote_addr.sin_port = htons(3333);                         // port in network byte order
  remote_addr.sin_addr.s_addr = inet_addr(devip_str_.c_str());// automatically fill in remote IP
  uint8_t sbuf[34]=//{0};
    {0xff,0xAA,0x00,0x1E,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x00,0x0B,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x05,0x01};
  uint8_t check=0x00;//用于保存异或结果
  int i;
  for (i=2;i<sizeof(sbuf)-4;i++)
    check ^= sbuf[i];
  sbuf[31]  = check;//check_bit
  sbuf[32]  = 0xEE;//frame_tail
  sbuf[33]  = 0xEE;//frame_tail
  if(sendto(sockfd, sbuf, sizeof(sbuf) ,0,(sockaddr*)&remote_addr, sizeof(remote_addr)) == -1)
  {
    perror("sendto request vertical angle resolution");
    close(sockfd);
    return -1;
  }
  return 0;
}

/** @brief send a Request to get  SN num. */
int InputSocket::requestSN(int &sockfd)
{
  sockaddr_in remote_addr;                                    // remote address information
  memset(&remote_addr, 0, sizeof(remote_addr));               // initialize to zeros
  remote_addr.sin_family = AF_INET;                           // host byte order
  remote_addr.sin_port = htons(3333);                         // port in network byte order
  remote_addr.sin_addr.s_addr = inet_addr(devip_str_.c_str());// automatically fill in remote IP
  uint8_t sbuf[34]=//{0};
    {0xff,0xAA,0x00,0x1E,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x00,0x0B,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x01};
  uint8_t check=0x00;//用于保存异或结果
  int i;
  for (i=2;i<sizeof(sbuf)-4;i++)
    check ^= sbuf[i];
  sbuf[31]  = check;//check_bit
  sbuf[32]  = 0xEE;//frame_tail
  sbuf[33]  = 0xEE;//frame_tail
  if(sendto(sockfd, sbuf, sizeof(sbuf) ,0,(sockaddr*)&remote_addr, sizeof(remote_addr)) == -1)
  {
    perror("sendto request vertical angle resolution");
    close(sockfd);
    return -1;
  }
  return 0;
}

/** @brief send a Request to get  vertical angle resolution. */
int InputSocket::requestVerAngleResolution(int &sockfd)
{
  sockaddr_in remote_addr;                                    // remote address information
  memset(&remote_addr, 0, sizeof(remote_addr));               // initialize to zeros
  remote_addr.sin_family = AF_INET;                           // host byte order
  remote_addr.sin_port = htons(3333);                         // port in network byte order
  remote_addr.sin_addr.s_addr = inet_addr(devip_str_.c_str());// automatically fill in remote IP
  uint8_t sbuf[34]=//{0};
    {0xff,0xAA,0x00,0x1E,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x00,0x0B,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x05,0x14};
  uint8_t check=0x00;//用于保存异或结果
  int i;
  for (i=2;i<sizeof(sbuf)-4;i++)
    check ^= sbuf[i];
  sbuf[30]  = 0x00;//check_bit
  sbuf[31]  = check;//check_bit
  sbuf[32]  = 0xEE;//frame_tail
  sbuf[33]  = 0xEE;//frame_tail

  if(sendto(sockfd, sbuf, sizeof(sbuf) ,0,(sockaddr*)&remote_addr, sizeof(remote_addr)) == -1)
  {
    perror("sendto request vertical angle resolution");
    close(sockfd);
    return -1;
  }

  return 0;
}

/** @brief Get one wjlidar packet. */
int InputSocket::getPacket(wanji_msgs::WanjiPacket* pkt, const double time_offset)
{
  double time1 = ros::Time::now().toSec();
  struct pollfd fds[1];
  fds[0].fd = sockfd_;
  fds[0].events = POLLIN;
  static const int POLL_TIMEOUT = 1000;  // one second (in msec)

  sockaddr_in sender_address;
  socklen_t sender_address_len = sizeof(sender_address);
  //while (flag == 1)
  while (true)
  {
    // Receive packets that should now be available from the
    // socket using a blocking read.
    // poll() until input available
    do
    {
      int retval = poll(fds, 1, POLL_TIMEOUT);
      if (retval < 0)  // poll() error?
      {
        if (errno != EINTR)
          ROS_ERROR("poll() error: %s", strerror(errno));
        return 1;
      }
      if (retval == 0)  // poll() timeout?
      {
        ROS_WARN_THROTTLE(2, "wjlidar poll() timeout");
        return 1;
      }
      if ((fds[0].revents & POLLERR) || (fds[0].revents & POLLHUP) || (fds[0].revents & POLLNVAL))  // device error?
      {
        ROS_ERROR("poll() reports wjlidar error");
        return 1;
      }
    } while ((fds[0].revents & POLLIN) == 0);
    ssize_t nbytes = recvfrom(sockfd_, &pkt->data[0], BUFF_SIZE, 0, (sockaddr*)&sender_address, &sender_address_len);
    if (nbytes < 0)
    {
      if (errno != EWOULDBLOCK)
      {
        perror("recvfail");
        ROS_INFO("recvfail");
        return 1;
      }
    }
    else if ((pkt->data[0]==0xFF && pkt->data[1]==0xEE) || (pkt->data[0]==0xFF && pkt->data[1]==0xDD))//judge scan data
    {
      // fprintf(stderr,"nbytes %d\n",nbytes);
      // if (devip_str_ != "" && sender_address.sin_addr.s_addr != devip_.s_addr)
      //   continue;
      // else
        break;  // done
    }

    ROS_DEBUG_STREAM("incomplete wjlidar packet read: " << nbytes << " bytes");
  }
  // Average the times at which we begin and end reading.  Use that to
  // estimate when the scan occurred. Add the time offset.
  double time2 = ros::Time::now().toSec();
  pkt->stamp = ros::Time((time2 + time1) / 2.0 + time_offset);

  return 0;
}

////////////////////////////////////////////////////////////////////////
// InputPCAP class implementation
////////////////////////////////////////////////////////////////////////

/** @brief constructor
   *
   *  @param private_nh ROS private handle for calling node.
   *  @param port UDP port number
   *  @param packet_rate expected device packet frequency (Hz)
   *  @param filename PCAP dump file name
   */
InputPCAP::InputPCAP(ros::NodeHandle private_nh, uint16_t port, double packet_rate, std::string filename,
                     bool read_once, bool read_fast, double repeat_delay)
  : Input(private_nh, port), packet_rate_(3000), filename_(filename)
{
  pcap_ = NULL;
  empty_ = true;
  npackets = 120;

  // get parameters using private node handle
  private_nh.param("read_once", read_once_, false);
  private_nh.param("read_fast", read_fast_, true);
  private_nh.param("repeat_delay", repeat_delay_, 0.0);

  if (read_once_)
    ROS_INFO("Read input file only once.");
  if (read_fast_)
    ROS_INFO("Read input file as quickly as possible.");
  if (repeat_delay_ > 0.0)
    ROS_INFO("Delay %.3f seconds before repeating input file.", repeat_delay_);

  // Open the PCAP dump file
  // ROS_INFO("Opening PCAP file \"%s\"", filename_.c_str());
  ROS_INFO_STREAM("Opening PCAP file " << filename_);
  fprintf(stderr,"rePCAP file:%s\n",filename_.c_str());
  if ((pcap_ = pcap_open_offline(filename_.c_str(), errbuf_)) == NULL)
  {
    fprintf(stderr,"Error opening wjlidar socket dump file\n");
    ROS_FATAL("Error opening wjlidar socket dump file.");
    return;
  }

  std::stringstream filter;
  if (devip_str_ != "")  // using specific IP?
  {
    filter << "src host " << devip_str_ << " && ";
  }
  filter << "udp dst port " << port;
  pcap_compile(pcap_, &pcap_packet_filter_, filter.str().c_str(), 1, PCAP_NETMASK_UNKNOWN);
}

/** destructor */
InputPCAP::~InputPCAP(void)
{
  pcap_close(pcap_);
}

/** @brief Get one wjlidar packet. */
int InputPCAP::getPacket(wanji_msgs::WanjiPacket* pkt, const double time_offset)
{
  struct pcap_pkthdr* header;
  const u_char* pkt_data;
 // while (flag == 1)
  while (true)
  {
    int res;
    if ((res = pcap_next_ex(pcap_, &header, &pkt_data)) >= 0)
    {
        int ress=pcap_offline_filter(&pcap_packet_filter_, header, pkt_data);
        // fprintf(stderr,"ress:%d\n",ress);
      // Skip packets not for the correct port and from the
      // selected IP address.
      if (!devip_str_.empty() && (0 ==ress))
        continue;
     
      // Keep the reader from blowing through the file.
      if (read_fast_ == false)
        packet_rate_.sleep();
      
      //judge scan data
      if (pkt_data[42]==0xFF && (pkt_data[43]==0xEE || pkt_data[43]==0xDD) && header->caplen==header->len)
      {
        int pack_len = header->caplen;
        memcpy(&pkt->data[0], pkt_data + 42, pack_len);
      }
      else 
      {
        fprintf(stderr,"error pcap len%d\n", header->caplen);
        continue;
      }
        

      
      pkt->stamp = ros::Time::now();  // time_offset not considered here, as no
                                      // synchronization required
      empty_ = false;
      return 0;  // success
    }

    if (empty_)  // no data in file?
    {
      fprintf(stderr,"empty_\n");
      ROS_WARN("Error %d reading wjlidar packet: %s", res, pcap_geterr(pcap_));
      return -1;
    }

    if (read_once_)
    {
         fprintf(stderr,"read_once_\n");
      ROS_INFO("end of file reached -- done reading.");
      return -1;
    }

    if (repeat_delay_ > 0.0)
    {
      ROS_INFO("end of file reached -- delaying %.3f seconds.", repeat_delay_);
      usleep(rint(repeat_delay_ * 1000000.0));
    }

    ROS_DEBUG("replaying wjlidar dump file");

    // I can't figure out how to rewind the file, because it
    // starts with some kind of header.  So, close the file
    // and reopen it with pcap.
    pcap_close(pcap_);
    pcap_ = pcap_open_offline(filename_.c_str(), errbuf_);
    empty_ = true;  // maybe the file disappeared?
  }                 // loop back and try again
}

void InputPCAP::refreshPackets(int &packs,unsigned char *pbuf,int rpm)
{
  if(packs != npackets)
  {
    if(pbuf[0] == 0xFF && pbuf[1] == 0xDD)
    {
      npackets = rpm / 30 * (pbuf[4]);
      npackets = 36000 / npackets;
      packs = npackets;
      fprintf(stderr,"refreshPackets1:%d channel:%d npacket:%d rpm:%d\n",npackets,pbuf[3],pbuf[4],rpm);
    }
    else if(pbuf[0] == 0xFF && pbuf[1] == 0xEE)
    {
      npackets = 72000 / rpm;
      packs = npackets;
      fprintf(stderr,"refreshPackets2:%d rpm:%d\n",npackets,rpm);
    }
    
  }
}

void InputPCAP::resleepPackets(int rpm)
{
  if(npackets != 0)
  {
    float l_ti = 1 / (60.0 / rpm - npackets / 3000.0) + 0.5;
    ros::Rate loop_rate(l_ti);
    loop_rate.sleep();
  }
}
}
