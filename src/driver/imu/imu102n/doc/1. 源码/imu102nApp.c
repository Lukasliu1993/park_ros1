#include "stdio.h"
#include "unistd.h"
#include "sys/types.h"
#include "sys/stat.h"
#include "sys/ioctl.h"
#include "fcntl.h"
#include "stdlib.h"
#include "string.h"
#include <poll.h>
#include <sys/select.h>
#include <sys/time.h>
#include <signal.h>
#include <fcntl.h>
#include "linux/ioctl.h"

#include "poll.h"
#include "sys/select.h"

/*********************************************************************
Copyright  © BDStar Navigation Co.,Ltd. 1998-2029. All rights reserved.
文件名		: imu102nApp.c
作者	  	: 李彦东
版本	   	: V1.0
描述	   	: icm20608设备测试APP。
使用方法	 ：./imu102nApp /dev/imu102n
**********************************************************************/

//char *filename = "/dev/imu102n";

struct IMU_data {
	int IMUID;
	short temp_low;				/* 温度低地址原始值 		*/
	short temp_high;			/* 温度高地址原始值 		*/

	short gyro_x_low;  			/* 陀螺仪X轴低地址原始值 	 */
	short gyro_x_high;   		/* 陀螺仪X轴高地址原始值 	 */
	short gyro_y_low;    	
	short gyro_y_high;   	
	short gyro_z_low;    	
	short gyro_z_high;   	

	short accel_x_low;   		/* 加速度计X轴低地址原始值 	*/
	short accel_x_high;  		/* 加速度计X轴高地址原始值 	*/
	short accel_y_low;   	
	short accel_y_high;  	
	short accel_z_low;   	
	short accel_z_high;  
	
	float temp_act;
	float gyro_x_act, gyro_y_act, gyro_z_act;
	float accel_x_act, accel_y_act, accel_z_act;
};

double calcu_gyro(short low, short high)
{
	// 把hight计算在内是为了提高精度
//	return 0.02*low + ((high>>15)*0.01) + ((high>>14)*0.01/2) + ((high>>13)*0.01/4) + ((high>> 12)*0.01/8) + ((high>>11)*0.01/16) + ((high>> 10)*0.01/32);
	
	return 0.02*(double)low;
}

double calcu_accel(short low,short high)
{
	return  (long)(low *65536+ high) *0.00001220703125*0.001;
}

static void cal_data(unsigned short rxbuf[])
{
	static struct IMU_data imudata;

	imudata.temp_low = rxbuf[0];
	imudata.temp_high = rxbuf[1];

	imudata.gyro_x_low = rxbuf[2];
	imudata.gyro_x_high = rxbuf[3];
	imudata.gyro_y_low = rxbuf[4];
	imudata.gyro_y_high = rxbuf[5];
	imudata.gyro_z_low = rxbuf[6];
	imudata.gyro_z_high = rxbuf[7];

	imudata.accel_x_low = rxbuf[8];
	imudata.accel_x_high = rxbuf[9];
	imudata.accel_y_low = rxbuf[10];
	imudata.accel_y_high = rxbuf[11];
	imudata.accel_z_low = rxbuf[12];
	imudata.accel_z_high = rxbuf[13];
	imudata.IMUID = rxbuf[14];
	
	/* 计算实际值 */
	imudata.temp_act = (float)(25 +  imudata.temp_low * 0.00565);
						
	imudata.gyro_x_act = calcu_gyro(imudata.gyro_x_low,  imudata.gyro_x_high);
	imudata.gyro_y_act = calcu_gyro(imudata.gyro_y_low,  imudata.gyro_y_high);
	imudata.gyro_z_act = calcu_gyro(imudata.gyro_z_low,  imudata.gyro_z_high);
						
	imudata.accel_x_act = calcu_accel(imudata.accel_x_low, imudata.accel_x_high);
	imudata.accel_y_act = calcu_accel(imudata.accel_y_low, imudata.accel_y_high);
	imudata.accel_z_act = calcu_accel(imudata.accel_z_low, imudata.accel_z_high);
/*
	printf("\r\n原始值:\r\n");
	printf("temp_low = %d, temp_high = %d\r\n", imudata.temp_low, imudata.temp_high);
	printf("gx %d, %d \r\n", imudata.gyro_x_low,  imudata.gyro_x_high);
	printf("gy %d, %d \r\n", imudata.gyro_y_low,  imudata.gyro_y_high);
	printf("ax %d, %d \r\n", imudata.accel_x_low, imudata.accel_x_high);
*/		
	printf("实际值:");
	printf("temp = %.3f°C\r\n", imudata.temp_act);
	printf("gx = %.3f°/S, gy = %.3f°/S, gz = %.3f°/S\r\n", imudata.gyro_x_act, imudata.gyro_y_act, imudata.gyro_z_act);
	printf("ax = %.3fg, ay = %.3fg, az = %.3fg\r\n", imudata.accel_x_act, imudata.accel_y_act, imudata.accel_z_act);
	printf("102n ID: %x\r\n",imudata.IMUID  );
}

/*
 * @description		: main主程序
 * @param - argc 	: argv数组元素个数
 * @param - argv 	: 具体参数
 * @return 			: 0 成功;其他 失败
 */
int main(int argc, char *argv[])
{
	int fd;
	int ret = 0;
	fd_set readfds;
	char *filename;
	struct pollfd fds;

	struct timeval timeout;

	unsigned short rxbuf[15];

	if (argc != 2) {
		printf("Error Usage!\r\n");
		printf("eg: ./imu102nApp /dev/imu102n \r\n");
		return -1;
	}

	filename = argv[1];
	fd = open(filename, O_RDWR);
	//fd = open(filename, O_RDWR | O_NONBLOCK);
	if(fd < 0) {
		printf("can't open file %s\r\n", filename);
		return -1;
	}

#if 1  // 阻塞访问
	while (1) 
	{	
		ret = read(fd, rxbuf, sizeof(rxbuf));
		if(ret == 0) 	/* 数据读取成功 */
		{ 					
			 cal_data(rxbuf);
		}
		usleep(500000); /*500ms */
	}
#endif

#if 0 		// 非阻塞访问
	
	fds.fd = fd;
	fds.events = POLLIN;

	while (1) 
	{
		ret = poll(&fds, 1, 500);
		if (ret) 				/* 数据有效 */
		{	
			ret = read(fd, rxbuf, sizeof(rxbuf));
			if(ret < 0) 	/* 读取错误 */
				printf("poll return error ...\r\n");
			else 
				cal_data(rxbuf);	
		} 
		else if (ret == 0) 		/* 超时 */
		{	
			printf("poll time out...\r\n");
		} 
		else if (ret < 0) 		/* 错误 */
		{	
			printf("poll return error ...\r\n");
		}
	}
#endif

	
#if 0		// 非阻塞访问
	while (1) 
	{ 
		FD_ZERO(&readfds);
		FD_SET(fd, &readfds);
		/* 构造超时时间 */
		timeout.tv_sec = 0;
		timeout.tv_usec = 500000; /* 500ms */
		ret = select(fd + 1, &readfds, NULL, NULL, &timeout);  // 实现非阻塞访问
		switch (ret) {
			case 0: 		/* 超时 */
				printf("select time out...\r\n");
				break;
			case -1:		/* 错误 */
				printf("select return error ...\r\n");
				break;
			default:  		/* 可以读取数据 */
				if(FD_ISSET(fd, &readfds)) 
				{
					ret = read(fd, rxbuf, sizeof(rxbuf));
					if(ret < 0) 	/* 读取错误 */
						printf("poll return error ...\r\n");
					else 
						cal_data(rxbuf);	
					
				}
				break;
		}	
	}
#endif

	close(fd);	/* 关闭文件 */	
	return 0;
}

