#ifndef IMU102N_H
#define IMU102N_H
/***************************************************************
Copyright@ BDStar Navigation Co.,Ltd.. 1998-2029. All rights reserved.
文件名		: imu102nreg.h
作者	  	: 李彦东
版本	   	: V1.0
描述	   	: IMU102N寄存器地址描述头文件
其他	   	: 无
日志	   	: 初版V1.0 2020/01/13 李彦东创建
***************************************************************/
#define IMU102N_ID	 0X66	/* ID值 */


#define TEMP_L	     0x000C	 
#define TEMP_H       0x000E

#define X_GYRO_L     0x0010
#define X_GYRO_H     0x0012
#define Y_GYRO_L     0x0014
#define Y_GYRO_H     0x0016           
#define Z_GYRO_L     0x0018	
#define Z_GYRO_H     0x001A

#define X_ACCL_L     0x001C
#define X_ACCL_H     0x001E
#define Y_ACCL_L     0x0020
#define Y_ACCL_H     0x0022
#define Z_ACCL_L     0x0024
#define Z_ACCL_H     0x0026

/*
#define X_MAGN_OUT     0x2800		// 
#define Y_MAGN_OUT     0x2A00       // 
#define Z_MAGN_OUT     0x2C00       // 

#define BAROM_LOW      0x2E00       // 
#define BAROM_OUT      0x3000       // 
*/

#define PROD_ID        0x007E



#endif

