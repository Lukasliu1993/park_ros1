

#define IMU102N_HEAD1		0X5A
#define IMU102N_HEAD2		0X5A
#define IMU102NDATA_LEN		59	
#define IMU102N_CRCINDEX	58

struct imu102n
{
	union {
		float 	fGx;
		int32_t int_gx;
	}Gx;
	union {
		float 	fGy;
		int32_t int_gy;
	}Gy;	
	union {
		float 	fGz;
		int32_t int_gz;
	}Gz;
	
	union {
		float 	fAx;
		int32_t int_ax;
	}Ax;
	union {
		float 	fAy;
		int32_t int_ay;
	}Ay;
	union {
		float 	fAz;
		int32_t int_az;
	}Az;
	
	int tem;
}imu102n;

static struct imu_txbuf
{
	uint8_t imu_head[2];
	uint8_t id[2];
	uint8_t len;
	uint8_t data[32];	
	uint8_t chcksum[2];
}imu_txbuf;

static int8_t imu102n_crc( uint8_t * data)
{
	uint8_t crc = 0;
	uint8_t i = 2;
	uint8_t ret = 0;
	for(i = 2; i<IMU102N_CRCINDEX; i++){
		crc += data[i];
	}

	if(data[IMU102N_CRCINDEX] ==  crc){
		ret = 1;
	}
	else{
		ret = -1;
	}
	return ret;
}

// 验证IMU102N数据的正确性
// 参数 data数组中存放依次从IMU102N中接收到的16进制数据
uint8_t cal_102n_val(uint8_t * data)
{
    uint8_t* p = data;
	uint8_t ret = 0;
	// 校验
	if( imu102n_crc(p) != 1){
		printf("crc error.\r\n");
		return  -1;
	}
	if(data[0] == IMU102N_HEAD1 && data[1] == IMU102N_HEAD2)
	{	
		//速度 正负200m/s 范围
		imu102n.Gx.int_gx = 0;
		imu102n.Gy.int_gy = 0;
		imu102n.Gz.int_gz = 0;
		imu102n.Gx.int_gx = data[5]  << 24  | data[4]  << 16 | data[3]  << 8  | data[2] << 0;
		imu102n.Gy.int_gy = data[9]  << 24  | data[8]  << 16 | data[7]  << 8  | data[6] << 0;
		imu102n.Gz.int_gz = data[13] << 24  | data[12] << 16 | data[11] << 8  | data[10] << 0;
		printf(" Gx = %f, Gy = %f, Gz = %f \r\n",imu102n.Gx.fGx, imu102n.Gy.fGy, imu102n.Gz.fGz);

		// 加速度 正负18g 范围
		imu102n.Ax.int_ax = 0;
		imu102n.Ay.int_ay = 0;
		imu102n.Az.int_az = 0;
		imu102n.Ax.int_ax = data[17] << 24  | data[16] << 16 | data[15] << 8  | data[14] << 0;
		imu102n.Ay.int_ay = data[21] << 24  | data[20] << 16 | data[19] << 8  | data[18] << 0;
		imu102n.Az.int_az = data[25] << 24  | data[24] << 16 | data[23] << 8  | data[22] << 0;
		printf(" Ax = %f, Ay = %f, Az = %f \r\n",imu102n.Ax.fAx, imu102n.Ay.fAy, imu102n.Az.fAz);
	}
}

