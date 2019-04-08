#ifndef MPU9250_H
#define MPU9250_H

#define MPU6500_ADDRESS 0xD0   //AD0接GND时地址为0x68，接VCC时地址为0x69
#define MPU_PWR_MGMT1_REG		0X6B	//电源管理寄存器1
#define MPU_GYRO_CFG_REG		0X1B	//陀螺仪配置寄存器
#define MPU_ACCEL_CFG_REG		0X1C	//加速度计配置寄存器
#define MPU_ACCEL_DLPF_DERG	0x1D	//加速度低通寄存器
#define MPU_SAMPLE_RATE_REG		0X19	//陀螺仪采样频率分频器
#define MPU_INT_EN_REG			0X38	//中断使能寄存器
#define MPU_USER_CTRL_REG		0X6A	//用户控制寄存器
#define MPU_FIFO_EN_REG			0X23	//FIFO使能寄存器
#define MPU_INTBP_CFG_REG		0X37	//中断/旁路设置寄存器
#define MPU_DEVICE_ID_REG		0X75	//器件ID寄存器
#define MPU_PWR_MGMT2_REG		0X6C	//电源管理寄存器2
#define MPU_CFG_REG				0X1A	//配置寄存器 低通滤波器配置寄存器

#define MPU_TEMP_OUTH_REG		0X41	//温度值高8位寄存器
#define MPU_TEMP_OUTL_REG		0X42	//温度值低8位寄存器

#define MPU_ACCEL_XOUTH_REG		0X3B	//加速度值，X轴高8位寄存器
#define MPU_ACCEL_XOUTL_REG		0X3C	//加速度值，X轴低8位寄存器
#define MPU_ACCEL_YOUTH_REG		0X3D	//加速度值，Y轴高8位寄存器
#define MPU_ACCEL_YOUTL_REG		0X3E	//加速度值，Y轴低8位寄存器
#define MPU_ACCEL_ZOUTH_REG		0X3F	//加速度值，Z轴高8位寄存器
#define MPU_ACCEL_ZOUTL_REG		0X40	//加速度值，Z轴低8位寄存器

#define MPU_GYRO_XOUTH_REG		0X43	//陀螺仪值，X轴高8位寄存器
#define MPU_GYRO_XOUTL_REG		0X44	//陀螺仪值，X轴低8位寄存器
#define MPU_GYRO_YOUTH_REG		0X45	//陀螺仪值，Y轴高8位寄存器
#define MPU_GYRO_YOUTL_REG		0X46	//陀螺仪值，Y轴低8位寄存器
#define MPU_GYRO_ZOUTH_REG		0X47	//陀螺仪值，Z轴高8位寄存器
#define MPU_GYRO_ZOUTL_REG		0X48	//陀螺仪值，Z轴低8位寄存器

#define MPU_WHO_I_AM	0x75	//陀螺仪ID

#define AK8963_ADDRESS 0x18		//磁力计地址0x0C
#define AK8963_CNTL1		0x0A  //磁力计读取寄存器
#define AK8963_HXL			0x03	//磁力计X轴低8位寄存器
#define AK8963_HXH			0x04	//磁力计X轴高8位寄存器
#define AK8963_HYL			0x05	//磁力计Y轴低8位寄存器
#define AK8963_HYH			0x06	//磁力计Y轴高8位寄存器
#define AK8963_HZL			0x07	//磁力计Z轴低8位寄存器
#define AK8963_HZH			0x08	//磁力计Z轴高8位寄存器
#define AK8963_WHO_I_AM	0x00	//磁力计ID

#define SENSORS_NBR_OF_BIAS_SAMPLES		1024	/*计算方差的采样样本个数 */
#define GYRO_VARIANCE_BASE				4000	/* 陀螺仪零偏方差阈值 */
#define SENSORS_ACC_SCALE_SAMPLES  		200		/* 加速度采样个数 */

struct Axisf
{
	float x;
	float y;
	float z;
};

struct Axisi
{
	int x;
	int y;
	int z;
};

struct MPU6500_t
{
	struct Axisf gyro;
	struct Axisf acc;
	struct Axisf attitude;
};

typedef struct
{
	struct Axisf     bias;
	int       isBiasValueFound;
	int       isBufferFilled;
	struct Axisi   buffer[SENSORS_NBR_OF_BIAS_SAMPLES];
}BiasObj;

extern BiasObj	gyroBiasRunning;;
extern struct MPU6500_t mpu6500;

int MPU6500_Init(void);
int GetImuData(void);
void imuDataHandle(void);
void imuUpdate(struct Axisf acc, struct Axisf gyro);

#endif
