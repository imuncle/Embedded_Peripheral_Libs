#include "mpu9250.h"
#include "i2c.h"
#include <math.h>

#define DEG2RAD		0.017453293f	/*度转弧度*/
#define RAD2DEG		57.29578f		/*弧度转度*/

struct MPU9250_t mpu9250;

float Kp = 2.0f;		/*比例增益*/
float Ki = 0.1f;		/*积分增益*/
float exInt = 0.0f;
float eyInt = 0.0f;
float ezInt = 0.0f;		/*积分误差累计*/

static float q0 = 1.0f;	/*四元数*/
static float q1 = 0.0f;
static float q2 = 0.0f;
static float q3 = 0.0f;

int yaw,pitch,roll;

volatile uint32_t last_update, now_update;

void MPU9250_Init(void)
{
	unsigned char pdata;
	//检查设备是否准备好
	HAL_I2C_IsDeviceReady(&hi2c1, MPU9250_ADDRESS, 10, HAL_MAX_DELAY);
	//检查总线是否准备好
	HAL_I2C_GetState(&hi2c1);

	pdata=0x80; //复位MPU
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, MPU_PWR_MGMT1_REG, 1, &pdata, 1, HAL_MAX_DELAY);
	HAL_I2C_IsDeviceReady(&hi2c1, MPU9250_ADDRESS, 10, HAL_MAX_DELAY);
	
	HAL_Delay(500);  //复位后需要等待一段时间，等待芯片复位完成

	pdata=0x01;	//唤醒MPU
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, MPU_PWR_MGMT1_REG, 1, &pdata, 1, HAL_MAX_DELAY);

	pdata=3<<3; //设置量程为2000
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, MPU_GYRO_CFG_REG, 1, &pdata, 1, HAL_MAX_DELAY); 

	pdata=01;	//设置加速度传感器量程±4g
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, MPU_ACCEL_CFG_REG, 1, &pdata, 1, HAL_MAX_DELAY); 

	pdata=0; //陀螺仪采样分频设置
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, MPU_SAMPLE_RATE_REG, 1, &pdata, 1, HAL_MAX_DELAY); 

	pdata=0;	//关闭所有中断
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, MPU_INT_EN_REG, 1, &pdata, 1, HAL_MAX_DELAY); 
	
	pdata=0;	//关闭FIFO
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, MPU_FIFO_EN_REG, 1, &pdata, 1, HAL_MAX_DELAY); 
	
	pdata=0X02;	//设置旁路模式，直接读取AK8963磁力计数据
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, MPU_INTBP_CFG_REG, 1, &pdata, 1, HAL_MAX_DELAY); 
	HAL_Delay(10);	//需要一段延时让磁力计工作
	
	pdata = 4;	//设置MPU9250的数字低通滤波器
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, MPU_CFG_REG, 1, &pdata, 1, HAL_MAX_DELAY);

	pdata=0;	//使能陀螺仪和加速度工作
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, MPU_PWR_MGMT2_REG, 1, &pdata, 1, HAL_MAX_DELAY);
	
	pdata = 0x01;
	HAL_I2C_Mem_Write(&hi2c1, AK8963_ADDRESS, AK8963_CNTL1, 1, &pdata, 1, HAL_MAX_DELAY);
	HAL_Delay(10);
}

void GetImuData(void)
{
	uint8_t imu_data[14]={0};
	uint8_t mag_data[6] = {0};
	uint8_t pdata;
	short accx,accy,accz;
	short gyrox,gyroy,gyroz;
	short magx,magy,magz;
	
	float gyro_sensitivity = 16.384f;
	int acc_sensitivity = 8192;
	
	static short mag_count = 0;
	
	HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDRESS, MPU_ACCEL_XOUTH_REG, 1, imu_data, 14, HAL_MAX_DELAY);	//读取陀螺仪和加速度计的数据
	accx = (imu_data[0]<<8)|imu_data[1];
	accy = (imu_data[2]<<8)|imu_data[3];
	accz = (imu_data[4]<<8)|imu_data[5];
	gyrox = (imu_data[8]<<8)|imu_data[9];
	gyroy = (imu_data[10]<<8)|imu_data[11];
	gyroz = (imu_data[12]<<8)|imu_data[13];
	
	mpu9250.gyro.x = (float)(gyrox-GYROX_BIAS)/gyro_sensitivity;
	mpu9250.gyro.y = (float)(gyroy-GYROY_BIAS)/gyro_sensitivity;
	mpu9250.gyro.z = (float)(gyroz-GYROZ_BIAS)/gyro_sensitivity;
	
	mpu9250.acc.x = (float)(accx-ACCX_BIAS)/acc_sensitivity;
	mpu9250.acc.y = (float)(accy-ACCY_BIAS)/acc_sensitivity;
	mpu9250.acc.z = (float)(accz-ACCZ_BIAS)/acc_sensitivity;
	
	mag_count++;
	if(mag_count == 10)	//磁力计不能读取太频繁
	{
		HAL_I2C_Mem_Read(&hi2c1, AK8963_ADDRESS, AK8963_HXL, 1, mag_data, 6, HAL_MAX_DELAY);	//读取磁力计数据
		magx = (mag_data[0]<<8)|mag_data[1];
		magy = (mag_data[2]<<8)|mag_data[3];
		magz = (mag_data[4]<<8)|mag_data[5];
		mpu9250.mag.x = (float)magy/1000.0f;		//磁力计的坐标方位不同
		mpu9250.mag.y = (float)magx/1000.0f;
		mpu9250.mag.z = -(float)magz/1000.0f;
		pdata = 1;
		HAL_I2C_Mem_Write(&hi2c1, AK8963_ADDRESS, AK8963_CNTL1, 1, &pdata, 1, HAL_MAX_DELAY);	//为下一次读取磁力计数据做准备
		mag_count = 0;
	}
}

void imuUpdate(struct Axisf acc, struct Axisf gyro, struct Axisf mag)
{
	float q0q0 = q0 * q0;
	float q1q1 = q1 * q1;
	float q2q2 = q2 * q2;
	float q3q3 = q3 * q3;

	float q0q1 = q0 * q1;
	float q0q2 = q0 * q2;
	float q0q3 = q0 * q3;
	float q1q2 = q1 * q2;
	float q1q3 = q1 * q3;
	float q2q3 = q2 * q3;
	
	float normalise;
	float ex, ey, ez;
	float halfT;
	float hx, hy, hz, bx, bz;
	float vx, vy, vz, wx, wy, wz;
	
	now_update = HAL_GetTick(); //单位ms
	halfT = ((float)(now_update - last_update) / 2000.0f);
	last_update = now_update;
	
	gyro.x *= DEG2RAD;	/*度转弧度*/
	gyro.y *= DEG2RAD;
	gyro.z *= DEG2RAD;
	
	/* 对加速度计数据进行归一化处理 */
	if(acc.x != 0 || acc.y != 0 || acc.z != 0)
	{
		normalise = sqrt(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z);
		acc.x /= normalise;
		acc.y /= normalise;
		acc.z /= normalise;
	}
	
	/* 对磁力计数据进行归一化处理 */
	if(mag.x != 0 || mag.y != 0 || mag.z != 0)
	{
		normalise = sqrt(mag.x * mag.x + mag.y * mag.y + mag.z * mag.z);
		mag.x /= normalise;
		mag.y /= normalise;
		mag.z /= normalise;
	}
	
	/* 计算磁力计投影到物体坐标上的各个分量 */
	hx = 2.0f*mag.x*(0.5f - q2q2 - q3q3) + 2.0f*mag.y*(q1q2 - q0q3) + 2.0f*mag.z*(q1q3 + q0q2);
	hy = 2.0f*mag.x*(q1q2 + q0q3) + 2.0f*mag.y*(0.5f - q1q1 - q3q3) + 2.0f*mag.z*(q2q3 - q0q1);
	hz = 2.0f*mag.x*(q1q3 - q0q2) + 2.0f*mag.y*(q2q3 + q0q1) + 2.0f*mag.z*(0.5f - q1q1 - q2q2);         
	bx = sqrt((hx*hx) + (hy*hy));
	bz = hz; 
	
	/* 计算加速度计投影到物体坐标上的各个分量 */
	vx = 2.0f*(q1q3 - q0q2);
	vy = 2.0f*(q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3;
	
	/* 处理过后的磁力计新分量 */
	wx = 2.0f*bx*(0.5f - q2q2 - q3q3) + 2.0f*bz*(q1q3 - q0q2);
	wy = 2.0f*bx*(q1q2 - q0q3) + 2.0f*bz*(q0q1 + q2q3);
	wz = 2.0f*bx*(q0q2 + q1q3) + 2.0f*bz*(0.5f - q1q1 - q2q2); 
	
	/* 叉积误差累计，用以修正陀螺仪数据 */
	ex = (acc.y*vz - acc.z*vy) + (mag.y*wz - mag.z*wy);
	ey = (acc.z*vx - acc.x*vz) + (mag.z*wx - mag.x*wz);
	ez = (acc.x*vy - acc.y*vx) + (mag.x*wy - mag.y*wx);
	
	/* 互补滤波 PI */
	exInt += ex * Ki * halfT;
	eyInt += ey * Ki * halfT;	
	ezInt += ez * Ki * halfT;
	gyro.x += Kp*ex + exInt;
	gyro.y += Kp*ey + eyInt;
	gyro.z += Kp*ez + ezInt;
	
	/* 使用一阶龙格库塔更新四元数 */
	q0 += (-q1 * gyro.x - q2 * gyro.y - q3 * gyro.z) * halfT;
	q1 += ( q0 * gyro.x + q2 * gyro.z - q3 * gyro.y) * halfT;
	q2 += ( q0 * gyro.y - q1 * gyro.z + q3 * gyro.x) * halfT;
	q3 += ( q0 * gyro.z + q1 * gyro.y - q2 * gyro.x) * halfT;
	
	/* 对四元数进行归一化处理 */
	normalise = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 /= normalise;
	q1 /= normalise;
	q2 /= normalise;
	q3 /= normalise;
	
	/* 由四元数求解欧拉角 */
	mpu9250.attitude.x = -asinf(-2*q1*q3 + 2*q0*q2) * RAD2DEG;	//pitch
	mpu9250.attitude.y = atan2f(2*q2*q3 + 2*q0*q1, -2*q1*q1 - 2*q2*q2 + 1) * RAD2DEG;	//roll
	mpu9250.attitude.z = atan2f(2*q1*q2 + 2*q0*q3, -2*q2*q2 - 2*q3*q3 + 1) * RAD2DEG;	//yaw
	
	yaw = mpu9250.attitude.z*100;		//用于J-Scope读取
	pitch = mpu9250.attitude.x*100;
	roll = mpu9250.attitude.y*100;
}
