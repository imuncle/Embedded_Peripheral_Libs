#include "mpu9250.h"
#include "i2c.h"
#include <math.h>

#define DEG2RAD		0.017453293f	/*度转弧度*/
#define RAD2DEG		57.29578f		/*弧度转度*/

struct MPU9250_t mpu9250;


BiasObj	gyroBiasRunning;
int gyroBiasFound = 0;
float accScaleSum = 0;
float accScale = 1;

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

uint8_t imu_data[14]={0};

int last_yaw,yaw_round;

float faccx,faccy,faccz;

struct Axisf gyroBias;

volatile uint32_t last_update, now_update;

int MPU6500_Init(void)
{
	unsigned char pdata;
	//检查设备是否准备好
	HAL_I2C_IsDeviceReady(&hi2c2, MPU9250_ADDRESS, 10, 10);

	pdata=0x80; //复位MPU
	HAL_I2C_Mem_Write(&hi2c2, MPU9250_ADDRESS, MPU_PWR_MGMT1_REG, 1, &pdata, 1, 10);
	
	HAL_Delay(500);  //复位后需要等待一段时间，等待芯片复位完成

	pdata=0x00;	//唤醒MPU
	HAL_I2C_Mem_Write(&hi2c2, MPU9250_ADDRESS, MPU_PWR_MGMT1_REG, 1, &pdata, 1, 10);
	
	HAL_I2C_Mem_Read(&hi2c2, MPU9250_ADDRESS, MPU_WHO_I_AM, 1, &pdata, 1, 10);
	if(pdata != 0x71) return 1;
	
	pdata=0x18; //设置量程为2000
	HAL_I2C_Mem_Write(&hi2c2, MPU9250_ADDRESS, MPU_GYRO_CFG_REG, 1, &pdata, 1, 10); 

	pdata=0x08;	//设置加速度传感器量程±4g
	HAL_I2C_Mem_Write(&hi2c2, MPU9250_ADDRESS, MPU_ACCEL_CFG_REG, 1, &pdata, 1, 10); 

	pdata=0; //陀螺仪采样分频设置
	HAL_I2C_Mem_Write(&hi2c2, MPU9250_ADDRESS, MPU_SAMPLE_RATE_REG, 1, &pdata, 1, 10); 

	pdata=0;	//关闭所有中断
	HAL_I2C_Mem_Write(&hi2c2, MPU9250_ADDRESS, MPU_INT_EN_REG, 1, &pdata, 1, 10); 
	
	pdata=0;	//关闭FIFO
	HAL_I2C_Mem_Write(&hi2c2, MPU9250_ADDRESS, MPU_FIFO_EN_REG, 1, &pdata, 1, 10); 
	
	pdata = 6;	//设置MPU9250的数字低通滤波器
	HAL_I2C_Mem_Write(&hi2c2, MPU9250_ADDRESS, MPU_CFG_REG, 1, &pdata, 1, 10);

	pdata = 0x0C;
	HAL_I2C_Mem_Write(&hi2c2, MPU9250_ADDRESS, MPU_ACCEL_DLPF_DERG, 1, &pdata, 1, 10);
	
	pdata=0;	//使能陀螺仪和加速度工作
	HAL_I2C_Mem_Write(&hi2c2, MPU9250_ADDRESS, MPU_PWR_MGMT2_REG, 1, &pdata, 1, 10);
	
	return 0;
}

/*计算方差和平均值*/
static void sensorsCalculateVarianceAndMean(BiasObj* bias, struct Axisf* varOut, struct Axisf* meanOut)
{
	uint32_t i;
	int64_t sum[3] = {0};
	int64_t sumsq[3] = {0};

	for (i = 0; i < SENSORS_NBR_OF_BIAS_SAMPLES; i++)
	{
		sum[0] += bias->buffer[i].x;
		sum[1] += bias->buffer[i].y;
		sum[2] += bias->buffer[i].z;
		sumsq[0] += bias->buffer[i].x * bias->buffer[i].x;
		sumsq[1] += bias->buffer[i].y * bias->buffer[i].y;
		sumsq[2] += bias->buffer[i].z * bias->buffer[i].z;
	}

	varOut->x = (sumsq[0] - ((int64_t)sum[0] * sum[0]) / SENSORS_NBR_OF_BIAS_SAMPLES);
	varOut->y = (sumsq[1] - ((int64_t)sum[1] * sum[1]) / SENSORS_NBR_OF_BIAS_SAMPLES);
	varOut->z = (sumsq[2] - ((int64_t)sum[2] * sum[2]) / SENSORS_NBR_OF_BIAS_SAMPLES);

	meanOut->x = (float)sum[0] / SENSORS_NBR_OF_BIAS_SAMPLES;
	meanOut->y = (float)sum[1] / SENSORS_NBR_OF_BIAS_SAMPLES;
	meanOut->z = (float)sum[2] / SENSORS_NBR_OF_BIAS_SAMPLES;
}
/*传感器查找偏置值*/
static int sensorsFindBiasValue(BiasObj* bias)
{
	int foundbias = 0;

	if (bias->isBufferFilled)
	{
		
		struct Axisf mean;
		struct Axisf variance;
		sensorsCalculateVarianceAndMean(bias, &variance, &mean);

		if (variance.x < GYRO_VARIANCE_BASE && variance.y < GYRO_VARIANCE_BASE && variance.z < GYRO_VARIANCE_BASE)
		{
			bias->bias.x = mean.x;
			bias->bias.y = mean.y;
			bias->bias.z = mean.z;
			foundbias = 1;
			bias->isBiasValueFound= 1;
		}else
			bias->isBufferFilled=0;
	}
	return foundbias;
}

/**
 * 计算陀螺方差
 */
int processGyroBias(int16_t gx, int16_t gy, int16_t gz, struct Axisf *gyroBiasOut)
{
	static int count = 0;
	gyroBiasRunning.buffer[count].x = gx;
	gyroBiasRunning.buffer[count].y = gy;
	gyroBiasRunning.buffer[count].z = gz;
	count++;
	if(count == 1024)
	{
		count = 0;
		gyroBiasRunning.isBufferFilled = 1;
	}

	if (!gyroBiasRunning.isBiasValueFound)
	{
		sensorsFindBiasValue(&gyroBiasRunning);
	}

	gyroBiasOut->x = gyroBiasRunning.bias.x;
	gyroBiasOut->y = gyroBiasRunning.bias.y;
	gyroBiasOut->z = gyroBiasRunning.bias.z;

	return gyroBiasRunning.isBiasValueFound;
}

/**
 * 根据样本计算重力加速度缩放因子
 */
int processAccScale(int16_t ax, int16_t ay, int16_t az)
{
	static int accBiasFound = 0;
	static uint32_t accScaleSumCount = 0;

	if (!accBiasFound)
	{
		accScaleSum += sqrtf(powf((float)ax/8192, 2) + powf((float)ay/8192, 2) + powf((float)az/8192, 2));
		accScaleSumCount++;

		if (accScaleSumCount == SENSORS_ACC_SCALE_SAMPLES)
		{
			accScale = accScaleSum / SENSORS_ACC_SCALE_SAMPLES;
			accBiasFound = 1;
		}
	}

	return accBiasFound;
}

int GetImuData(void)
{
	if(HAL_I2C_Mem_Read(&hi2c2, MPU9250_ADDRESS, MPU_ACCEL_XOUTH_REG, 1, imu_data, 14, 10) == HAL_OK)	//读取陀螺仪和加速度计的数据
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

void imuDataHandle(void)
{
	short accx,accy,accz;
	short gyrox,gyroy,gyroz;
	float fgyrox, fgyroy, fgyroz;
	
	float gyro_sensitivity = 16.384f;
	int acc_sensitivity = 8192;
	
	accx = (imu_data[0]<<8)|imu_data[1];
	accy = (imu_data[2]<<8)|imu_data[3];
	accz = (imu_data[4]<<8)|imu_data[5];
	gyrox = (imu_data[8]<<8)|imu_data[9];
	gyroy = (imu_data[10]<<8)|imu_data[11];
	gyroz = (imu_data[12]<<8)|imu_data[13];
	
	gyroBiasFound = processGyroBias(gyrox, gyroy, gyroz, &gyroBias);
	
	if (gyroBiasFound)
	{
		processAccScale(accx, accy, accz);	/*计算accScale*/
	}
	
	fgyrox = -(float)(gyrox-gyroBias.x)/gyro_sensitivity;
	fgyroy = (float)(gyroy-gyroBias.y)/gyro_sensitivity;
	fgyroz = (float)(gyroz-gyroBias.z)/gyro_sensitivity;
	
	mpu9250.gyro.x = 0.8f*fgyrox + 0.2f*mpu9250.gyro.x;
	mpu9250.gyro.y = 0.8f*fgyroy + 0.2f*mpu9250.gyro.y;
	mpu9250.gyro.z = 0.8f*fgyroz + 0.2f*mpu9250.gyro.z;
	
	faccx = -(float)(accx)/acc_sensitivity/accScale;
	faccy = (float)(accy)/acc_sensitivity/accScale;
	faccz = (float)(accz)/acc_sensitivity/accScale;
	
	mpu9250.acc.x = 0.2f*faccx + 0.8f*mpu9250.acc.x;
	mpu9250.acc.y = 0.2f*faccy + 0.8f*mpu9250.acc.y;
	mpu9250.acc.z = 0.2f*faccz + 0.8f*mpu9250.acc.z;
}

void imuUpdate(struct Axisf acc, struct Axisf gyro)
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
	float vx, vy, vz;
	
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
	
	/* 计算加速度计投影到物体坐标上的各个分量 */
	vx = 2.0f*(q1q3 - q0q2);
	vy = 2.0f*(q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3;

	/* 叉积误差累计，用以修正陀螺仪数据 */
	ex = (acc.y*vz - acc.z*vy);
	ey = (acc.z*vx - acc.x*vz);
	ez = (acc.x*vy - acc.y*vx);
	
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
	
	last_yaw = mpu9250.angle.yaw;
	
	mpu9250.angle.yaw = mpu9250.attitude.z*22.756f;
	mpu9250.angle.pitch = mpu9250.attitude.x*22.756f;
	
	if(yaw - last_yaw > 4096)
	{
		yaw_round--;
	}
	else if(yaw - last_yaw < -4096)
	{
		yaw_round++;
	}
	mpu9250.angle.ecd_yaw = mpu9250.angle.yaw + yaw_round * 8192;
}
