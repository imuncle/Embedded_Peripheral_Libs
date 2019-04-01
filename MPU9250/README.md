# 原理讲解
[MPU9250姿态解析](https://imuncle.github.io/content.html?id=39)

# 注意事项
代码中使用的是STM32的**硬件I2C2**，如果你使用的I2C方式不一样，请注意修改。

# 函数接口
function_name|brief
:--:|:--:
MPU6500_Init|MPU6500初始化
GetImuData|获取IMU的陀螺仪数据和加速度数据
imuUpdate|IMU姿态解析函数，使用AHRS融合算法

# 调用示例
```c
MPU6500_Init();
while(1)
{
    GetImuData();
    imuUpdate();
}
```