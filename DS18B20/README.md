# 原理讲解
[DS18B20温度传感器数据读取](https://imuncle.github.io/content.html?id=46)

# 函数接口
function_name|brief
:--:|:--:
DS18B20_Delay|基于TIM7的微秒延时函数，其中TIM7预分频之后频率为1MHz
DS18B20_Init|初始化函数，传入引脚号
DS18B20_MatchRom|匹配 DS18B20 ROM
DS18B20_GetTemp_SkipRom|在跳过匹配 ROM 情况下获取 DS18B20 温度值
DS18B20_GetTemp_MatchRom|在匹配 ROM 情况下获取 DS18B20 温度值
DS18B20_ReadId|在匹配 ROM 情况下获取 DS18B20 温度值

# 调用示例代码
```c
while(1)
{
    DS18B20_Init(GPIOA, GPIO_PIN_6);
	tempareture1 = DS18B20_GetTemp_SkipRom(GPIOA, GPIO_PIN_6);
}
```