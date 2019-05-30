#注意事项
本份代码基于HAL库，在STM32F4系列开发板上测试通过，使用串口6进行通信，适用于多摩川SA48系列编码器。

#函数接口
function_name|brief
:--:|:--:
tamagawa_read|多摩川编码器数据读取函数，输入参数见`tamagawa.h`的`DATA_ID`枚举量

#调用示例
```c
tamagawa_read(DATA_ID_3);
```