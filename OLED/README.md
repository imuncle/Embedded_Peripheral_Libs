# 注意事项
本份代码使用的是STM32的SPI1接口，如果你的使用方式不一样，请注意修改。

# 函数接口
function_name|brief
:--:|:--:
OLED_Init|OLED屏幕初始化，清屏
OLED_ShowString|显示字符串
OLED_ShowNum|显示数字

# 调用示例
```c
OLED_Init();
OLED_ShowString(0,0,(uint8_t *)"OLED TEST", 12);
OLED_ShowNum(103,6,123,3,12);
```