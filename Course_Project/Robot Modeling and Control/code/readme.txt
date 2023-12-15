舵机采用Matlab控制，本次使用的舵机型号为AX-12A因此只能使用Protocol1.0进行控制

mdl_5dof.m 机械臂模型

mdl_Dyn_5dof.m 加入动力学参数的机械臂模型

trajectory planning.m 轨迹规划

electronic syste.m 是所有电机代码的整合，包括读写，同步输入等操作

PoseEstimation.py 为视觉识别程序

work.m MATLAB 中代码的实现，已整合电机部分

pressure system.info 转换模块主要使用方法： 
1.使用 AO 引脚，适合做有无压力，压力趋势变化，或者粗略测量压力值的用途 
2.可以控制调节 AO_RES 电阻，从而调节输出的模拟电压值的范围增益灵敏度，请自行调节到合适位置。可用万用表测量电压，或者单片机连接程序读取数值，调节到自己认为合适或者标定位置。 
AO 引脚最大输出范围是 0.1v-3.3v，实际对应不同型号传感器，输出的范围会小于这个范围。 //下面 4 项内容需要根据实际型号和量程修正//最小量程 根据具体型号对应手册获取,单位 
#define PRESS_MIN 20//最大量程 
根据具体型号对应手册获取,单位是 g，这里以 RP-18.3-ST 型号为例， #define PRESS_MAX 6000//以下 2 个参数根据获取方法：// 
理论上：// 1.薄膜压力传感器不是精准的压力测试传感器，只适合粗略测量压#define VOLTAGE_MIN 100#define VOLTAGE_MAX 3300 
51 单片机，本部分调整代码在 main.c 文件内。 
STM32 单片机，本部分调整代码在 main.c 文件内。 
2.使用 DO 引脚，适合做是否有压力的应用。 
可以通过调节 DO_RES 电阻，从而调节 DO 引脚输出的阀值。当压力大于调节的阀值时候， 
DO 引脚输出高电平， DO_LED 点亮。当压力小于设定的阀值时候，DO 引脚输出低电平，DO_LED 熄灭。 
ARDUINO 例程连接方式 
1.ARDUINO 例程读取 DO 读取 
· 
功能：ARDUINO UNO 读取 DO 的开关量数据，然后串口打印按下的次数， 
按下一次加 1 
接线方式： 
ARDUINO 
FSR 
5V -> VCC 
GND -> GND 
2 -> DO  
2.ARDUINO 例程读取 AO 
· 
功能：ARDUINO UNO 读取 AO 的模拟量数据，当按下传感器时候串口打印出 AD 值 
接线方式： 
ARDUINO 
FSR 
5V -> VCC 
GND -> GND 
A0 -> AO  

 转换模块主要使用方法： 
1.使用 AO 引脚，适合做有无压力，压力趋势变化，或者粗略测量压力值的用途 
2.可以控制调节 AO_RES 电阻，从而调节输出的模拟电压值的范围增益灵敏度，请自行调节到合适位置。可用万用表测量电压，或者单片机连接程序读取数值，调节到自己认为合适或者标定位置。 
AO 引脚最大输出范围是 0.1v-3.3v，实际对应不同型号传感器，输出的范围会小于这个范围。 //下面 4 项内容需要根据实际型号和量程修正//最小量程 根据具体型号对应手册获取,单位 
#define PRESS_MIN 20//最大量程 
根据具体型号对应手册获取,单位是 g，这里以 RP-18.3-ST 型号为例， #define PRESS_MAX 6000//以下 2 个参数根据获取方法：// 
理论上：// 1.薄膜压力传感器不是精准的压力测试传感器，只适合粗略测量压#define VOLTAGE_MIN 100#define VOLTAGE_MAX 3300 
51 单片机，本部分调整代码在 main.c 文件内。 
STM32 单片机，本部分调整代码在 main.c 文件内。 
2.使用 DO 引脚，适合做是否有压力的应用。 
可以通过调节 DO_RES 电阻，从而调节 DO 引脚输出的阀值。当压力大于调节的阀值时候， 
DO 引脚输出高电平， DO_LED 点亮。当压力小于设定的阀值时候，DO 引脚输出低电平，DO_LED 熄灭。 
ARDUINO 例程连接方式 
1.ARDUINO 例程读取 DO 读取 
· 
功能：ARDUINO UNO 读取 DO 的开关量数据，然后串口打印按下的次数， 
按下一次加 1 
接线方式： 
ARDUINO 
FSR 
5V -> VCC 
GND -> GND 
2 -> DO  
2.ARDUINO 例程读取 AO 
· 
功能：ARDUINO UNO 读取 AO 的模拟量数据，当按下传感器时候串口打印出 AD 值 
接线方式： 
ARDUINO 
FSR 
5V -> VCC 
GND -> GND 
A0 -> AO
