#include <Arduino.h>
//#include <iostream>
#define DEBUGSerial Serial
//using namespace std;
int sensorPin = A0; 	//定义传感器的引脚

//下面4项内容需要根据实际型号和量程修正

//朢�小量稄1�7 根据具体型号对应手册获取,单位是g，这里以RP-18.3-ST型号为例，最小量程是20g
#define PRESS_MIN	20
//朢�大量稄1�7 根据具体型号对应手册获取,单位是g，这里以RP-18.3-ST型号为例，最大量程是6kg
#define PRESS_MAX	6000

//以下2个参数根据获取方法：
//理论上：
// 1.薄膜压力传感器不是精准的压力测试传感器，只��合粗略测量压力用，不能当压力计精确测量〄1�7
// 2. AO引脚输出的电压有效范围是0.1v刄1�73.3v，��实际根据不同传感器范围会在这个范围内，并不丢�定是朢�大��1�73.3v，也可能低于3.3v，要实际万用表测量，
// 	例程只是给出理论值，想要精确请自行万用表测量然后修正以下2个AO引脚电压输出的最大和朢�小��1�7
//调节方法＄1�7
//薄膜压力传感器的AO引脚输出的增益范围是通过板载AO_RES电位器调节实现的＄1�7
//想要稍微精准点，霢�要自己给定具体已知力，然后调节AO_RES电位器到串口输出重量正好是自己给定力就可以了
#define VOLTAGE_MIN 100
#define VOLTAGE_MAX 3300

void setup()
{
	DEBUGSerial.begin(9600); // setup serial
	Serial.println("setup end!");
}

void loop()
{
  //  cout << "loop start!" << endl;
	long Fdata = getPressValue(sensorPin);
	DEBUGSerial.print("F = ");
	DEBUGSerial.print(Fdata);
	DEBUGSerial.println(" g,");
	delay(300);
}

long getPressValue(int pin)
{
	long PRESS_AO = 0;
	int VOLTAGE_AO = 0;
	int value = analogRead(pin);

	DEBUGSerial.print("AD = ");
	DEBUGSerial.print(value);
	DEBUGSerial.print(" ,");

	VOLTAGE_AO = map(value, 0, 1023, 0, 5000);

	DEBUGSerial.print("V = ");
	DEBUGSerial.print(VOLTAGE_AO);
	DEBUGSerial.print(" mv,");

	if(VOLTAGE_AO < VOLTAGE_MIN)
	{
		PRESS_AO = 0;
	}
	else if(VOLTAGE_AO > VOLTAGE_MAX)
	{
		PRESS_AO = PRESS_MAX;
	}
	else
	{
		PRESS_AO = map(VOLTAGE_AO, VOLTAGE_MIN, VOLTAGE_MAX, PRESS_MIN, PRESS_MAX);
	}
	

	return PRESS_AO;
}
