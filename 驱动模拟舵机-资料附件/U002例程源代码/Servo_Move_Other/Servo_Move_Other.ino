/*------------------------------------------------------------------------------------
  版权说明：Copyright 2022 Robottime(Beijing) Technology Co., Ltd. All Rights Reserved.
           Distributed under MIT license.See file LICENSE for detail or copy at
           https://opensource.org/licenses/MIT
           by 机器谱 2022-5-30 https://www.robotway.com/
  ----------------------------------------
  实验功能：
          实现舵机直接转动到某一角度.
          本实验使用自定义舵机函数实现舵机转动.
  ----------------------------------------
  实验接线：
          舵机D4
 ------------------------------------------------------------------------------------*/
#define ServoPin 4 //定义舵机引脚号为D4


void setup() {
  Serial.begin(9600);
  pinMode(ServoPin, OUTPUT);
  digitalWrite(ServoPin, LOW);//先保证拉低
  
}

void loop()
{
  ServoControl(0);  //舵机转到60度
  delay(2000);       //等待2秒
  ServoControl(150); //舵机转到120度
  delay(2000);       //等待2秒
}

void ServoControl(int servoAngle)
{
  double thisAngle = map(servoAngle, 0, 180, 500, 2500);//等比例角度值范围转换高电平持续时间范围
  unsigned char i = 50;//50Hz 每秒的周期次数(周期/秒) 即1S 50 个周期 每个周期20ms
  while (i--)
  {
    digitalWrite(ServoPin, HIGH); 
    delayMicroseconds(thisAngle); //高电平时间
    digitalWrite(ServoPin, LOW); 
    delayMicroseconds(20000 - thisAngle);//每个周期20ms减去高电平持续时间
  }
}
