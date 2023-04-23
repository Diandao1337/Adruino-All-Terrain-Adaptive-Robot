
/*------------------------------------------------------------------------------------
  版权说明：Copyright 2022 Robottime(Beijing) Technology Co., Ltd. All Rights Reserved.
           Distributed under MIT license.See file LICENSE for detail or copy at
           https://opensource.org/licenses/MIT
           by 机器谱 2022-5-30 https://www.robotway.com/
  -----------------------------------------------
  实验功能：
          实现舵机直接转动到某一角度.
          本实验现象为舵机直接转到90度，再转到120度.
  ------------------------------------------------
  实验接线：
          舵机D4
 ------------------------------------------------------------------------------------*/
#include<Servo.h>     //调用舵机库
#define Servo_Pin 4   //定义舵机引脚号
#define Angle_One 90  //设置舵机转动的角度为90度
#define Angle_Two 120 //设置舵机转动的角度为120度
Servo myservo;        //声明舵机对象

void setup() {
   Serial.begin(9600);//开启串口，并设置波特率为9600
   myservo.attach(Servo_Pin);//设置舵机引脚
   myservo.write(Angle_One); //3号引脚舵机直接转到90度
   delay(1000); //等待1秒
   myservo.write(Angle_Two); //3号引脚舵机直接转到120度
   delay(1000); //等待1秒
}

void loop() {
  
}
