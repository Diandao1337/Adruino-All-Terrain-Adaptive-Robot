/*------------------------------------------------------------------------------------
  版权说明：Copyright 2022 Robottime(Beijing) Technology Co., Ltd. All Rights Reserved.
           Distributed under MIT license.See file LICENSE for detail or copy at
           https://opensource.org/licenses/MIT
           by 机器谱 2022-5-30 https://www.robotway.com/
  ------------------------------------------------
  实验功能：
          实现舵机缓慢转动。本实验现象为舵机
          从60度缓慢转到120度,再从120度缓慢转动到60度。
  ------------------------------------------------
  实验接线：
          舵机D4
 ------------------------------------------------------------------------------------*/
#include<Servo.h>     //调用舵机库
Servo myservo;        //声明舵机对象
#define Servo_Pin 4   //定义舵机引脚号
#define Angle1 0     //设置舵机转动的角度为90度
#define Angle2 270    //设置舵机转动的角度为120度
#define Servo_Move_Delay 20 //舵机没动一次延时时间（单位：毫秒）

void setup() {
   Serial.begin(9600);//开启串口，并设置波特率为9600
   myservo.attach(Servo_Pin);//设置舵机引脚
   myservo.write(Angle1); //先让舵机快速转动到角度Angle1
}

void loop() {
  Servo_Move( Angle1, Angle2 ); //3号引脚舵机从Angle1缓慢转动到Angle2
  Servo_Move( Angle2, Angle1 ); //3号引脚舵机从Angle2缓慢转动到Angle1
}

//舵机缓慢运动函数
//计算该舵机从角度A转到角度B花费的总时间为：
//1.计算角度A与角度B的差值的绝对值，例如本示例中的：delta_angle
//2.总时间为： All_Time = delta_angle * Servo_Move_Delay.单位为：毫秒。
void Servo_Move(int start_angle, int end_angle){
  int delta_angle = abs(start_angle - end_angle); //计算两角度差的绝对值
  if( delta_angle == 0 ){ delta_angle = 1; } //避免输入角度与输出角度一致
  int servo_flags = 0; //定义舵机标志位
  servo_flags = (start_angle - end_angle)>0 ? -1 : 1;
  for( int i=0;i<delta_angle;i++){
    myservo.write(start_angle + i*servo_flags);
    delay(Servo_Move_Delay);
  }
}
