/*------------------------------------------------------------------------------------
  版权说明：Copyright 2023 Robottime(Beijing) Technology Co., Ltd. All Rights Reserved.
           Distributed under MIT license.See file LICENSE for detail or copy at
           https://opensource.org/licenses/MIT
           by 机器谱 2023-02-09 https://www.robotway.com/
  ------------------------------
  实验接线：两个直流电机接口的针脚号分别为D5,D6以及D9,D10;灰度传感器连在A0、A4接口上                                     
------------------------------------------------------------------------------------*/
int i=0;
void Forward();
void Back();
void Turn_left();
void Turn_right();
void Speed_up();
void Slow_down();
void setup() {
  // put your setup code here, to run once:
pinMode(A0,INPUT);
pinMode(A4,INPUT);
pinMode(9,OUTPUT);
pinMode(10,OUTPUT);
pinMode(5,OUTPUT);
pinMode(6,OUTPUT);
Serial.begin(9600);
}
void loop() {
  // put your main code here, to run repeatedly: 
 int a=digitalRead(A0);
 int b=digitalRead(A4);
 Serial.println(a);
 Serial.println(b);//Here is serial monitor,you will kown in later learning
if(a==0&&b==0)
Forward();
if(a==1&&b==0)
Turn_left();
if(a==0&&b==1)
Turn_right();
 
}
void Forward()//
{
analogWrite(9,120);
analogWrite(10,0);
analogWrite(5,120);
analogWrite(6,0);
delay(50);
}
void Back()
{
digitalWrite(9,LOW);
digitalWrite(10,HIGH);
digitalWrite(5,LOW);
digitalWrite(6,HIGH);
delay(4000);
}
void Turn_left()
{
digitalWrite(9,0);
digitalWrite(10,0);
digitalWrite(5,100);
digitalWrite(6,0);
delay(50);
}
void Turn_right()
{
digitalWrite(9,100);
digitalWrite(10,0);
digitalWrite(5,0);
digitalWrite(6,0);
delay(50);
}
void Speed_up()
{
 for(i=0;i<=255;i+=5)
  {
  analogWrite(9,i);
  analogWrite(10,0);
  analogWrite(5,i);
  analogWrite(6,0);
  delay(50);
  }//speed up

}
void Slow_down()
{
for(i=255;i>=0;i-=5)
  {  
  analogWrite(9,i);
  analogWrite(10,0);
  analogWrite(5,i);
  analogWrite(6,0);
  delay(50);
  }
}
