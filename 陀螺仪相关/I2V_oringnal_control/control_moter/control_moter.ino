#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h> 


Servo sg90; 
int servo_pin =3;


MPU6050 sensor ;
int16_t ax, ay, az ;
int16_t gx, gy, gz ;

//PID
float value;
float a=value*12.5;//PID计算

struct look{
float InputNum;//输入值 
float PointNum;//指定值 
float LastError;//上一次计算误差(微分需要) 
float NowError;//当前计算误差 
float Integral;//积分值 
float Kp;//初始化比例系数 
float Ki;//初始化积分系数
float Kd;//初始化微分系数
float Output;//输出的变化量大小 
}PID; //定义一个PID object

void initial(){
PID.InputNum=0;
PID.PointNum=90;
PID.LastError=0;
PID.NowError=0;
PID.Integral=0;
PID.Kp=2;
PID.Ki=0;
PID.Kd=0.2;
PID.Output=0;
}//初始化PID的参数 

float ComputePID(){
PID.NowError=PID.PointNum-PID.InputNum;
//累计误差，积分值
PID.Integral+=PID.NowError;
// float LimiteIntegral=30;
// if(PID.Integral>LimiteIntegral){
//  PID.Integral=LimiteIntegral;//积分限幅度
//  }
// else if(PID.Integral<-LimiteIntegral){
//  PID.Integral=-LimiteIntegral;//积分限幅度
// }
//PID计算 
PID.Output = PID.Kp*PID.NowError+PID.Ki*PID.Integral+PID.Kd*ax;
//角度的微分就是角速度，因此将(PID.NowError-PID.LastError)替换
PID.LastError = PID.NowError;
PID.InputNum += PID.Output;
value = PID.Output+PID.InputNum;//电机输出

return value;
}//计算PID数值  

void setup ()
{
sg90.attach (servo_pin);
// initial();//PID初始化
Serial.begin (9600);
Serial.println (sensor.testConnection () ? "Successfully Connected" : "Connection failed");
delay (200);
Serial.println ( "Taking Values from the sensor" );
delay (200);
}

void loop () 
{
sensor.getMotion6 (&ax, &ay, &az, &gx, &gy, &gz);
// PID.InputNum = ax;
// ax = map(ComputePID(), -17000, 17000, 25, 155);
ax = map(ax, -17000, 17000, 25, 155);
Serial.println (ax);
sg90.write (ax);
delay (200);
}