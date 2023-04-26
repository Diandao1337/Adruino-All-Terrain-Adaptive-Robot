#include<Wire.h>
#include<I2Cdev.h>
#include<MPU6050.h>
//#include<Timer.h>
#include<Stepper.h>
#include <MsTimer2.h>

//Timer t;
MPU6050 book;
float timeChange=200;//滤波法采样时间间隔毫秒  
float dt=timeChange*0.001;//注意：dt的取值为滤波器采样时间  
float K1=0.05;// 对加速度计取值的权重  
float angle1;//一阶滤波角度输出 （角度为舵机的角度位置）
float angleax;
const float accLBS=16834;//加速度计的灵敏度
const float groLBS=131;//加速度计的灵敏度
int16_t raw_ax,raw_ay,raw_az;
int16_t raw_gx,raw_gy,raw_gz;
float ax,ay,az,gx,gy,gz;//定义传感器测量出的实际加速度和角速度
// 
//PID
float value;
float a=value*12.5;//PID计算

Stepper myStepper(a, 8,9,10,11);

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
PID.Output = PID.Kp*PID.NowError+PID.Ki*PID.Integral+PID.Kd*gy;
//角度的微分就是角速度，因此将(PID.NowError-PID.LastError)替换
PID.LastError = PID.NowError;
PID.InputNum += PID.Output;
value = PID.Output+PID.InputNum;//电机输出

return value;
}//计算PID数值  

//一阶滤波
void Yijielvbo(float angle_m, float gyro_m)
{
angle1 = K1 * angle_m+ (1-K1) * (angle1 + gyro_m * dt); //得到一个滤波后的值
}

void getangle() {
book.getMotion6(&raw_ax,&raw_ay,&raw_az,&raw_gx,&raw_gy,&raw_gz);//求原始值a，g的值
ax=raw_ax/accLBS;
ay=raw_ay/accLBS;
az=raw_az/accLBS;//传感器测量出来的实际加速度的值  
gx=raw_gx/groLBS;
gy=raw_gy/groLBS;
gz=raw_gz/groLBS;//传感器测量出来的实际角速度的值
angleax=atan2(ax,az)*57.3;
Yijielvbo(ax,-gy);//此时会返回一个滤波以后的值angle1，将其作为PID的输入
}

void printout()
{Serial.print(angleax);Serial.print("\t");
Serial.print(angle1);Serial.print("\t");
}

void setup() {
Wire.begin();//初始化I2C
Serial.begin(38400);//串口初始化   
book.initialize();
initial();//PID初始化
myStepper.setSpeed(20);
//int tickEvent1=t.every(timeChange, getangle);
MsTimer2::set(timeChange, getangle);
MsTimer2::start();
}

void loop(){
PID.InputNum=angle1;
//t.update();
myStepper.step(a);
printout();
delay(500);
}