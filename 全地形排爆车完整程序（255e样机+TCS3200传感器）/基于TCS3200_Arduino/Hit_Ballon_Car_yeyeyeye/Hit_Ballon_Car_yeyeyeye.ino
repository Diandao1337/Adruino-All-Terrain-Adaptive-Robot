 /*
 *=====================================================================================================*
 *实验接线:                                                                                            |
 *=====================================================================================================*
 *                       车头
 *   灰度传感器：     A2   A3   A4
 *                *----------------*
 *                |                |
 *                |                |
 *                |                | 
 *                |                | 右侧
 *         motor  |                | 车轮
 *          9,10  |                | 5，6
 *                |                |
 *                |                |
 *                |                | 
 *                |                | 
 *                |                | 
 *                *----------------*
 *                       车尾
 * 舵机接线：
 *         气球舵机:3
 *
*/

#include<ServoTimer2.h>        //调用舵机库函数
ServoTimer2 myServo;           //声明舵机
#define Forward_Left_Speed 125  //小车前进时左轮速度
#define Forward_Right_Speed 90//小车前进时右轮速度
#define Back_Left_Speed 160    //小车后退时左轮速度
#define Back_Right_Speed 110   //小车后退时右轮速度
#define Left_Left_Speed 235    //小车左转时左轮速度
#define Left_Right_Speed 240   //小车左转时右轮速度
#define Right_Left_Speed 235   //小车右转时左轮速度
#define Right_Right_Speed 240  //小车右转时右轮速度
#define Car_speed_stop 255     //小车刹车制动的速度
#define TrackingSensorNum 3    //小车寻迹时使用的灰度传感器数量

#define DEBUG                //程序进入调试模式
//#define Debug_Color_Card     //检测色卡颜色
#define Debug_Color_Balloon  //检测气球颜色
//#define Debug_Gray_Sensor    //检测灰度传感器
//#define Debug_Car_Forward    //检测小车走直线

int servo_num = 1;//定义舵机数量
int servo_port = 8;//定义舵机引脚
float value_init = 5;//定义舵机初始角度
int Car_DC_Motor_Pin[4] = {9,10,5,6};//直流电机引脚
int Gray_SensorPin[3]={A4,A3,A2};//寻迹、检测路口传感器
int f = 60; //定义舵机每个状态间转动的次数，以此来确定每个舵机每次转动的角度
int motor_num = sizeof(Car_DC_Motor_Pin) / sizeof(Car_DC_Motor_Pin[0]);//定义电机数量
int Car_Head_Gray_SensorPin_Num = sizeof(Gray_SensorPin)/sizeof(Gray_SensorPin[0]);//定义gray数量

bool finish=true;
int Gray_Three = 0; //记录三个灰度传感器同时触发的次数（即记录小车经过特殊路口的次数）
bool finish_all = true;//判断小车是否结束比赛（true表示没有结束比赛，false表示结束比赛）
int color_detection_card = 0; //记录颜色传感器识别到色卡的数值（红色为1，蓝色为2，绿色为3）
int color_detection_ballon = 0; //记录颜色传感器识别到气球的数值（红色为1，蓝色为2，绿色为3）
enum{Forward=1,Back,Left,Right,Stop,ForwardSpeedDown,BackSpeedDown,ForwardRoad,Tracking_automatic};//小车各种模式状态


void setup() {
  delay(1500);Serial.begin(9600);//打开串口并启用9600波特率
  Motor_Sensor_Init();//电机及传感器引脚初始化
  Servo_Init(); //舵机引脚初始化
  Color_Init();delay(1000);//颜色引脚初始化
  #ifdef DEBUG //判断小车是否要进入调试模式
    Car_Debug_Test();
  #endif
}

void loop() {
  
  Automatic_Tracking_analogRead(); 
  
}
