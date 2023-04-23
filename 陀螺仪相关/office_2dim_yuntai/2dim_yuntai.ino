/*   
    功能：云台可以根据陀螺仪传感器的数据实现姿态跟随
    接线：陀螺仪传感器接在扩展板的Gnd、Vcc（3.3v）、RX、TX；
          底座舵机0号接在扩展板的（Gnd、Vcc、D4）；
          摆动舵机1号接在扩展板的（Gnd、Vcc、D3）
*/
#include<ServoTimer2.h>  //调用舵机库函数
#include<Math.h>
ServoTimer2 myservo[2];  //声明两个舵机
int myservopin[2] = {4, 3}; // 定义舵机的引脚
#define Gyroscope_left_LimitAngle_X   -1.05  //读取到陀螺仪 X 轴向左偏的极限数值
#define Gyroscope_Right_LimitAngle_X   1.01  //读取到陀螺仪 X 轴向右偏的极限数值
#define Gyroscope_Middle_LimitAngle_X   0    //读取到陀螺仪 X 轴平放时的数值
#define Gyroscope_left_LimitAngle_Y   -1.05  //读取到陀螺仪 Y 轴向左偏的极限数值
#define Gyroscope_Right_LimitAngle_Y   1.01  //读取到陀螺仪 Y 轴向右偏的极限数值
#define Gyroscope_Middle_LimitAngle_Y   0    //读取到陀螺仪 Y 轴平放时的数值

#define Servo_One_Mix_Angle 0       //1号舵机最小角度
#define Servo_One_Max_Angle 180     //1号舵机最大角度
#define Servo_Two_Mix_Angle 0       //2号舵机最小角度
#define Servo_Two_Max_Angle 110     //2号舵机最大角度
#define Servo_Speed 10              //舵机速度

unsigned char Re_buf[11],counter=0;
unsigned char sign=0;
float a[3],w[3],angle[3],T;
int servo_angle_current[2] = {0,0};
float value_init[2]={90,90}; 
float f = 10.0;                     //舵机的频率 

void Get_gyroscope_And_Control()
{
   int gyroscope_acc_data[2]={0,0};
   int map_data[2]={0,0};
   if(sign)
  {  
     sign=0;
     if(Re_buf[0]==0x55)      //检查帧头
     {  
	switch(Re_buf [1])
	{
	case 0x51:
               {
		a[0] = (short(Re_buf [3]<<8| Re_buf [2]))/32768.0*16;
		a[1] = (short(Re_buf [5]<<8| Re_buf [4]))/32768.0*16;
		a[2] = (short(Re_buf [7]<<8| Re_buf [6]))/32768.0*16;
		T = (short(Re_buf [9]<<8| Re_buf [8]))/340.0+36.25;

                //把陀螺仪的沿X轴的加速度值转为舵机的角度
                map_data[0] = a[0] * 100;
                map_data[1] = a[1] * 100;
                if(a[0]>=Gyroscope_Middle_LimitAngle_X && a[0]<=Gyroscope_Right_LimitAngle_X){                                      
                   gyroscope_acc_data[0] = (int)map(map_data[0], 0, (int)Gyroscope_Right_LimitAngle_X*100, (Servo_One_Mix_Angle+Servo_One_Max_Angle)/2, Servo_One_Mix_Angle);
//                   Serial.print("X_left:"); Serial.print(gyroscope_acc_data[0]); Serial.print("  |  ");
                }
                if(a[0]<Gyroscope_Middle_LimitAngle_X && a[0]>=Gyroscope_left_LimitAngle_X){                   
                   gyroscope_acc_data[0] = (int)map(map_data[0], 0, (int)Gyroscope_left_LimitAngle_X*100, (Servo_One_Mix_Angle+Servo_One_Max_Angle)/2, Servo_One_Max_Angle);
//                   Serial.print("X_right: "); Serial.print(gyroscope_acc_data[0]); Serial.print("  |  ");
                }
                
                if(a[1]>=Gyroscope_Middle_LimitAngle_Y && a[1]<=Gyroscope_Right_LimitAngle_Y){                                      
                   gyroscope_acc_data[1] = (int)map(map_data[1], 0, (int)Gyroscope_Right_LimitAngle_Y*100, (Servo_Two_Mix_Angle+Servo_Two_Max_Angle)/2, Servo_Two_Mix_Angle);
//                   Serial.print("Y_left:"); Serial.print(gyroscope_acc_data[1]); Serial.print("  |  ");
                }
                if(a[1]<Gyroscope_Middle_LimitAngle_Y && a[1]>=Gyroscope_left_LimitAngle_Y){                   
                   gyroscope_acc_data[1] = (int)map(map_data[1], 0, (int)Gyroscope_left_LimitAngle_Y*100, (Servo_Two_Mix_Angle+Servo_Two_Max_Angle)/2, Servo_Two_Max_Angle);
//                   Serial.print("Y_right: "); Serial.print(gyroscope_acc_data[1]); Serial.print("  |  ");
                }
//                Serial.println();          
                servo_move(gyroscope_acc_data[0], gyroscope_acc_data[1]);                
               }break;
	}
    }
  } 
  delay(100); //后面如果没有其他的程序最好加一个小小的延时  
}

void serialEvent() {
  while (Serial.available()) {    
    //char inChar = (char)Serial.read(); Serial.print(inChar); //Output Original Data, use this code   
    Re_buf[counter]=(unsigned char)Serial.read();
    if(counter==0&&Re_buf[0]!=0x55) return;      //第0号数据不是帧头              
    counter++;       
    if(counter==11)             //接收到11个数据
    {    
       counter=0;               //重新赋值，准备下一帧数据的接收 
       sign=1;
    }      
  }
}

void servo_move(float value0, float value1) 
{
  float value_arguments[2] = {value0, value1};
  float value_delta[2];
  int value_date_int[2];
  for(int i=0;i<2;i++)
  {
    value_delta[i] = (value_arguments[i] - value_init[i]) / f;
  }
  
  for(int i=0;i<f;i++)
  {
    for(int k=0;k<2;k++)
    {
      value_init[k] = value_delta[k] == 0 ? value_arguments[k] : value_init[k] + value_delta[k];
      value_date_int[k] = value_init[k];
    }
    
    for(int j=0;j<2;j++)
    {
      myservo[j].write(map(value_date_int[j],0,180,500,2500));
//      myservo[j].write(map(value_init[j],500,2500,0,180));
      delay(Servo_Speed);
    }
  }
}

void setup()
{
   Serial.begin(115200);           //打开串口，并设置波特率为115200
   myservo[0].attach(myservopin[0]);
   myservo[1].attach(myservopin[1]);
} 

void loop()
{
   Get_gyroscope_And_Control();   //根据陀螺仪传感器的数据实现姿态跟随
}