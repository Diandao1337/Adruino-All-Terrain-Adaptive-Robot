#include <FlexiTimer2.h>//需要自行安装该库

//颜色识别 
#define S0     3     
#define S1     4                      
#define S2     7//S2和S3的组合决定让红、绿、蓝，哪种光线通过滤波器
#define S3     8
#define OUT    2//颜色传感器输出信号连接到Arduino中断引脚，并引发脉冲信号中断
#define LED    13//控制颜色传感器是否点亮LED

int   g_count = 0;// 计算与反射光强相对应颜色传感器输出信号的脉冲数
int   g_array[3];// 数组用于存储在1s内输出信号的脉冲数
int   g_flag = 0;// 滤波器模式选择顺序标志
int   c_state = 0;//颜色状态，绿球为0,红色为1,蓝色为2
int   Interrupt_time =20;//一次识别的时间
int   Initial_color=0;//颜色储存初状态值
int   Last_color=1;//颜色储存终状态值
//寻迹
int sensor[3] = {0, 0, 0};//存储A0A2A3
float max = 3;//误差最大值
float s_range = 200;//速度映射最大范围
float Kp = 1, Ki = 0.5, Kd = 0.5; //PID控制算法
float error = 0, P = 0, I = 0, D = 0, PID_value = 0;//P比例I积分D微分
float previous_error = 0, previous_I = 0;//微分和积分常量
int initial_motor_speed = 200;//电机速度基数（正常时的速度）

//舵机
int servoPin =12;// 定义Servo对象来控制
int pos = 0;// 角度存储变量0
int poss =180;// 角度存储变量180

//黑线跳变
int All_black = 0;//记三个灰度传感器变黑次数
int Jump_state=0;//跳变状态0/1
int before=0;//上一状态值
int button=0;//现在的值
//其他调参
int Black_rangeA0=40;//A0A1A2口判定为黑范围 < Black_range判断为黑
int Black_rangeA1=40;
int Black_rangeA2=40;

int enter_timr=75;//识别区域到第一个颜色判定区域的循迹时间
int distinguish_time=40;//颜色判定区域各个颜色之间的循迹时间
int end_time=250;//终点循迹时间


void setup()//初始化 
{
  pinMode(5,OUTPUT);//电机
  pinMode(6,OUTPUT);
  pinMode(9,OUTPUT);
  pinMode(10,OUTPUT);
  pinMode(A0,INPUT);//传感
  pinMode(A2,INPUT);
  pinMode(A3,INPUT);
  pinMode(S0, OUTPUT);//颜色  
  pinMode(S1, OUTPUT);  
  pinMode(S2, OUTPUT);  
  pinMode(S3, OUTPUT);  
  pinMode(OUT, INPUT);  
  pinMode(LED, OUTPUT);
  pinMode(12,OUTPUT);//舵机
  
  digitalWrite(S0, HIGH);//颜色   
  digitalWrite(S1, HIGH);
  attachInterrupt(0, Count, RISING);
  Serial.begin(9600);
  delay(100);
}

void read_sensor_values()//模拟灰度识别
{
  sensor[0] = analogRead(A0);//读取模拟量的值
  sensor[1] = analogRead(A2);
  sensor[2] = analogRead(A3);
  if (sensor[0] < Black_rangeA0)//参数根据实际需要量修改
  {
    sensor[0] = 1;//转化为数字量
  }
  else
  {
    sensor[0] = 0;
  }
  if (sensor[1] < Black_rangeA1)
  {
    sensor[1] = 1;
  }
  else
  {
    sensor[1] = 0;
  }
  if (sensor[2] < Black_rangeA2)
  {
    sensor[2] = 1;
  }
  else
  {
    sensor[2] = 0;
  }

  if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1))//右转
  {
    error = -2;//需要大角度修正
  }
  else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 1))
  {
    error = -1;//需要小角度修正
  }
  else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 0))
  {
    error = 0;//直行
  }
  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 0))//左转
  {
    error = 1;//需要小角度修正
  }
  else if ((sensor[0] == 1) && (sensor[1] == 0) && (sensor[2] == 0))
  {
    error = 2;//需要大角度修正
  }
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0))//跑上一状态
  {
    if (error > 0)
    {
      error = max;//需要原地左转修正
    }
    else if(error != 0)
    {
      error = -max;//需要原地右转修正
      
    }
  }
  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1))
  {
     if ((error > 0) && (previous_error > 0))
        {    
        error = 0;
      }
      else if ((error < 0) && (previous_error < 0))
      { 
        error = 0;
      }
  }
}

void calculate_pid()//pid算法
{
  P = error;
  I = I + previous_I;
  D = error - previous_error;

  PID_value = (Kp * P) + (Ki * I) + (Kd * D);//PID算式
  Serial.println(PID_value);

  previous_I = I;
  previous_error = error;
}

void motor_control()
{
  // 计算有效电机转速：
  int left_motor_speed = initial_motor_speed - PID_value;//左电机速度
  int right_motor_speed = initial_motor_speed + PID_value;//右电机速度

  left_motor_speed = constrain(left_motor_speed, -s_range, s_range);//设定范围
  right_motor_speed = constrain(right_motor_speed, -s_range, s_range);

  //run(left_motor_speed, right_motor_speed);

   if((error>=-2)&&(error<=2))
   {
    run(left_motor_speed, right_motor_speed);
   }
   else if(error<-2)
   {
      //error = 0;
      sright(180);
   }
   else
   {
      //error = 0;
      sleft(180);
   }
}

void run(float Speed1,float Speed2)//前进 
{
  if(Speed1>0)
  {
    analogWrite(9, Speed1); //左
    analogWrite(10, 0);
  }
  else
  {
    analogWrite(9,0); //左
    analogWrite(10,abs(Speed1) );
  }
  if(Speed2>0)
  {
    analogWrite(5, Speed2); //右
    analogWrite(6, 0);
  }
  else
  {
    analogWrite(5, 0); //右
    analogWrite(6, abs(Speed2));
    }
}

void stop()   //刹车
{
  analogWrite(5,200);
  analogWrite(6,200);
  analogWrite(9,200);
  analogWrite(10,200);
}

/*void js()   //加速
{
  analogWrite(5,200);
  analogWrite(6,0);
  analogWrite(9,200);
  analogWrite(10,0);
}*/

void sleft(float Speed)//左转
{

  analogWrite(9, 0); 
  analogWrite(10, Speed);
  analogWrite(5, Speed); 
  analogWrite(6, 0); 
}

void sright(float Speed)//右转
{
  analogWrite(9, Speed);
  analogWrite(10, 0);
  analogWrite(5, 0);  
  analogWrite(6,Speed);
}

//颜色
//选择滤波器模式，决定让红、绿、蓝，哪种光线通过滤波器  
void FilterColor(int Level01, int Level02)  
{  
  if(Level01 != 0)  
    Level01 = HIGH;  
  if(Level02 != 0)  
    Level02 = HIGH;  
  digitalWrite(S2, Level01);  
  digitalWrite(S3, Level02); 
  //delay(1000); 
}  

//中断函数，计算TCS3200输出信号的脉冲数  
void Count()  
{  
  g_count ++ ;
} 
void Callback()
{  
  switch(g_flag)
  {  
    case 0:  
         //Serial.println("正在颜色识别");  
         WB(LOW, LOW);//选择让红色光线通过滤波器的模式  
         break;  
    case 1:  
         //Serial.print("0.3秒内红色脉冲数：");  
         //Serial.println(g_count);//打印红光通过滤波器时，TCS3200输出的脉冲个数  
         g_array[0] = g_count;//存储红光通过滤波器时，TCS3200输出的脉冲个数  
         WB(HIGH, HIGH);//选择让绿色光线通过滤波器的模式  
         break;  
    case 2:  
         //Serial.print("0.3秒内绿色脉冲数：");  
         //Serial.println(g_count);//打印绿光通过滤波器时，TCS3200输出的脉冲个数  
         g_array[1] = g_count;//存储绿光通过滤波器时，TCS3200输出的脉冲个数  
         WB(LOW, HIGH);//选择让蓝色光线通过滤波器的模式  
         break;  
   
    case 3:  
         //Serial.print("0.3秒内蓝色脉冲数");  
         //Serial.println(g_count);//打印蓝光通过滤波器时，TCS3200输出的脉冲个数  
         //Serial.println("颜色识别结束");  
         g_array[2] = g_count;//存储蓝光通过滤波器时，TCS3200输出的脉冲个数  
         WB(HIGH, LOW);//选择无滤波器的模式    
         break;  
   default:  
         g_count = 0;//计数值清零  
         break;  
  }  
}  

//设置反射光中红、绿、蓝三色光分别通过滤波器时如何处理数据的标志    
void WB(int Level0, int Level1)      
{  
  g_count = 0;   //计数值清零  
  g_flag ++;     //输出信号计数标志  
  FilterColor(Level0, Level1); //滤波器模式  
}

void get_color()
{
  int turns = 1;
  int red_num =0;
  int gre_num =0;
  int blu_num =0;
  digitalWrite(LED, HIGH);
  FlexiTimer2::set(Interrupt_time/4,Callback);
  FlexiTimer2::start();//定时器中断开
    for(int i =1;i<=turns;i++)
    {
      g_flag = 0;  
      delay(Interrupt_time);//一次识别时间为Interrupt_time
     if((g_array[0] > g_array[1]) && (g_array[0] > g_array[2]))
     {
      red_num++;
     }
      else if((g_array[2] > g_array[1]) && (g_array[2] > g_array[0]) )
     {
       blu_num++;
     }
      else
     {
       gre_num++;
     }
  }
 FlexiTimer2::stop();//定时器中断关
   if(red_num>blu_num && red_num>gre_num)
   {
    //Serial.println("红色小球 ");
    c_state = 1;
   } 
   else if(blu_num>red_num && blu_num>gre_num)
   {
    //Serial.println("蓝色小球 ");
    c_state = 2;
   }
   else
   {
    //Serial.println("绿色小球 ");
    c_state = 0;
   }  
   digitalWrite(LED, LOW);
}


//舵机
void servo(int angle)//定义一个舵机控制函数
{
  for(int i=0;i<50;i++)
  {
    int pulsewidth = (angle * 11)+500;
    digitalWrite(servoPin, HIGH); 
    delayMicroseconds(pulsewidth);
    digitalWrite(servoPin, LOW);
    delayMicroseconds(20000 - pulsewidth);
  }
  delay(100);
}

//跳变计数
void blackeningCount()//记录跳变为
{
  if((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1))
  {
    delay(20);
    if((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1))
    {
      Jump_state=1;
     }
    }
    else
    {
      Jump_state=0;
      }
   button=Jump_state;
   if(button==1 && before==0)
   {
      All_black++;
    }
   if(button!=before)
   {
    delay(20);
    }
    before=button;
}
void delayTracing(int time)//毫秒,短暂寻迹
{
  int t_base;
  for(t_base=0;t_base<time;t_base++)
  {
      read_sensor_values();//寻迹
      calculate_pid();
      motor_control();
      delay(10);
   }
}

void judge()//颜色判断
{
  int step1,step2;
  for(step1=0;step1<20;step1++)
  {
    stop();//停车
    delay(10);
    get_color();//第二次获得颜色
    Last_color=c_state;//获取第二次颜色
    
  }
  if(Last_color==Initial_color)//如果Last_color存储的颜色与Initial_color的相同
  {
    for(step2=0;step2<10;step2++)
    {
      servo(poss);//舵机转向扎气球
      delay(10);
    }
    for(step2=0;step2<5;step2++)
    {
      servo(90);//舵机转向回正
      delay(10);
    }
   }  
  }

void yanse()//颜色识别扎气球
{
   int i,j,k;
  if(All_black==5)//三个寻迹传感器全识别到黑
  {
    for(i=0;i<10;i++)
    {
      stop();//停车
      delay(10);
      servo(pos);//舵机转到颜色识别区域
      get_color();//获得颜色
      Initial_color=c_state;//获取初始颜色
      //Serial.print(Initial_color);
    }
    delayTracing(enter_timr);
    for(k=0;k<3;k++)
    {
      judge();//颜色判断
      //Serial.println(Last_color);
      if(Last_color==Initial_color)
      {
        break;
        }
      delayTracing(distinguish_time);
    }
    All_black++;//跳出本次程序
    }
}

void endTrack()//终点部分循迹后停止
{
  if(All_black==7)
  {
    delayTracing(end_time);
    while(1)
    {
      stop();//延迟后刹车
    }
   }
}

void loop() 
{
  read_sensor_values();//寻迹
  calculate_pid();
  motor_control();
  blackeningCount();
  yanse();//颜色识别扎气球
  endTrack();
  //Serial.println(PID_value);
  //Serial.print(sensor[0]);
  //Serial.print(Initial_color);
  //Serial.println(Last_color);
  //Serial.println(All_black);
  delay(10);
}
