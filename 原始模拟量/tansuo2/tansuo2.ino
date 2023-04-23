////原始code
#include <Arduino.h>
#include <Servo.h>
#include <TimerOne.h> //申明库文件
#include <FlexiTimer2.h>
#define left_pin    A0//左侧模拟灰度传感器
#define right_pin   A2//右侧模拟灰度传感器
#define LFwheel_1   5//左前轮引脚1
#define LFwheel_2   6//左前轮引脚2
#define RFwheel_1   9//右前轮引脚1
#define RFwheel_2   10//右前轮引脚2
//#define after_pin   7//右侧触须

//把TCS3200颜色传感器各控制引脚连到Arduino数字端口
#define S0    A4   //物体表面的反射光越强，TCS3002D内置振荡器产生的方波频率越高，
#define S1    A5  //S0和S1的组合决定输出信号频率比例因子，比例因子为2%
                 //比率因子为TCS3200传感器OUT引脚输出信号频率与其内置振荡器频率之比
#define S2     8   //S2和S3的组合决定让红、绿、蓝，哪种光线通过滤波器
#define S3     3
#define OUT    2  //TCS3200颜色传感器输出信号连接到Arduino中断0引脚，并引发脉冲信号中断
                  //在中断函数中记录TCS3200输出信号的脉冲个数
#define LED    A3  //控制TCS3200颜色传感器是否点亮LED灯
float g_SF[3];     //从TCS3200输出信号的脉冲数转换为RGB标准值的RGB比例因子
int   g_count = 0;  // 计算与反射光强相对应TCS3200颜色传感器输出信号的脉冲数
// 数组用于存储在1s内TCS3200输出信号的脉冲数，它乘以RGB比例因子就是RGB标准值
int   g_array[3];  
int   g_flag = 0;   // 滤波器模式选择顺序标志
int   c_state = 3;     //颜色状态，绿球为0，红色为1,蓝色为2，默认为绿球
int   c_state1 = 3;     //颜色状态，绿球为0，红色为1,蓝色为2，默认为绿球
int   c_state2 = 3;     //颜色状态，绿球为0，红色为1,蓝色为2，默认为绿球
int   c_state3 = 3;     //颜色状态，绿球为0，红色为1,蓝色为2，默认为绿球
int   Interrupt_time =120;//一次识别的时间
int red = 0;
int green = 0;
int blue = 0;
int yanse = 0;
int yanse1 = 0;
int yanse2 = 0;
int yanse3 = 0;

int turn_panduan = 0;
int left_val=0;
int right_val=0;
int after_val = 0;
int j = 0;
int i = 0;
int time = 0;
int panduanyanse = 0;
void FilterColor(int Level01, int Level02);
void Count();
void Callback();
void WB(int Level0, int Level1);
void get_color();
void get_color1();
void get_color2();
void get_color3();

int servoPin = 12;
int now_time = 0;
int after_time = 0;

void drive_forward1(int after_time){
    for(int now_time = 0;now_time < after_time;now_time++){
      trace();
    }
}

void servo(int angle) { //定义一个脉冲函数
  //发送50个脉冲
  for(int ppp=0;ppp<120;ppp++){
    int pulsewidth = (angle * 11) + 500; //将角度转化为500-2480的脉宽值
    digitalWrite(servoPin, HIGH);   //将舵机接口电平至高
    delayMicroseconds(pulsewidth);  //延时脉宽值的微秒数
    digitalWrite(servoPin, LOW);    //将舵机接口电平至低
    delayMicroseconds(20000 - pulsewidth);
  }
  delay(100);
}

void FilterColor(int Level01, int Level02)  
{  
  if(Level01 != 0)  
    Level01 = HIGH;  
  if(Level02 != 0)  
    Level02 = HIGH;  
  digitalWrite(S2, Level01);  
  digitalWrite(S3, Level02);  
}  
   
//中断函数，计算TCS3200输出信号的脉冲数  
void Count()  
{  
  g_count ++ ;
} 
//定时器中断函数，每1s中断后，把该时间内的红、绿、蓝三种光线通过滤波器时，  
//TCS3200输出信号脉冲个数分别存储到数组g_array[3]的相应元素变量中  
void Callback()  
{  
  switch(g_flag)
  {  
    case 0:  
         //Serial.println("正在颜色识别");  
         WB(LOW, LOW);   //选择让红色光线通过滤波器的模式  
         break;  
    case 1:  
         //Serial.print("0.3秒内红色脉冲数：");  
         //Serial.println(g_count);   //打印红光通过滤波器时，TCS3200输出的脉冲个数  
         g_array[0] = g_count;    //存储红光通过滤波器时，TCS3200输出的脉冲个数  
         WB(HIGH, HIGH);  //选择让绿色光线通过滤波器的模式  
         break;  
    case 2:  
         //Serial.print("0.3秒内绿色脉冲数：");  
         //Serial.println(g_count);   //打印绿光通过滤波器时，TCS3200输出的脉冲个数  
         g_array[1] = g_count ;    //存储绿光通过滤波器时，TCS3200输出的脉冲个数  
         WB(LOW, HIGH);  //选择让蓝色光线通过滤波器的模式  
         break;  
   
    case 3:  
         //Serial.print("0.3秒内蓝色脉冲数");  
         //Serial.println(g_count);   //打印蓝光通过滤波器时，TCS3200输出的脉冲个数  
         //Serial.println("颜色识别结束");  
         g_array[2] = g_count + 300;     //存储蓝光通过滤波器时，TCS3200输出的脉冲个数  
         WB(HIGH, LOW);   //选择无滤波器的模式    
         break;  
   default:  
         g_count = 0;     //计数值清零  
         break;  
  }  
}  

//设置反射光中红、绿、蓝三色光分别通过滤波器时如何处理数据的标志  
//该函数被Callback( )调用  
void WB(int Level0, int Level1)      
{  
  g_count = 0;   //计数值清零  
  g_flag ++;     //输出信号计数标志  
  FilterColor(Level0, Level1); //滤波器模式  
}



void get_color()
{
  
  int turns = 1;
  //Serial.println("开始识别颜色！打开LED补光灯！");
  int red_num =0;
  int gre_num =0;
  int blu_num =0;
  digitalWrite(LED, HIGH);//点亮LED灯
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
    Serial.println("                                    识别结果为红色小球！");
    c_state = 1;
    panduanyanse = 1;
   } 
   else if(blu_num>red_num && blu_num>gre_num)
   {
    Serial.println("                                     识别结果为蓝色小球！");
    c_state = 2;
    panduanyanse = 1;
   }
   else
   {
    Serial.println("                                      识别结果为绿色小球！");
    c_state = 0;
    panduanyanse = 1;
   } 
   Serial.println("识别颜色结束，关闭LED补光灯！"); 
   digitalWrite(LED, LOW);//关闭LED灯
}

void get_color1()
{
  
  int turns = 1;
  //Serial.println("开始识别颜色！打开LED补光灯！");
  int red_num =0;
  int gre_num =0;
  int blu_num =0;
  digitalWrite(LED, HIGH);//点亮LED灯
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
    Serial.println("                                    识别结果为红色小球！");
    c_state1 = 1;
    //panduanyanse = 1;
   } 
   else if(blu_num>red_num && blu_num>gre_num)
   {
    Serial.println("                                     识别结果为蓝色小球！");
    c_state1 = 2;
    //panduanyanse = 1;
   }
   else
   {
    Serial.println("                                      识别结果为绿色小球！");
    c_state1 = 0;
    //panduanyanse = 1;
   } 
   Serial.println("识别颜色结束，关闭LED补光灯！"); 
   digitalWrite(LED, LOW);//关闭LED灯
}

void get_color2()
{
  
  int turns = 1;
  //Serial.println("开始识别颜色！打开LED补光灯！");
  int red_num =0;
  int gre_num =0;
  int blu_num =0;
  digitalWrite(LED, HIGH);//点亮LED灯
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
    Serial.println("                                    识别结果为红色小球！");
    c_state2 = 1;
    //panduanyanse = 1;
   } 
   else if(blu_num>red_num && blu_num>gre_num)
   {
    Serial.println("                                     识别结果为蓝色小球！");
    c_state2 = 2;
    //panduanyanse = 1;
   }
   else
   {
    Serial.println("                                      识别结果为绿色小球！");
    c_state2 = 0;
    //panduanyanse = 1;
   } 
   Serial.println("识别颜色结束，关闭LED补光灯！"); 
   digitalWrite(LED, LOW);//关闭LED灯
}

void get_color3()
{
  
  int turns = 1;
  //Serial.println("开始识别颜色！打开LED补光灯！");
  int red_num =0;
  int gre_num =0;
  int blu_num =0;
  digitalWrite(LED, HIGH);//点亮LED灯
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
    Serial.println("                                    识别结果为红色小球！");
    c_state3 = 1;
    //panduanyanse = 1;
   } 
   else if(blu_num>red_num && blu_num>gre_num)
   {
    Serial.println("                                     识别结果为蓝色小球！");
    c_state3 = 2;
    //panduanyanse = 1;
   }
   else
   {
    Serial.println("                                      识别结果为绿色小球！");
    c_state3 = 0;
    //panduanyanse = 1;
   } 
   Serial.println("识别颜色结束，关闭LED补光灯！"); 
   digitalWrite(LED, LOW);//关闭LED灯
}

int yanse_shibie(){
  while(yanse == 0){
  digitalWrite(LED, HIGH);//点亮LED灯
  g_flag = 0;
  delay(2500);
   red = int (g_array[0] * 0.32273);
   green = int (g_array[1] * 0.5284);
   blue = int (g_array[2] * 0.4573);
   //blue = 230;
     if(red > 200 && red > green && red > blue){
    Serial.println("r");
    return 1;
    
   }
      if(green > 200 && green > red && green > blue){
    Serial.println("g");
    return 2;
    
   }
      if(blue > 200 && blue > green && blue > red){
    Serial.println("b");
    return 3;
    
   }
   
  }
  //digitalWrite(LED, LOW);//点亮LED灯
}

void scaler()         //计时器封装函数
{ 
  time = millis();
  if(time > 15000){
  //after_val=digitalRead(after_pin);
  //Serial.println(i);
  i = 1;
  if (j==0)
    {
    if (after_val == 0)
      j=1;
    }  
  if (j==1&&after_val == 1)
    {
      j=0;
      i=i+1;
      }
  }
}

void heixian(){
  if(left_val < 550  && right_val < 450 && panduanyanse == 0){
    delay(120);
    stop(); 
    delay(500);
    get_color();
    delay(500);
    i = 2;
}
}

void heixian2(){
  if(left_val < 470  && right_val < 400 && panduanyanse == 0){
    delay(500);
    stop(); 
    i = 4;
}
}

void trace()
{
   left_val=analogRead(left_pin);
   right_val=analogRead(right_pin);
   turn_panduan = left_val - right_val;
     Serial.print(left_val);
  Serial.print(" ");
  Serial.println(right_val);
   //Serial.println(turn_panduan);

  if (turn_panduan < -200 && turn_panduan > -300) //当右侧灰度值大于900为检测黄地 左转
  {
     drive_left();
  }
  else{
  if(turn_panduan < -300){
    drive_left2();
  }
  else{
    if (turn_panduan > 200 && turn_panduan < 300) //当左侧灰度值大于900为检测黄地 右转
    {
      drive_right(); 
    }
    else{
        if (turn_panduan > 300) //当左侧灰度值大于900为检测黄地 右转
    {
      drive_right2(); 
    }
    else{

      drive_forward();
    }
    }
  }
  }
}

void trace2()
{
   left_val=analogRead(left_pin);
   right_val=analogRead(right_pin);
   turn_panduan = left_val - right_val;
     Serial.print(left_val);
  Serial.print(" ");
  Serial.println(right_val);
   //Serial.println(turn_panduan);

  if (turn_panduan < -100 && turn_panduan > -300) //当右侧灰度值大于900为检测黄地 左转
  {
     drive_right(); 
  }
  else{
  if(turn_panduan < -300){
    drive_right2();
  }
  else{
    if (turn_panduan > 100 && turn_panduan < 180) //当左侧灰度值大于900为检测黄地 右转
    {
      drive_left();
    }
    else{
        if (turn_panduan > 180) //当左侧灰度值大于900为检测黄地 右转
    {
      drive_left2();
    }
    else{

      drive_forward();
    }
    }
  }
  }
}

// 小车前进
void drive_forward()
{
  analogWrite(LFwheel_1,230);//230
  analogWrite(LFwheel_2,0);
  analogWrite(RFwheel_1,230);//195
  analogWrite(RFwheel_2,0);
}

//小车左转
void drive_left()
{
  analogWrite(LFwheel_1,30);
  analogWrite(LFwheel_2,0);
  analogWrite(RFwheel_1,160);
  analogWrite(RFwheel_2,0);
}
void drive_left2()
{
  analogWrite(LFwheel_1,0);
  analogWrite(LFwheel_2,50);
  analogWrite(RFwheel_1,160);
  analogWrite(RFwheel_2,0);
}
//小车右转
void drive_right()
{
  analogWrite(LFwheel_1,160);
  analogWrite(LFwheel_2,0);
  analogWrite(RFwheel_1,30);
  analogWrite(RFwheel_2,0);
}

void drive_right2()
{
  analogWrite(LFwheel_1,160);
  analogWrite(LFwheel_2,0);
  analogWrite(RFwheel_1,0);
  analogWrite(RFwheel_2,50);
}

// 小车停止
void stop()
{
  analogWrite(LFwheel_1,0);
  analogWrite(LFwheel_2,0);
  analogWrite(RFwheel_1,0);
  analogWrite(RFwheel_2,0);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
   //servo.attach(12);//PWM引脚设置，与GPIO引脚号对应.
  pinMode(S0, OUTPUT);  
  pinMode(S1, OUTPUT);  
  pinMode(S2, OUTPUT);  
  pinMode(S3, OUTPUT);  
  pinMode(OUT, INPUT);  
  pinMode(LED, OUTPUT);
  pinMode(servoPin, OUTPUT); //设定舵机接口为输出接口
  digitalWrite(S0, HIGH);   
  digitalWrite(S1, HIGH);
  attachInterrupt(0, Count, RISING);
  
  delay(100);
  
}

void loop() {
  // put your main code here, to run repeatedly:
 // trace();

  if(i == 0){
    scaler();
    trace();
  }
  if(i == 1){
    trace();
    heixian();
  }
  if(i == 2){
    drive_forward1(80);
    //delay(760);
    stop();
    delay(500);
    get_color1();
    //识别第一个气球穷举
    if(c_state == 0 && c_state1 == 0){
       servo(0);
       delay(500);
       servo(100);
       delay(500);
     
       i = 3;
    }
    if(c_state == 0 && c_state1 == 1){
    drive_forward1(80);
    //delay(700);
    stop();
     delay(500);
    get_color2();
    }
    if(c_state == 0 && c_state1 == 2){
    drive_forward1(80);
    //delay(700);
    stop();
     delay(500);
    get_color2();
    }
          if(c_state == 1 && c_state1 == 0){
    drive_forward1(80);
    //delay(700);
    stop();
     delay(500);
    get_color2();
    }
           if(c_state == 1 && c_state1 == 1){
     servo(0);
     delay(500);
     servo(100);
     delay(500);

 
       i = 3;
    }
            if(c_state == 1 && c_state1 == 2){
    drive_forward1(80);
    //delay(700);
    stop();
     delay(500);
    get_color2();
    }
                    if(c_state == 2 && c_state1 == 0){
    drive_forward1(80);
    //delay(700);
    stop();
     delay(500);
    get_color2();
    }
                     if(c_state == 2 && c_state1 == 1){
    drive_forward1(80);
    //delay(700);
    stop();
     delay(500);
    get_color2();
    }
                      if(c_state == 2 && c_state1 == 2){
       servo(0);
       delay(500);
       servo(100);
       delay(500);
 
       i = 3;
    }
    //第二个气球
    if(c_state == 0 && c_state2 == 0){
       servo(0);
       delay(500);
       servo(100);
       delay(500);

       i = 3;
    }
    if(c_state == 0 && c_state2 == 1){
    drive_forward1(80);
    //delay(700);
    stop();
     delay(500);
    get_color3();
    }
    if(c_state == 0 && c_state2 == 2){
    drive_forward1(80);
    //delay(700);
    stop();
     delay(500);
    get_color3();
    }
          if(c_state == 1 && c_state2 == 0){
    drive_forward1(80);
    //delay(700);
    stop();
     delay(500);
    get_color3();
    }
           if(c_state == 1 && c_state2 == 1){

 servo(0);
 delay(500);
 servo(100);
 delay(500);

       i = 3;
    }
            if(c_state == 1 && c_state2 == 2){
    drive_forward1(80);
    //delay(700);
    stop();
     delay(500);
    get_color3();
    }
                    if(c_state == 2 && c_state2 == 0){
    drive_forward1(80);
    //delay(700);
    stop();
     delay(500);
    get_color3();
    }
                     if(c_state == 2 && c_state2 == 1){
    drive_forward1(80);
    //delay(700);
    stop();
     delay(500);
    get_color3();
    }
                      if(c_state == 2 && c_state2 == 2){
       servo(0);
       delay(500);
       servo(100);
       delay(500);

       i = 3;
    }
    //第三个
    if(c_state == 0 && c_state3 == 0){
       servo(0);
       delay(500);
       servo(100);
       delay(500);

       i = 3;
    }
    if(c_state == 0 && c_state3 == 1){
    drive_forward1(80);
    //delay(700);
    stop();
    
    //get_color2();
    }
    if(c_state == 0 && c_state3 == 2){
    drive_forward1(80);
    //delay(700);
    stop();
    //get_color2();
    }
          if(c_state == 1 && c_state3 == 0){
    drive_forward1(80);
    //delay(700);
    stop();
    //get_color2();
    }
           if(c_state == 1 && c_state3 == 1){
       servo(0);
       delay(500);
       servo(100);
       delay(500);

       i = 3;
    }
            if(c_state == 1 && c_state3 == 2){
    drive_forward1(80);
    //delay(700);
    stop();
    //get_color2();
    }
                    if(c_state == 2 && c_state3 == 0){
    drive_forward1(80);
    //delay(700);
    stop();
    //get_color2();
    }
                     if(c_state == 2 && c_state3 == 1){
    drive_forward1(80);
    //delay(700);
    stop();
    //get_color2();
    }
                      if(c_state == 2 && c_state3 == 2){
       servo(0);
       delay(500);
       servo(100);
       delay(500);
 
       i = 3;
    }

}
if(i == 3){
drive_forward1(400);
i = 4;
}
if(i == 4){
  stop();
}
 //scaler();
}
