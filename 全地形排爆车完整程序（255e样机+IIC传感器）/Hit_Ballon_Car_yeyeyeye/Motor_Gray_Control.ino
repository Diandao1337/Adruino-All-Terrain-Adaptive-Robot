void Motor_Sensor_Init()
{
  for(int i=0;i<Car_Head_Gray_SensorPin_Num;i++) {//初始化灰度传感器
     pinMode(Gray_SensorPin[i],INPUT);
     delay(20);
  }
  for(int i=0;i<motor_num;i++) {//初始化 电机
     pinMode(Car_DC_Motor_Pin[i],OUTPUT);
     delay(20);
  } 
}

void Serialprint_gray_sensor_data_analogRead()
{
   int data_sensor[3]={0,0,0};
   for(int i=0;i<3;i++)
   {
     Serial.print(Gray_SensorPin[i]);Serial.print(": ");
     Serial.print(analogRead(Gray_SensorPin[i]));
     Serial.print(" | ");
   }
   Serial.println();  
}

void Serialprint_gray_sensor_data()
{
   int data_sensor[3]={0,0,0};
   for(int i=0;i<3;i++)
   {
     Serial.print(Gray_SensorPin[i]);Serial.print(": ");
     Serial.print(digitalRead(Gray_SensorPin[i]));
     Serial.print(" | ");
   }
   Serial.println();  
}

/*-----------------------------------------------------------------------
  A2         A3         A4
  ----------------------------------
  0          0          0      0x00  表示三个传感器都没有触发
  0          0          1      0x01  表示小车右边一个传感器触发
  0          1          0      0x02  表示小车中间传感器触发
  0          1          1      0x03  表示小车右边两个传感器都触发
  1          0          0      0x04  表示小车左边一个传感器触发
  1          0          1      0x05  
  1          1          0      0x06  表示小车左边两个传感器都触发
  1          1          1      0x07  表示小车三个传感器都触发  
------------------------------------------------------------------------*/
int Detection_tracking() //灰度传感器A4,A3,A2,用来小车巡线时，返回传感器数值；
{
  int num = 0;
  for(int i=0;i<Car_Head_Gray_SensorPin_Num;i++)
  {
    num |= ( (!digitalRead(Gray_SensorPin[i]) ) << i); 
  }
  Serial.println(num);
  return num;
}

void Tracking_Automatic_Tracking(unsigned long delay_tracking_time)
{
  unsigned long now_times = millis();
  while(millis()-now_times<delay_tracking_time)
  {
    Automatic_Tracking();
  }
}

void Automatic_Tracking() //小车前方两个灰度传感器用来寻迹（即小车巡线）。
{
  int Get_Data = 0;
  Get_Data = Detection_tracking();
  switch(Get_Data)
  {    
    case 0x00: Car_Move(Forward,Forward_Left_Speed,Forward_Right_Speed);break;//forward
    case 0x01: Car_Move(Right,  Right_Left_Speed,  Right_Right_Speed  );break;//RIGHT    
    case 0x02: Car_Move(Forward,Forward_Left_Speed,Forward_Right_Speed);break;//forward
    case 0x03: Car_Move(Right,  Right_Left_Speed,  Right_Right_Speed  );break;//right
    case 0x04: Car_Move(Left,   Left_Left_Speed,   Left_Right_Speed   );break;//left
    case 0x06: Car_Move(Left,   Left_Left_Speed,   Left_Right_Speed   );break;//LEFT
    case 0x07: //Gray_Three ++; 
               Car_Move(Forward,Forward_Left_Speed,Forward_Right_Speed);break;//forward
    default: break;     
  }
}

int Detection_tracking_analogRead() //灰度传感器A4,A3,A2,用来小车巡线时，返回传感器数值；
{
  int num = 0;
  int analogRead_data[3] = {0,0,0};
  for(int i=0;i<Car_Head_Gray_SensorPin_Num;i++)
  {
    analogRead_data[i] = analogRead(Gray_SensorPin[i]);
//    if( analogRead_data[0] <=300 ){analogRead_data[0] = 1;} else{analogRead_data[0] = 0;}//18
//    if( analogRead_data[1] <=300 ){analogRead_data[1] = 1;} else{analogRead_data[1] = 0;}//17
//    if( analogRead_data[2] <=170 ){analogRead_data[2] = 1;} else{analogRead_data[2] = 0;} //16 
    if( analogRead_data[i] <=240 ){analogRead_data[i] = 1;} else{analogRead_data[i] = 0;}
    num |= ( (analogRead_data[i]) << i); 
  }
  //Serial.println(num);
  return num;
}


void Automatic_Tracking_analogRead() //小车前方两个灰度传感器用来寻迹（即小车巡线）。
{
  int Get_Data = 0;
  Get_Data = Detection_tracking_analogRead();
  switch(Get_Data)
  {    
    case 0x00: Car_Move(Forward,Forward_Left_Speed,Forward_Right_Speed);break;//Led_shine(0,0, 0, 1);break;//forward
    case 0x01: Car_Move(Right,  Right_Left_Speed,  Right_Right_Speed  );break;//Led_shine(0,0, 1, 1);break;//RIGHT    
    case 0x02: Car_Move(Forward,Forward_Left_Speed,Forward_Right_Speed);break;//Led_shine(0,1, 0, 1);break;//forward
    case 0x03: Car_Move(Right,  Right_Left_Speed,  Right_Right_Speed  );break;//Led_shine(0,1, 1, 1);break;//right
    case 0x04: Car_Move(Left,   Left_Left_Speed,   Left_Right_Speed   );break;//Led_shine(1,0, 0, 1);break;//left
    case 0x06: Car_Move(Left,   Left_Left_Speed,   Left_Right_Speed   );break;//Led_shine(1,1, 0, 1);break;//LEFT
    case 0x07: 
           {
             Gray_Three ++; Car_Move(Forward,Forward_Left_Speed,Forward_Right_Speed);
             if(Gray_Three == 3)
             {
               road_one();
//               road_three();
             }
             if(Gray_Three == 2)
             {
               road_two();
             }             
             if(Gray_Three == 1)
             {
               road_three();
             }
    }break;
    default: break;     
  }
}

void road_one()
{
  Car_Move(Forward,253,212);delay(3500);
  Car_move_state_delaytime(Tracking_automatic,4000);
}

void road_two()
{
  Car_Move(Forward,255,200);delay(3500);
  Car_move_state_delaytime(Tracking_automatic,4000);
}

void road_three()
{
    Tracking_Automatic_Tracking(456);
    Car_Move(Stop,Car_speed_stop,Car_speed_stop);delay(3000);
    return_color_card(); //小车走到色卡开始识别色卡
    
    Tracking_Automatic_Tracking(1266);Car_Move(Stop,Car_speed_stop,Car_speed_stop);delay(500);
    return_color_ballon();
    if(color_detection_card == color_detection_ballon){
      Zha_Qi_Qiu(2);
      finish = false;
      finish_all = false;
    }//小车走到第一个气球区域，并判断气球颜色。

    if(finish_all){
        Tracking_Automatic_Tracking(1324);Car_Move(Stop,Car_speed_stop,Car_speed_stop);delay(500);
        return_color_ballon();
        if( (color_detection_card == color_detection_ballon) && finish ){
          Zha_Qi_Qiu(2);
          finish = false;
          finish_all = false;
      }//小车走到第二个气球区域，并判断气球颜色。
    }

    if(finish_all){
        Tracking_Automatic_Tracking(1236);Car_Move(Stop,Car_speed_stop,Car_speed_stop);delay(500);
        return_color_ballon();
        
        if( (color_detection_card == color_detection_ballon) && finish ){
          Zha_Qi_Qiu(2);
          finish = false;
          finish_all = false;
      }//小车走到第三个气球区域，并判断气球颜色。
    }
    Car_Move(Stop,Car_speed_stop,Car_speed_stop);delay(3000);
}


void Car_Move(int Mode,int LeftSpeed,int RightSpeed)                                                                           
{
  switch(Mode)
  {
    case Forward:{analogWrite(Car_DC_Motor_Pin[0],LeftSpeed);analogWrite(Car_DC_Motor_Pin[1],0);analogWrite(Car_DC_Motor_Pin[2],RightSpeed);analogWrite(Car_DC_Motor_Pin[3],0); }break;
    case Back:   {analogWrite(Car_DC_Motor_Pin[0],0);analogWrite(Car_DC_Motor_Pin[1],LeftSpeed);analogWrite(Car_DC_Motor_Pin[2],0);analogWrite(Car_DC_Motor_Pin[3],RightSpeed);}break;  
    case Left:   {analogWrite(Car_DC_Motor_Pin[0],0);analogWrite(Car_DC_Motor_Pin[1],LeftSpeed);analogWrite(Car_DC_Motor_Pin[2],RightSpeed);analogWrite(Car_DC_Motor_Pin[3],0); } break; 
    case Right:  {analogWrite(Car_DC_Motor_Pin[0],LeftSpeed);analogWrite(Car_DC_Motor_Pin[1],0);analogWrite(Car_DC_Motor_Pin[2],0);analogWrite(Car_DC_Motor_Pin[3],RightSpeed); }break;  
    case Stop:   {analogWrite(Car_DC_Motor_Pin[0],LeftSpeed);analogWrite(Car_DC_Motor_Pin[1],LeftSpeed);analogWrite(Car_DC_Motor_Pin[2],RightSpeed);analogWrite(Car_DC_Motor_Pin[3],RightSpeed);} break;
    default: break;    
  }
}

void Car_Move_Test()
{
  Car_Move(Forward,Forward_Left_Speed,Forward_Right_Speed);delay(1500);Car_Move(Stop,Car_speed_stop,Car_speed_stop);delay(1500);
  Car_Move(Back,Forward_Left_Speed,Forward_Right_Speed);delay(1500);Car_Move(Stop,Car_speed_stop,Car_speed_stop);delay(1500);
  Car_Move(Left,Left_Left_Speed,   Left_Right_Speed);delay(1500);Car_Move(Stop,Car_speed_stop,Car_speed_stop);delay(1500);
  Car_Move(Right,Right_Left_Speed,  Right_Right_Speed);delay(1500);Car_Move(Stop,Car_speed_stop,Car_speed_stop);delay(1500);
}


void Car_Stop(int delay_time)
{
  Car_Move(Stop,Car_speed_stop,Car_speed_stop);
  delay(delay_time);
}


void Car_move_state_delaytime(int Mode,int delay_time)
{
  switch(Mode)
  {
    case Forward:{ unsigned long time_now1 = millis();while( (millis() - time_now1)  < delay_time ){Car_Move(Forward,Forward_Left_Speed,Forward_Right_Speed);}}break;
    case Back:   { unsigned long time_now2 = millis();while( (millis() - time_now2)  < delay_time ){Car_Move(Back,Forward_Left_Speed,Forward_Right_Speed);}}break;
    case Left:   { unsigned long time_now3 = millis();while( (millis() - time_now3)  < delay_time ){Car_Move(Left,Left_Left_Speed,   Left_Right_Speed);}}break;
     case Right: { unsigned long time_now4 = millis();while( (millis() - time_now4)  < delay_time ){Car_Move(Right,Right_Left_Speed,  Right_Right_Speed);}}break;
     case Tracking_automatic:{ unsigned long time_now5 = millis();while( (millis() - time_now5)  < delay_time ){Automatic_Tracking();}}break;               
    default: break;   
  }
}

void Car_state_adjust_speed(int mode, int leftspeed_start, int leftspeed_end, int rightspeed_start, int rightspeed_end, int delay_time)
{
   switch(mode)
   {
     case Forward:
     {
        int max_delta_data = 0;
        int left_flag = 0;
        int right_flag = 0;
        int delta_time = 0;
        int L_speed_delta = 0;
        int R_speed_delta = 0;
        int delta_subtract_left  = abs( leftspeed_start - leftspeed_end );
        int delta_subtract_right = abs( rightspeed_start - rightspeed_end );
        max_delta_data = delta_subtract_left > delta_subtract_right ? delta_subtract_left : delta_subtract_right;
        delta_time = delay_time / max_delta_data ;
        left_flag = leftspeed_start - leftspeed_end >0 ? -1 : 1;
        right_flag = rightspeed_start - rightspeed_end >0 ? -1 : 1;
        unsigned long time_delay = millis();
        for( int i = 0; i< max_delta_data; i++ )
        {
          L_speed_delta = leftspeed_start +  i*left_flag;
          R_speed_delta = rightspeed_start + i*right_flag;
          if( (L_speed_delta<= leftspeed_end) || (L_speed_delta>= leftspeed_end) )
          {
            L_speed_delta = leftspeed_end;
          }
          if( (R_speed_delta<= rightspeed_end) ||  (R_speed_delta>= rightspeed_end) )
          {
            R_speed_delta = rightspeed_end;
          }
          analogWrite(Car_DC_Motor_Pin[0],L_speed_delta);
          analogWrite(Car_DC_Motor_Pin[1],0);
          analogWrite(Car_DC_Motor_Pin[2],R_speed_delta);
          analogWrite(Car_DC_Motor_Pin[3],0);
          delay(delta_time);
        }
     }break;
   }
}

void Car_Debug_Test()////程序进入调试模式（检测色卡颜色、检测气球颜色、检测灰度传感器、检测小车走直线）
{
  while(1)
  {
    #ifdef Debug_Color_Card
//      return_color_card();
      color();
    #endif
    
    #ifdef Debug_Color_Balloon
      return_color_ballon();
    #endif    

    #ifdef Debug_Gray_Sensor
      Serialprint_gray_sensor_data_analogRead();
    #endif

    #ifdef Debug_Car_Forward
      Car_Move(Forward,Forward_Left_Speed,Forward_Right_Speed);
    #endif
    delay(10);   
  }
}
