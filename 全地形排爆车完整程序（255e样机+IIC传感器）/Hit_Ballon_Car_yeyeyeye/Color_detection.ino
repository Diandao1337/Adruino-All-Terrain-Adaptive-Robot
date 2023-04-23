/*********************接线方式 
TCS3473x  Arduino_Uno 
  SDA         A4
  SCL         A5
  VIN         5V
  GND         GND
*************************/
#include <Wire.h>        //调用IIC库函数
#include "MH_TCS34725.h" //调用颜色识别传感器库函数

//颜色传感器不同通道值设置
MH_TCS34725 tcs = MH_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X); //设置颜色传感器采样周期50毫秒

void Color_Init()
{
  if (tcs.begin()) {                 //如果检测到颜色传感器模块
    Serial.println("Found sensor");  //串口打印 Found sensor
  } else {                           //如果没有检测到颜色传感器模块
    Serial.println("No TCS34725 found ... check your connections");//串口打印：没有找到颜色识别传感器模块
    while (1); // halt! //程序陷入死循环
  }
}

/*
 * color_judge[0]   red green
 * color_judge[0]   green red
 */
void return_color_ballon()
{
  int numbers_count = 0;
  int color_judge[12]={0,0,0,0,0,0,0,0,0,0,0,0};
  int red_summer,green_summer,blue_summer;
  Serial.println("---------------Start---------------");
  unsigned long time_now = millis();
  while( (millis() - time_now ) < 2000)
  {
     numbers_count ++;
     uint16_t clear, red, green, blue; 
     tcs.getRGBC(&red, &green, &blue, &clear);
     if(red>=blue){color_judge[0] = color_judge[0] +1;}  else{color_judge[1] = color_judge[1] +1;}
     if(red>=green){color_judge[2] = color_judge[2] +1;} else{color_judge[3] = color_judge[3] +1;}

     if(blue>=red){color_judge[4] = color_judge[4] +1;}  else{color_judge[5] = color_judge[5] +1;}
     if(blue>=green){color_judge[6] = color_judge[6] +1;} else{color_judge[7] = color_judge[7] +1;}

     if(green>=red){color_judge[8] = color_judge[8] +1;}  else{color_judge[9] = color_judge[9] +1;}
     if(green>=blue){color_judge[10] = color_judge[10] +1;} else{color_judge[11] = color_judge[11] +1;}     
  }
  Serial.println();
  if( (color_judge[0] > color_judge[1])  && ((color_judge[2] > color_judge[3])) )
  {
#ifdef DEBUG
    Serial.println("The color is red");
#endif
    color_detection_ballon = 1;
  }

  else if( (color_judge[4] > color_judge[5])  && ((color_judge[6] > color_judge[7]))  )
  {
#ifdef DEBUG
    Serial.println("The color is blue");
#endif
    color_detection_ballon = 2;
  }

  else if( (color_judge[8] > color_judge[9])  && ((color_judge[10] > color_judge[11])) )
  {
#ifdef DEBUG
    Serial.println("The color is green");
#endif
    color_detection_ballon = 3;   
  }
  else
  {
#ifdef DEBUG
    Serial.println("None color");
#endif    
  }
}


void return_color_card()
{
  int numbers_count = 0;
  int color_judge[12]={0,0,0,0,0,0,0,0,0,0,0,0};
  int red_summer,green_summer,blue_summer;
  Serial.println("---------------Start---------------");
  unsigned long time_now = millis();
  while( (millis() - time_now ) < 2000)
  {
     numbers_count ++;
     uint16_t clear, red, green, blue; 
     tcs.getRGBC(&red, &green, &blue, &clear);
     if(red>=blue){color_judge[0] = color_judge[0] +1;}  else{color_judge[1] = color_judge[1] +1;}
     if(red>=green){color_judge[2] = color_judge[2] +1;} else{color_judge[3] = color_judge[3] +1;}

     if(blue>=red){color_judge[4] = color_judge[4] +1;}  else{color_judge[5] = color_judge[5] +1;}
     if(blue>=green){color_judge[6] = color_judge[6] +1;} else{color_judge[7] = color_judge[7] +1;}

     if(green>=red){color_judge[8] = color_judge[8] +1;}  else{color_judge[9] = color_judge[9] +1;}
     if(green>=blue){color_judge[10] = color_judge[10] +1;} else{color_judge[11] = color_judge[11] +1;}      
  }
  Serial.println();
  if( (color_judge[0] > color_judge[1])  && ((color_judge[2] > color_judge[3])) )
  {
#ifdef DEBUG
    Serial.println("The color is red");
#endif
    color_detection_card = 1;  
  }

  else if( (color_judge[4] > color_judge[5])  && ((color_judge[6] > color_judge[7]))  )
  {
#ifdef DEBUG
    Serial.println("The color is blue");   
#endif
    color_detection_card = 2;
  }
  else if( (color_judge[8] > color_judge[9])  && ((color_judge[10] > color_judge[11])) )
  {
#ifdef DEBUG
    Serial.println("The color is green");
#endif
    color_detection_card = 3;   
  }  
  else
  {
#ifdef DEBUG
    Serial.println("None color");
#endif    
  }
}

void color() 
{ 
 uint16_t clear, red, green, blue; 
 tcs.getRGBC(&red, &green, &blue, &clear);
 Serial.print("red:");Serial.print(red);Serial.print(" | ");
 Serial.print("blue:");Serial.print(blue);Serial.print(" | ");
 Serial.print("green:");Serial.println(green);
}
