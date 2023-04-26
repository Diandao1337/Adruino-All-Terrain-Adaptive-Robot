/*********************接线方式

TCS3473x   Arduino_Uno

  SDA         A4

  SCL         A5

  VIN         5V

  GND         GND

*************************/

#include <Wire.h>        //调用IIC库函数

#include "MH_TCS34725.h" //调用颜色识别传感器库函数

#ifdef __AVR__

  #include <avr/power.h>

#endif



//颜色传感器不同通道值设置

// MH_TCS34725 tcs = MH_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X); //设置颜色传感器采样周期50毫秒

MH_TCS34725 tcs = MH_TCS34725(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X);//设置颜色传感器采样周期700毫秒

//MH_TCS34725 tcs = MH_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_60X);//设置颜色传感器采样周期50毫秒（不推荐）

//MH_TCS34725 tcs = MH_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_16X);//设置颜色传感器采样周期50毫秒（不推荐）



void setup() {

  Serial.begin(115200);   //开启串口，并设置串口波特率位115200

  Serial.println("Color View Test!"); //串口打印：Color View Test!

  //检测是否有颜色传感器模块

  if (tcs.begin()) {                 //如果检测到颜色传感器模块

    Serial.println("Found sensor");   //串口打印 Found sensor

  } else {                           //如果没有检测到颜色传感器模块

    Serial.println("No TCS34725 found ... check your connections");//串口打印：没有找到颜色识别传感器模块

    while (1); // halt! //程序陷入死循环

  }

}



//串口打印Red、Green、Blue三色值

void loop() {

  uint16_t clear, red, green, blue; //分别定义用于存储红、绿、蓝三色值变量

  tcs.getRGBC(&red, &green, &blue, &clear); //将原始R/G/B值转换为色温（以度为单位）

  tcs.lock();   //禁用中断（可省略）

 

  uint32_t sum = clear;           //===========

  float r, g, b;                  // 计算红

  r = red; r /= sum;              // 绿、蓝

  g = green; g /= sum;            // 三色数

  b = blue; b /= sum;             // 值

  r *= 256; g *= 256; b *= 256;   //===========

  Serial.print("\t");////////////////////////////////////////////

  Serial.print((int)r); Serial.print("\t"); // 在串口中分别打印

  Serial.print((int)g); Serial.print("\t"); // 红、绿、蓝三色

  Serial.print((int)b);                     // 值

  Serial.println();
  delay(100);//////////////////////////////////////////////

}