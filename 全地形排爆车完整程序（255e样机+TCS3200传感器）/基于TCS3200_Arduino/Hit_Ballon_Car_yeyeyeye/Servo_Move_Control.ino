

//===================舵机========================舵机=======================舵机=======================舵机===========================舵机==========================舵机==============================舵机==============================舵机
//====舵机===========================舵机======================舵机=======================舵机=========================舵机==========================舵机============================舵机==================================舵机=============
//===================舵机========================舵机=======================舵机=======================舵机===========================舵机==========================舵机==============================舵机==============================舵机
//void Servo_Init()
//{
//  for(int i=0;i<servo_num;i++)
//  {
////    myServo[i].attach(servo_port[i]);
////    myServo[i].write(map(value_init[i],0,180,500,2500));
////    delay(20);
//    myServo.attach(servo_port[i]);
//    myServo.write(map(value_init[i],0,180,500,2500));
//    delay(20);
//  }
//}

void Servo_Init()
{
  for(int i=0;i<servo_num;i++)
  {
//    myServo[i].attach(servo_port[i]);
//    myServo[i].write(map(value_init[i],0,180,500,2500));
//    delay(20);
    myServo.attach(servo_port);
    myServo.write(map(value_init,0,180,500,2500));
    delay(20);
  }
}

//void ServoStop(int which){//释放舵机
//  myServo[which].detach();
//  digitalWrite(servo_port[which],LOW);
//}

void ServoStop(){//释放舵机
  myServo.detach();
  digitalWrite(servo_port,LOW);
}

//void ServoGo(int which , float where){//打开并给舵机写入相关角度
//  if(where!=200){
//    if(where==201) ServoStop(which);
//    else{
//      myServo[which].write(map(where,0,180,500,2500));
//    }
//  }
//}

void ServoGo(float where){//打开并给舵机写入相关角度
  if(where!=200){
    if(where==201) ServoStop();
    else{
      myServo.write(map(where,0,180,500,2500));
    }
  }
}

void Servo_Move_Single(int Start_angle,int End_angle,unsigned long Servo_move_time)
{
  int servo_flags = 0;
  int delta_servo_angle = abs(Start_angle-End_angle);
  if( (Start_angle - End_angle)<0 )
  {
    servo_flags = 1;
  }
  else{ servo_flags = -1; }
  for(int i=0;i<delta_servo_angle;i++)
  {
    myServo.write(map( Start_angle+(servo_flags*i) ,0,180,500,2500));
    delay(Servo_move_time);
  }
}

//void servo_move(float value0, float value1, float value2,int delaytimes){ //舵机动作函数
//  float value_arguments[] = {value0, value1, value2};
//  float value_delta[servo_num];  
//  for(int i=0;i<servo_num;i++){
//    value_delta[i] = (value_arguments[i] - value_init[i]) / f;
//  }  
//  for(int i=0;i<f;i++){
//    for(int k=0;k<servo_num;k++){
//      value_init[k] = value_delta[k] == 0 ? value_arguments[k] : value_init[k] + value_delta[k];
//    }
//    for(int j=0;j<servo_num;j++){
//      ServoGo(j,value_init[j]);
//    }
//    delay(delaytimes/f);
//  }
//}

void Zha_Qi_Qiu(int Numbers)
{
  for(int i=0;i<Numbers;i++)
  {
//    myServo.write(map( 130 ,0,180,500,2500));delay(350);
    myServo.write(map( 175 ,0,180,500,2500));delay(1000);
    myServo.write(map( 5 ,0,180,500,2500));delay(1000);
//    Servo_Move_Single(130,,2);delay(1000);
//    Servo_Move_Single(30,130,3);delay(1000);
  } 
}


