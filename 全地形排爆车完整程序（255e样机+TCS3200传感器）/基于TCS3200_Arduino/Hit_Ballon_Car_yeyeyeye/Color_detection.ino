

/*
 * 黄--------S3
 * 橙--------S2
 * 红--------GND
 * 棕--------VCC
 * 黑--------OUT
 * 白--------LED
 * 灰--------S1
 * 紫--------S0
 */
//TCS230连接设置 
const int s0 = 18; 
const int s1 = 19; 
const int s2 = 14; 
const int s3 = 15; 
const int out = 2; 
const int led = 17;

// Variables 
int red = 0; 
int green = 0; 
int blue = 0; 

void Color_Init()
{
 pinMode(s0, OUTPUT); 
 pinMode(s1, OUTPUT); 
 pinMode(s2, OUTPUT); 
 pinMode(s3, OUTPUT); 
 pinMode(out, INPUT); 
 pinMode(led, OUTPUT);
 digitalWrite(s0, HIGH); 
 digitalWrite(s1, HIGH);   
}


/*
 * color_judge[0]   red green
 * color_judge[0]   green red
 */
void return_color_ballon()
{
  digitalWrite(led, HIGH);
  int numbers_count = 0;
  int color_judge[12]={0,0,0,0,0,0,0,0,0,0,0,0};
  int red_summer,green_summer,blue_summer;
  Serial.println("---------------Start---------------");
  unsigned long time_now = millis();
  while( (millis() - time_now ) < 2000)
  {
     numbers_count ++;
     digitalWrite(s2, LOW); 
     digitalWrite(s3, LOW); 
     //count OUT, pRed, RED 
     red = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH); //blue
     digitalWrite(s3, HIGH); 
     //count OUT, pBLUE, BLUE 
     blue = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH); //red
     digitalWrite(s2, HIGH); 
     //count OUT, pGreen, GREEN 
     green = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH); //green
     if(red<blue){color_judge[0] = color_judge[0] +1;}  else{color_judge[1] = color_judge[1] +1;}
     if(red<green){color_judge[2] = color_judge[2] +1;} else{color_judge[3] = color_judge[3] +1;}

     if(blue<red){color_judge[4] = color_judge[4] +1;}  else{color_judge[5] = color_judge[5] +1;}
     if(blue<green){color_judge[6] = color_judge[6] +1;} else{color_judge[7] = color_judge[7] +1;}

     if(green<red){color_judge[8] = color_judge[8] +1;}  else{color_judge[9] = color_judge[9] +1;}
     if(green<blue){color_judge[10] = color_judge[10] +1;} else{color_judge[11] = color_judge[11] +1;}     
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
  digitalWrite(led, LOW);delay(500);
}


void return_color_card()
{
  digitalWrite(led, HIGH);
  int numbers_count = 0;
  int color_judge[12]={0,0,0,0,0,0,0,0,0,0,0,0};
  int red_summer,green_summer,blue_summer;
  Serial.println("---------------Start---------------");
  unsigned long time_now = millis();
  while( (millis() - time_now ) < 2000)
  {
     numbers_count ++;
     digitalWrite(s2, LOW); 
     digitalWrite(s3, LOW); 
     //count OUT, pRed, RED 
     red = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH); //blue
     digitalWrite(s3, HIGH); 
     //count OUT, pBLUE, BLUE 
     blue = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH); //red
     digitalWrite(s2, HIGH); 
     //count OUT, pGreen, GREEN 
     green = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH); //green
     if(red<blue){color_judge[0] = color_judge[0] +1;}  else{color_judge[1] = color_judge[1] +1;}
     if(red<green){color_judge[2] = color_judge[2] +1;} else{color_judge[3] = color_judge[3] +1;}

     if(blue<red){color_judge[4] = color_judge[4] +1;}  else{color_judge[5] = color_judge[5] +1;}
     if(blue<green){color_judge[6] = color_judge[6] +1;} else{color_judge[7] = color_judge[7] +1;}

     if(green<red){color_judge[8] = color_judge[8] +1;}  else{color_judge[9] = color_judge[9] +1;}
     if(green<blue){color_judge[10] = color_judge[10] +1;} else{color_judge[11] = color_judge[11] +1;}      
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
  digitalWrite(led, LOW);delay(500);
}

void color() 
{ 
 digitalWrite(led, HIGH);
 digitalWrite(s2, LOW); 
 digitalWrite(s3, LOW); 
 //count OUT, pRed, RED 
 red = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH); //blue
 digitalWrite(s3, HIGH); 
 //count OUT, pBLUE, BLUE 
 blue = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH); //red
 digitalWrite(s2, HIGH); 
 //count OUT, pGreen, GREEN 
 green = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH); //green
 Serial.print("red:");Serial.print(red);Serial.print(" | ");
 Serial.print("blue:");Serial.print(blue);Serial.print(" | ");
 Serial.print("green:");Serial.println(green);
}
