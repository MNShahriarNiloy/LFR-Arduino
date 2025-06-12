#include <Arduino.h>
#include <NewPing.h>
#define INA1   5
#define INA2   4
#define INB1   7
#define INB2   8
#define PWMA   3
#define PWMB   9
#define led4   11 //
#define button1 12 
#define button2 2
#define SONAR_NUM 1      // Number of sensors.
#define MAX_DISTANCE 15 // Maximum distance (in cm) to ping.


int BaseSpeed = 120;
int maxspeed = 255;
int speedA, speedB;

int sensor[8], minValue[8], maxValue[8], reading, sreading, last_reading;
int threshold[8] = {586,572,587,575,598,510,589,551};
uint32_t i, j, SN[1],count_L = 0,count_T = 0,count_cross = 0;
unsigned long current_millis,previous_millis;

int P, D, I, last_error, error, PID, position;
bool intersection = false,left = false,right = false,t_section = false,cross_section = false,deadEnd = false,T_left = false, T_right = false,cross_flag = false,bridge = false,inverted = false,obstacle = false, cross_triangle = false, sensorValue[8],sensorValueS[8];

float Kp = 0.022;
float Ki = 0.000000;
float Kd = 0.046999;

int base[8] = {1, 2, 4, 8, 16, 32, 64, 128};


void setup()
{

  Serial.begin(9600);

  pinMode(PWMA, OUTPUT);
  pinMode(INA1, OUTPUT);
  pinMode(INA2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(INB1, OUTPUT);
  pinMode(INB2, OUTPUT);
  //pinMode(button1, INPUT);
  pinMode(button2, INPUT);
  pinMode(led4, OUTPUT);
  //pinMode(led5, OUTPUT);
  //pinMode(led6, OUTPUT);
  //pinMode(led7, OUTPUT);
}



void motordrive(int M1pwm, int M2pwm)
{
  if (M1pwm >= 0)
  {
    digitalWrite(INA1, HIGH);
    digitalWrite(INA2, LOW);
    analogWrite(PWMA, M1pwm);
  }
  else if (M1pwm < 0)
  {
    digitalWrite(INA1, LOW);
    digitalWrite(INA2, HIGH);
    analogWrite(PWMA, -M1pwm);
  }
  else
  {
    digitalWrite(INA1, LOW);
    digitalWrite(INA2, LOW);
  }

  if (M2pwm >= 0)
  {
    digitalWrite(INB1, LOW);
    digitalWrite(INB2, HIGH);
    analogWrite(PWMB, M2pwm);
  }
  else if (M2pwm < 0)
  {
    digitalWrite(INB1, HIGH);
    digitalWrite(INB2, LOW);
    analogWrite(PWMB, -M2pwm);
  }
  else
  {
    digitalWrite(INB1, LOW);
    digitalWrite(INB2, LOW);
  }
}

void sensorRead_short_turn()
{
  sreading = 0;
  for (i = 0; i < 8; i++)
  {
    sensor[i] = analogRead(i);
    (sensor[i] > threshold[i]) ? sensorValueS[i] = 0 : sensorValueS[i] = 1;

    sreading += sensorValueS[i] * base[7-i];
    //Serial.print(sensorValueS[i]);
  }
  //Serial.println();
}


void sensorRead_short()
{
  sreading = 0;
  for (i = 0; i < 8; i++)
  {
    sensor[i] = analogRead(i);
    (sensor[i] > threshold[i]) ? sensorValueS[i] = 0 : sensorValueS[i] = 1;

    sreading += sensorValueS[i] * base[7-i];
    Serial.print(sensorValueS[i]);
  }
  Serial.println();
}
void turn()
{
 /**/ if(bridge == true)
  {
    motordrive(150,150);
    delay(2000);
  }/**/
   /*if(cross_triangle == true)
  {
    motordrive(120,120);
    delay(130);
  }*/
  else  if(left == true)
  {
    Serial.println("Left turn found");
    delay(100);
    motordrive(0,0);
    delay(5);
    while(1){
      //Serial.print("  turning  ");
      motordrive(-80,80);
      sensorRead_short_turn();
      if(sensorValueS[3] == 1 && sensorValueS[4] == 1) break;
    }
    motordrive(0,0);
    Serial.println("turn done");
    left = false;
  }
  else if(right == true)
  {
    Serial.println("Right turn found");
    delay(160);
    motordrive(0,0);
    delay(5);
    while(1){
      //Serial.print("  turning  ");
      motordrive(80,-80);
      sensorRead_short_turn();
      if((sensorValueS[5] == 1 && sensorValueS[4] == 1 )|| (sreading == 0b00011000)) break;
    }
    motordrive(0,0);
    Serial.println("turn done");
    right = false;
  }
  /*else if(t_section == true)
  {
    Serial.println("T-section found");
    delay(45);
    motordrive(0,0);
    delay(5);
    if(count_T < 100)
    {
      while(1){
      //Serial.print("  turning  ");
      motordrive(-70,70);
      sensorRead_short_turn();
      if(sensorValueS[3] == 1|| sensorValueS[4] == 1) break;
    }
    motordrive(0,0);
    Serial.println("turn done");
    t_section = false;
    count_T++;
    }
    else
    {
      while(1){
      //Serial.print("  turning  ");
      motordrive(60,-60);
      sensorRead_short_turn();
      if(sensorValueS[3] == 1|| sensorValueS[4] == 1) break;
    }
    motordrive(0,0);
    //Serial.println("turn done");
    count_T++;
    t_section = false;
    }
    

  }
  else if(cross_section == true)
  {
    Serial.println("cross section found");
      delay(25);
      motordrive(0,0);
      delay(5);
      cross_flag = false;
      while(1){
        //Serial.print("  turning  ");
        motordrive(80,-80);
        sensorRead_short_turn();
        if(sreading == 0b00000000) cross_flag = true;
        if((sensorValueS[3] == 1 && sensorValueS[4] == 1) && (cross_flag == true)) break;
        else if(sreading == 0b11111111) break;
      }
      motordrive(0,0);
      //Serial.println("turn done");
      cross_flag = false;
      count_cross++;
    cross_section = false;
  }
  
  else if(deadEnd == true)
  {
    Serial.println("deadend found");
    motordrive(0,0);
    delay(5);
    while(1){
      //Serial.print("  turning  ");
      motordrive(70,-70);
      sensorRead_short_turn();
      if(sensorValueS[3] == 1 && sensorValueS[4] == 1) break;
    }
    motordrive(0,0);
    Serial.println("turn done");
    deadEnd = false;
  }
  else if(T_left == true)
  {
    Serial.println("T_left found");
    if(count_L < 50)
    {
      delay(45);
      motordrive(0,0);
      delay(5);
      while(1){
        //Serial.print("  turning  ");
        motordrive(-80,80);
        sensorRead_short_turn();
        if(sreading == 0b00000000) cross_flag = true;
        if((sensorValueS[3] == 1 && sensorValueS[4] == 1) && (cross_flag == true)) break;
        else if(sreading == 0b11111111) break;
    }
    motordrive(0,0);
    Serial.println("turn done");
    cross_flag = false;
    count_L++;
    
  }
       T_left = false;
    }
  else if(T_right == true)
  {
    Serial.println("T-right found");
    //delay(45);
    motordrive(speedA,speedB);
    delay(100);
    T_right = false;
  }*/
  intersection = false;

}
void calculate_pid()
{
 
  P = error;
  I = I + error;
  D = error - last_error;
  last_error = error;


  PID = Kp * P + Ki * I + Kd * D;

  speedA = BaseSpeed - PID;
  speedB = BaseSpeed + PID;

  if (speedA > maxspeed)
    speedA = maxspeed;
  if (speedB < -maxspeed)
    speedB = -maxspeed;
}

void avoid_obstacle()
{
  motordrive(-100,100);
  delay(300);
  motordrive(100,100);
  delay(800);
  motordrive(100,-100);
  delay(300);
  motordrive(100,100);
  delay(500);
  motordrive(100,-100);
  delay(300);
  motordrive(100,100);
  delay(800);
  motordrive(-100,100);
  delay(300);
  obstacle = false;
}

void sensorRead()
{
  int numSens = 0, sum = 0, position,reading = 0;
  for (i = 0; i < 8; i++)
  {
    sensor[i] = analogRead(i);
    (sensor[i] > threshold[i]) ? sensorValue[i] = 0 : sensorValue[i] = 1;
    Serial.print(sensorValue[i]);
    numSens += sensorValue[i];
    sum += sensorValue[i] * i * 1000;
    reading += sensorValue[i] * base[7-i];
  }
  Serial.print("    ");
  Serial.print(reading);
  Serial.print("    ");

  /*if(reading == 0b11100111 || reading == 0b00100100 || reading == 0b01100110 || reading == 0b01110110 || reading == 0b11110011 || reading == 0b11110111)
  {
    inverted = true;
  }*/

  position = sum / numSens;
  error = position - 3500;
  if(sum == 8) error = 10000;
  Serial.print(position);
  Serial.print("  ");
  Serial.print(error);
  Serial.print("    ");
  Serial.println();
  delay(1);
  //SN[0] = sonar[0].ping_cm();
  //Serial.print(SN[0]);
  //Serial.print(" cm");
  //Serial.print("  ");
  /*if(SN[0] > 0 && SN[0] <= 12 ) {
    obstacle = true;
    motordrive(0,0);
    avoid_obstacle();
  }*/
  

  if(reading == 0b11000011 || reading == 0b11000001 || reading == 0b10000011 ) bridge = true;
  else if(reading == 0b00000000)
  {
    motordrive(0,0);
    if(last_reading >= 0)
    {
      previous_millis = millis();
      while (1)
      {
        motordrive(120,120);
        current_millis = millis();
        if(current_millis - previous_millis >= 200ul) break;
        sensorRead_short();
        if(sreading > 0)
        {
          calculate_pid();
          motordrive(speedA, speedB);
          break;
        } 
      }  
      if(sreading == 0b00000000) deadEnd = true;    
    } 
    
  }
  /*else if(reading == 0b11111111 || ((sensorValue[0] == 1 || sensorValue[1] == 1 || sensorValue[2] == 1) && (sensorValue[5] == 1 || sensorValue[6] == 1 || sensorValue[7] == 1)))
  {
    //if (reading == 0b10011001) cross_triangle = true;
    delay(10);
    //Serial.println("delay done");
    sensorRead_short();
    //Serial.println(sreading);
    if(((sensorValueS[3] == 1 && sensorValueS[4] == 1) || (sensorValueS[5] == 1 && sensorValueS[4] == 1) || (sensorValueS[3] == 1 && sensorValueS[2] == 1)) && (sreading != 0b11111111)) cross_section = true;
    else if(sreading == 0b0000000 ) t_section = true;
    else if(sreading == 0b11111111)
    {
      delay(150);
      sensorRead_short();
      if(sreading == 0b11111111) error = 10000;
      Serial.println("End point found");
      //digitalWrite(led7,HIGH);
      //digitalWrite(led5,HIGH);
      //digitalWrite(led6,HIGH);
    }
    
  }*/
  
  else if(((sensorValue[7] == 1 && sensorValue[6] == 1) || (sensorValue[5] == 1 && sensorValue[6] == 1) || (sensorValue[7] == 1) ) && (sensorValue[0] == 0 && sensorValue[1] == 0)){
    delay(50);
    sensorRead_short();
    //if((sensorValueS[2] == 1 && sensorValueS[3] == 1) || (sensorValueS[3] == 1 && sensorValueS[4] == 1 ) || (sensorValueS[3] == 1 || sensorValueS[4] == 1 )) T_left = true;
     if(sreading == 0b00000000 || (sensorValueS[3] == 0 && sensorValueS[4] == 0 && sensorValueS[5] == 0 && sensorValueS[6] == 0 && sensorValueS[7] == 0)) left = true;
  }

  else if((((sensorValue[0] == 1 && sensorValue[1] == 1) ||sensorValue[0] == 1 ) && (sensorValue[7] == 0 && sensorValue[6] == 0)) || (reading == 0b10000000)){
    delay(50);
    //Serial.println("right_delay");
    sensorRead_short();
    //if((sensorValueS[3] == 1 && sensorValueS[4] == 1)||(sensorValueS[3] == 1 && sensorValueS[2] == 1)||(sensorValueS[5] == 1 && sensorValueS[4] == 1)||(sensorValueS[3] == 1 || sensorValueS[4] == 1)) T_right = true;
    if(sreading == 0b00000000 || (sensorValueS[2] == 0 && sensorValueS[3] == 0 && sensorValueS[4] == 0 && sensorValueS[5] == 0 && sensorValueS[6] == 0  && sensorValueS[7] == 0)) right = true;
  }
  last_reading = reading;
  
  if(left == true || right == true || t_section == true || cross_section == true || deadEnd == true || T_right == true || T_left == true || bridge == true) intersection = true;
}

void sensorRead_Inverted()
{
  int numSens = 0, sum = 0, position,reading = 0;
  for (i = 0; i < 8; i++)
  {
    sensor[i] = analogRead(i);
    (sensor[i] > threshold[i]) ? sensorValue[i] = 1 : sensorValue[i] = 0;
    Serial.print(sensorValue[i]);
    numSens += sensorValue[i];
    sum += sensorValue[i] * i * 1000;
    reading += sensorValue[i] * base[7-i];
  }
  Serial.print("    ");
  Serial.print(reading);
  Serial.print("    ");

  position = sum / numSens;
  error = position - 3500;
  //if(sum == 8) error = 10000;
  Serial.print(position);
  Serial.print("  ");
  Serial.print(error);
  Serial.println();

}

void LineFollow_inverted()
{
  previous_millis = millis();
  while (1)
  {
    current_millis = millis();
    if(current_millis - previous_millis >= 200ul) break;
    sensorRead_Inverted();
    calculate_pid();
    motordrive(speedA+50, speedB+50);
  }
  motordrive(150,150);
  delay(1000);
  inverted = false;
}


void LineFollow()
{
  while (1)
  {
    sensorRead(); //if (inverted == false)
    //else if(inverted == true) sensorRead_Inverted();
    
    
    if(error == 10000){
      motordrive(0,0);
      break;
     }
    else if(inverted == true)
    {
      LineFollow_inverted();
    }  
    /*else if(obstacle == true)
    {
      avoid_obstacle();
    }*/
    else if(intersection == true)
    {
      Serial.println("found intersaction");
      turn();
    }
    else {
    calculate_pid();
    motordrive(speedA, speedB);
    } 
  }

}

void calibrate()
{
  digitalWrite(led4, HIGH);
  for (i = 0; i < 8; i++)
  {
    minValue[i] = analogRead(i);
    maxValue[i] = analogRead(i);
  }
  for (i = 0; i < 3000; i++)
  {
    motordrive(55, -55);

    for (j = 0; j < 8; j++)
    {
      if (analogRead(j) < minValue[j])
      {
        minValue[j] = analogRead(j);
      }
      else if (analogRead(j) > maxValue[j])
      {
        maxValue[j] = analogRead(j);
      }
    }
  }

  for (i = 0; i < 8; i++)
  {
    threshold[i] = (minValue[i] + maxValue[i]) / 2;
    Serial.print(threshold[i]);
    Serial.print("  --  ");
  }
  Serial.println();
  motordrive(0, 0);
  digitalWrite(led4, LOW);
}

void loop()
{
  while (digitalRead(button1) == HIGH)
  {
    calibrate();
    break;
  }
  //j = Serial.read();
  while (digitalRead(button2) == HIGH ) //|| j == 10
  {
    LineFollow();
    break;
  }
  motordrive(0, 0);
  j = 0;
}