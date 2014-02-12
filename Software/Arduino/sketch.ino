#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <Servo.h>

#define SERVO_PIN 6
#define TS 0.00025 
#define TF 1 
/*1<= K <=2*/
#define K 1.5
#define MAX_PULSE 2400
#define MIN_PULSE 600

Servo servo;

void setup()
{
  Serial.begin(115200);
  /*PWR on Led*/
  digitalWrite(13,LOW);
  /*Servo initialize*/
  servo.attach(SERVO_PIN);
  servo.writeMicroseconds(MIN_PULSE);
}

void loop()
{ 
  /*Serial*/
  char buffer[256];
  char incomingByte;
  char *p=buffer;
  char *str;
  float input[2];
  static int i,counter=0;
  
  /*Read from serial*/
  while (Serial.available() > 0){
    incomingByte = Serial.read();
    /*new data*/
    if (incomingByte == '\n'){
      p=buffer;
      i=0;
      while((str = strtok_r(p, ";" , &p))){
        input[i] = atof(str);
        i++;
      }
      counter = 0;
      int target_pulse = map(input[0]*10, 0, 10, MIN_PULSE, MAX_PULSE);
      /*Move servo and write to serial the Load*/
      servoMove(target_pulse);
      /*Direct servo control without Trapezoidal velocity profile*/
      //control(target_pulse);    
    }
    /*Fill the buffer with incoming data*/
    else {
      buffer[counter] = incomingByte;
      counter++;
    }
  }
}

/*Trapezoidal velocity profile*/
void servoMove(float final_pos)
{
  //unsigned long t_start,t_stop;
  
  static float init_pos = 0;
  static float t = 0;
  float target_pos;
  float tb;
  float v;
  float a;
  
  /*Initialize interpolation parameters*/
  if (final_pos == init_pos){
    v = 0;
    a = 0;
    tb = 0;
  }
  else{
    v = K*(final_pos-init_pos)/TF;
    tb = TF-((final_pos-init_pos)/v);
    a = v/tb;
  } 
  
  //t_start = micros();
  
  /*Main Loop*/
  while (t <= TF){
    if (t<=tb)
    target_pos = init_pos+0.5*a*t*t;
    else if (t < TF-tb)
      target_pos = 0.5*(final_pos+init_pos-v*TF)+v*t;
    else
      target_pos = final_pos-0.5*a*TF*TF+a*TF*t-0.5*a*t*t;
    t = t+TS;
    /*Servo move*/
    control(target_pos);
  }
  init_pos = target_pos;
  t = 0;
  
  //t_stop = micros();
  //Serial.println((t_stop-t_start));
  
}

/*Control Servo*/
void control(float set_point)
{
  int pulse;
  pulse = set_point;
  servo.writeMicroseconds(pulse);
}
