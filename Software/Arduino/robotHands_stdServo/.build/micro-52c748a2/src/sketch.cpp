#include <Arduino.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <Servo.h>
void setup();
void loop();
void interpolation(float final_pos);
void control(float set_point);
void serial_response();
void servo_readings();
int filter_power(int power);
int filter_position(int pos);
int adc(int pin);
#line 1 "src/sketch.ino"
//#include <string.h>
//#include <stdlib.h>
//#include <math.h>
//#include <Servo.h>

#define POWER_PIN 1
#define SERVO_PIN 9
#define TS 0.00025 
#define TF 1 
/*1<= K <=2*/
#define K 1.5
#define MAX_PULSE 2400
#define MIN_PULSE 544
/*
SHIFT(k)    Bandwidth (Normalized to 1Hz)   Rise Time (samples)
 1    			0.1197                          3
 2    			0.0466                          8
 3    			0.0217                          16
 4    			0.0104                          34
 5    			0.0051                          69
 6    			0.0026                          140
 7    			0.0012                          280
 8    			0.0007                          561
*/
#define FILTER_SHIFT_POS 1
#define FILTER_SHIFT_POWER 2

Servo servo;
/*Global*/
float state[2];

void setup()
{
  Serial.begin(115200);
  /*PWR on Led*/
  digitalWrite(13,LOW);
  /*Servo initialize*/
  servo.attach(SERVO_PIN);
  servo.writeMicroseconds(MIN_PULSE);
  for (int i=0; i<2;i++)
    state[i] = 0;
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
      /*Interpolation*/
      interpolation(input[0]);
      /*Send data*/
      serial_response();
     
    }
    /*Fill the buffer with incoming data*/
    else {
      buffer[counter] = incomingByte;
      counter++;
    }
  }
}

/*Trapezoidal velocity profile*/
void interpolation(float final_pos)
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
    /*Servo module*/
    servo_readings();
    control(target_pos); 
  }
  init_pos = target_pos;
  t = 0;
  
  //t_stop = micros();
  //Serial.println((t_stop-t_start));
  
}

/*Control Law*/
void control(float set_point)
{
  int pulse;
  /*Feedfoward control*/
  pulse = 0.005*set_point*set_point+7.472*set_point+389.39;
  //pulse = 8.773*set_point+336.354;
  servo.writeMicroseconds(pulse);
}

/*Send data*/
void serial_response()
{
  Serial.print(state[0]);
  Serial.print(";");
  Serial.print(state[1]);
  Serial.println(";");
}

/*Readings the servo sensors*/
void servo_readings()
{
  int tmp_pulse;
  //tmp_pulse = servo.readMicroseconds();
  tmp_pulse = filter_position(servo.readMicroseconds());
  state[0] = 0.121*tmp_pulse-41.11;
  //state[1] = adc(POWER_PIN);
  state[1] = filter_power(adc(POWER_PIN));
}

/*Simple digital Low-pass filter*/
int filter_power(int power)
{
  static long filter_power = 0;
  filter_power = filter_power - (filter_power >> FILTER_SHIFT_POWER) + power;
  return (int) (filter_power >> FILTER_SHIFT_POWER);
}
int filter_position(int pos)
{
  static long filter_pos = 0;
  filter_pos = filter_pos - (filter_pos >> FILTER_SHIFT_POS) + pos;
  return (int) (filter_pos >> FILTER_SHIFT_POS);
}

/*Analog to Digital conversion*/
int adc(int pin)
{
  return(analogRead(pin));
}
