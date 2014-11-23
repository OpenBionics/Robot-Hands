#include <Arduino.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <Servo.h>
void setup();
void loop();
void servoMove(float final_pos);
void control(float set_point);
int servoLoad();
int filter_load(int load);
void serialWrite(int package);
int adc(int pin);
#line 1 "src/sketch.ino"
//#include <string.h>
//#include <stdlib.h>
//#include <math.h>
//#include <Servo.h>

#define SERVO_PIN 6
#define LOAD_PIN 8
#define TS 0.00025 
#define TF 0.5
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
#define FILTER_SHIFT_LOAD 8

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
      int target_pulse = map(input[0]*10, 0, 100, MIN_PULSE, MAX_PULSE);
      /*Move servo and write to serial*/
      servoMove(target_pulse);    
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
	digitalWrite(13,HIGH);
	
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
  
  delay(100);
  serialWrite(servoLoad());
  
  init_pos = target_pos;
  t = 0;
  
  /*Stop write to serial*/
  //serialWrite(-1); 
  
  //t_stop = micros();
  //Serial.println((t_stop-t_start));
  
  digitalWrite(13,LOW);
  
}

/*Control Servo*/
void control(float set_point)
{
  int pulse;
  pulse = set_point;
  servo.writeMicroseconds(pulse);
}

/*Read the servo Load*/
int servoLoad()
{	
	int load;
	//load = filter_load(adc(LOAD_PIN));
	load = adc(LOAD_PIN);
	return load;
}

/*Simple digital Low-pass filter*/
int filter_load(int load)
{
	static long filterLoad = 512;
	filterLoad = filterLoad - (filterLoad >> FILTER_SHIFT_LOAD) + load;
	return (int) (filterLoad >> FILTER_SHIFT_LOAD);
}

/*Serial Communication*/
void serialWrite(int package)
{
	Serial.print(package);
	Serial.println(";");
}
	
/*Analog to Digital conversion*/
int adc(int pin)
{
  return(analogRead(pin));
}
