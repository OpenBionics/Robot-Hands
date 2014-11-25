#include <DynamixelSerial.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>

#define MAX_POS 0
#define MIN_POS 800

struct state {
	int Position;
	int Load;
};

typedef struct state State;

State servoMove(float final_pos)
{
  State servoState;
  
  Dynamixel.ledStatus(1,ON);
  
  Dynamixel.move(1,final_pos);
  delay(1000);

  servoState.Position = Dynamixel.readPosition(1);
  servoState.Load = Dynamixel.readLoad(1);
    
  Dynamixel.ledStatus(1,OFF);
  
  return(servoState);
}

/*Serial Communication*/
void serialWrite(State package)
{
	Serial.print(package.Load);
	Serial.println(";");
}

void setup()
{
	Serial.begin(115200);
	
	Dynamixel.begin(1000000,2);
	delay(1000);typedef struct record Record;
	//Initialize dynamixel servo
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
      int target_pos = map(input[0]*10, 0, 100, MIN_POS, MAX_POS);
      /*Move servo and write to serial*/
      State servoState = servoMove(target_pos);
      serialWrite(servoState);   
    }
    /*Fill the buffer with incoming data*/
    else {
      buffer[counter] = incomingByte;
      counter++;
    }
  }
}
