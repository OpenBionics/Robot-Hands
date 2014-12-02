#include <DynamixelSerial.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>

#define DYNPIN 2
#define MAXPOS 1023
#define MINPOS 100

void setup()
{
  /* Baud Rate 115200 */
  Serial.begin(115200);
  /* Initialize Dynemixel Serial interface */
  Dynamixel.begin(1000000,DYNPIN);  
  Dynamixel.ledStatus(1,ON);
  Dynamixel.move(1,MAXPOS);
  delay(1000);
  Dynamixel.ledStatus(1,OFF);
}

void loop()
{
  /*Serial*/
  char buffer[256];
  char incomingByte;
  char *data = buffer;
  char *str;
  float rx[4];
  static int i,counter = 0;
  
  /*Read from serial*/
  while (Serial.available() > 0)
  {
    incomingByte = Serial.read();
    /* New data */
    if (incomingByte == '\n')
    {
      buffer[counter]=0;
      if (buffer[0] == 'P' && buffer[1] == 'S')
      {
        strncpy(data, buffer+2, 100);
        i=0;
        while ((str = strtok_r(data, ";" , &data)))
        {
          rx[i] = atof(str);
          i++;
        }
        counter = 0;
        /* Move servo with Position Control */
        servoMove(rx[0]);      
      }
      else if (buffer[0] == 'F' && buffer[1] == 'C')
      {
        strncpy(data, buffer+2, 100);
        i=0;
        while ((str = strtok_r(data, ";" , &data)))
        {
          rx[i] = atof(str);
          i++;
        }
        counter = 0;
        /* Move servo wiht Force Control */
        force();      
      }
    }
    /* Fill the buffer with incoming data */
    else 
    {
      buffer[counter] = incomingByte;
      counter ++;
    }
  }
}

/* Posotion Control */
void servoMove(float final_pos)
{ 
  if (final_pos >= MINPOS && final_pos <=  MAXPOS)
  {
    Dynamixel.move(1,final_pos);
    delay(50);

    int angle = Dynamixel.readPosition(1);
    int load = Dynamixel.readLoad(1);
    /* Serial print to Host-PC */
    Serial.print(angle);
    Serial.print(";");
    Serial.print(load);
    Serial.println(";");
  }
}

/* Force Control */
void force()
{
  Serial.println("ToDo");
}
