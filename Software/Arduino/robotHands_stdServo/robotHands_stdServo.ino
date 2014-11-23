/* Commands: PS arg0 */
/* arg0: Angle of motor */
/* Angle range(deg): 0 - 218 */

#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <Servo.h>

#define SERVO_PIN 10
#define MAX_PULSE 2400
#define MIN_PULSE 544
#define deg2pulse 11 // The 2200us is 200deg 

Servo servo;

void setup()
{
  Serial.begin(115200);
  servo.attach(SERVO_PIN);
  /* Initial position of Robot Hand */
  servo.writeMicroseconds(MIN_PULSE);
}

void loop()
{
  char buffer[256];
  char incomingByte;
  char *data = buffer;
  char *str;
  float rx[4];
  static int i,counter = 0;

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
        while ((str = strtok_r(data, "," , &data)))
        {
          rx[i] = atof(str);
          i++;
        }
        counter = 0;
        servoMove(rx[0]);
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

/* Move the servo */
void servoMove(float final_pos)
{
  /* TF of servo */
  int pulse = deg2pulse*final_pos; 
  servo.writeMicroseconds(pulse);      
}