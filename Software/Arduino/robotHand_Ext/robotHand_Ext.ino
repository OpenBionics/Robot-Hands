#include <DynamixelSerial.h>

#define openPIN 6
#define closePIN 9
#define DYNPIN 2
#define MAXPOS 1023
#define MINPOS 100


int openState = 0;
int closeState = 0;
int pos = MAXPOS;

void setup()
{
  //Init the PIN's
  pinMode(openPIN, INPUT);
  pinMode(closePIN, INPUT);
  //Init Dynamixel Serv0
  Dynamixel.begin(1000000,DYNPIN);  
  delay(1000);
  Dynamixel.move(1,MAXPOS);
  Dynamixel.ledStatus(1,ON);
  delay(1000);
  Dynamixel.ledStatus(1,OFF);
}

void loop()
{
  //Read push buttons
  openState = digitalRead(openPIN);
  closeState = digitalRead(closePIN);

  if (openState == HIGH && closeState == LOW)
  {
    pos = pos - 10;
    if (pos < MINPOS)
      pos = MINPOS;
  }
  else if (openState == LOW && closeState == HIGH)
  {
    pos = MAXPOS;
  }

  //Move Servo
  Dynamixel.move(1,pos);
  delay(50);

}