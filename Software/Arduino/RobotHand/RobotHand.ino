/*
This program is to control the robot hands of openbionics initiative.
URL: openbionics.org
Copyright (C) 2015  azisi(https://github.com/zisi)

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software Foundation,
Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301  USA
*/

#include "Arduino.h"

/*
  MODE 1 for Standard Servo
  MODE 2 for AX12 Servo
*/
#define MODE 2

/*
  EXTENSION 0 without RobotHandExtension
  EXTENSION 1 with RobotHandExtension
*/
#define EXTENSION 0

#if MODE == 1
  #include <Servo.h>
  #define StdPin 10
  #define MaxPulse 2400 /* Pulse in us */
  #define MinPulse 544 /* Pulse in us */
  #define MaxAngle 218 /* Full rotation in deg */
  #define MaxVel 300 /* Max Velocity deg/s */
  Servo StdServo;
#elif MODE == 2
  #include <DynamixelSerial.h>
  #define DynPin 2
  #define MaxPos 1023
  #define MinPos 0
  #define MaxAngle 300 /* Full rotation in deg */
  #define DynBaudRate 1000000
  #define ID 1
  #define MaxTorque 1023
  #define DynVel 0.666   /* deg/s */
  #define MaxVel 350 /* Max Velocity */
#endif

#if EXTENSION == 1
  #define OpenButton 6
  #define CloseButton 9
  #define StepAngle 2
#endif

#define BaudRate 115200
#define BufferSize 256

void setup();
void loop();
void GetServoState(void);
void MoveServoPosition(float GoalPos);
void MoveServoVelocity(float GoalVl);
double signal2deg(int Signal);
int deg2signal(double deg);
boolean isNumber(char *input);

struct ServoState {
  double Pos;
  double Load;
};

void setup()
{
  /* Initialize serial communication */
  Serial.begin(BaudRate);
  /* Initialize Standard Servo */
  #if MODE == 1
    StdServo.attach(StdPin);
    MoveServoPosition(0);
  /* Initialize Dynamixel Servo */
  #elif MODE == 2
    Dynamixel.begin(DynBaudRate, DynPin);
    Dynamixel.ledStatus(ID, ON);
    Dynamixel.setID(1, ID);
    Dynamixel.setMaxTorque(ID, MaxTorque);
    delay(1000);
    MoveServoPosition(0);
    Dynamixel.ledStatus(ID, OFF);
  #endif
  #if EXTENSION == 1
    pinMode(OpenButton, INPUT);
    pinMode(CloseButton, INPUT);
  #endif
}

void loop()
{
  #if EXTENSION == 0
    char Buffer[BufferSize];
    char IncomingByte;
    char *Data = Buffer;
    static float GoalPos = 0;
    static float GoalVl = 0;
    static int BufferCnt = 0;

    while (Serial.available() > 0)
    {
      IncomingByte = Serial.read();
      /* New data */
      if (IncomingByte == '\n')
      {
        Buffer[BufferCnt]=0;
        /* Set Position */
        if ((Buffer[0] == 'p' && Buffer[1] == 's') || (Buffer[0] == 'P' && Buffer[1] == 'S'))
        {
          strncpy(Data, Buffer+2, 100);
          if (isNumber(Data) && *Data != 0 && atof(Data) >= 0 && atof(Data) <= MaxAngle)
          {
            GoalPos = atof(Data);
            MoveServoPosition(GoalPos);
            /* Return 0 is OK */
            Serial.print("0");
            Serial.print("\n");
          }
          else
          {
            /* Wrong Command return -1 */
            Serial.print("-1");
            Serial.print("\n");
          }
          /* Reset Buffer Counter */
          BufferCnt = 0;
        }
        /* Set Velocity */
        else if ((Buffer[0] == 'v' && Buffer[1] == 'l') || (Buffer[0] == 'V' && Buffer[1] == 'L'))
        {
          strncpy(Data, Buffer+2, 100);
          if (isNumber(Data) && *Data != 0 && atof(Data) >= -MaxVel && atof(Data) <= MaxVel)
          {
            GoalVl = atof(Data);
            MoveServoVelocity(GoalVl);
            /* Return 0 is OK */
            Serial.print("0");
            Serial.print("\n");
          }
          else
          {
            /* Wrong Command return -1 */
            Serial.print("-1");
            Serial.print("\n");
          }
          /* Reset Buffer Counter */
          BufferCnt = 0;
        }
        /* Get State of Servo Motor */
        else if ((Buffer[0] == 'g' && Buffer[1] == 's') || (Buffer[0] == 'G' && Buffer[1] == 'S'))
        {
          strncpy(Data, Buffer+2, 100);
          if (*Data == 0)
          {
            GetServoState();
          }
          else
          {
            /* Wrong Command return -1 */
            Serial.print("-1");
            Serial.print("\n");
          }
          /* Reset Buffer Counter */
          BufferCnt = 0;
        }
        else
        {
          /* Reset Buffer Counter */
          BufferCnt = 0;
          /* Wrong Command return -1 */
          Serial.print("-1");
          Serial.print("\n");
        }
      }
      else
      {
        /* Fill the buffer with incoming data */
        Buffer[BufferCnt] = IncomingByte;
        BufferCnt ++;
      }
    }
   #elif EXTENSION == 1
     static int OpenState = 0;
     static int CloseState = 0;
     static float pos = 0;

     /* Read push buttons */
     OpenState = digitalRead(OpenButton);
     CloseState = digitalRead(CloseButton);
     delay(20);

     if (OpenState == HIGH && CloseState == LOW)
     {
       pos += StepAngle;
     }
     else if (OpenState == LOW && CloseState == HIGH)
     {
       pos -= StepAngle;
     }
     if (pos < 0)
       pos = 0;
     else if (pos > MaxAngle)
       pos = MaxAngle;
     MoveServoPosition(pos);
   #endif
}

/* Get state of motor */
void GetServoState()
{
  ServoState State;
  #if MODE == 1
    State.Pos = signal2deg(StdServo.readMicroseconds());
    State.Load = 0;
  #elif MODE == 2
    State.Pos = signal2deg(Dynamixel.readPosition(ID));
    State.Load = Dynamixel.readLoad(ID);
  #endif
  /* Send servo state to host PC */
  Serial.print("0");
  Serial.print(";");
  Serial.print(State.Pos);
  Serial.print(";");
  Serial.print(State.Load);
  Serial.print("\n");
}

/* Position control */
void MoveServoPosition(float GoalPos)
{
  int Signal;
  Signal = deg2signal(GoalPos);
  #if MODE == 1
    StdServo.writeMicroseconds(Signal);
  #elif MODE == 2
    Dynamixel.setEndless(ID, OFF);
    Dynamixel.move(ID, Signal);
  #endif
}

/* Velocity control */
void MoveServoVelocity(float GoalVl)
{
  int Signal;
  #if MODE == 1
    static float qprv = 0;
    float q;
    float dt = 0.003;
    q = qprv + GoalVl*dt;
    qprv = q;
    Signal = deg2signal(q);
    StdServo.writeMicroseconds(Signal);
  #elif MODE == 2
    Signal = abs(GoalVl/DynVel);
    Dynamixel.setEndless(ID, ON);
    if (GoalVl > 0)
      Dynamixel.turn(ID, RIGTH, Signal);
    else if (GoalVl < 0)
      Dynamixel.turn(ID, LEFT, Signal);
    else
      Dynamixel.turn(ID, LEFT, 0);
  #endif
}

/* Convert signal to degrees */
double signal2deg(int Signal)
{
  #if MODE == 1
    return (double(Signal-MaxPulse)/double(MinPulse-MaxPulse))*MaxAngle;
  #elif MODE == 2
    return (Signal*double(MaxAngle)/double(MaxPos));
  #endif
}

/* Convert degrees to signal */
int deg2signal(double deg)
{
  #if MODE == 1
    return MaxPulse+(MinPulse-MaxPulse)*deg/MaxAngle ;
  #elif MODE == 2
    return MaxPos*deg/MaxAngle;
  #endif
}

/* Check if is argument in number */
boolean isNumber(char *input)
{
  for (int i = 0; input[i] != '\0'; i++)
  {
    if (isalpha(input[i]))
      return false;
  }
   return true;
}
