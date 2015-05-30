#include "Arduino.h"

/*
  MODE 1 for Standard Servo
  MODE 2 for AX12 Servo
*/
#define MODE 2

#if MODE == 1
  #define MaxAngle 218 /* Full rotation in deg */
#elif MODE == 2
  #define MaxAngle 300 /* Full rotation in deg */
#endif

#define SliderPin0 0
#define SliderPin1 1
#define MaxSignal 1023
#define MinSignal 0
#define BaudRate 115200
#define BufferSize 256

#define W 0.5

void setup();
void loop();
void GetPotState();
double signal2deg(int Signal);

void setup()
{
  /* Initialize serial communication */
  Serial.begin(BaudRate);
}

void loop()
{
  char Buffer[BufferSize];
  char IncomingByte;
  char *Data = Buffer;
  static int BufferCnt = 0;

  while (Serial.available() > 0)
  {
    IncomingByte = Serial.read();
    /* New data */
    if (IncomingByte == '\n')
    {
      Buffer[BufferCnt]=0;
      /* Get State of Potentiometer */
      if ((Buffer[0] == 'g' && Buffer[1] == 's') || (Buffer[0] == 'G' && Buffer[1] == 'S'))
      {
        strncpy(Data, Buffer+2, 100);
        if (*Data == 0)
        {
          GetPotState();
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
}

/* Read analog inputs */
void GetPotState()
{
  /* If you use 2 potentiometers uncomment the code */
  static int SliderValue0 = 0;
  /* static int SliderValue1 = 0; */
  
  SliderValue0 = analogRead(SliderPin0);
  /* delay(5); */
  /* SliderValue1 =  analogRead(SliderPin1); */
  /* Send slider values to host PC */
  Serial.print("0");
  Serial.print(";");
  Serial.print(signal2deg(SliderValue0));
  /* Serial.print(";"); */
  /* Serial.print(signal2deg(SliderValue1)); */
  Serial.print("\n");
}

/* Convert signal to degrees */
double signal2deg(int Signal)
{
  return ((double)MaxAngle*Signal/(double)MaxSignal);
}
