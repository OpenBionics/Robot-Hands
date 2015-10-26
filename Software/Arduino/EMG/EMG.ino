#include "Arduino.h"

#define EMGPin0 0
#define EMGPin1 0
#define MaxSignal 1023
#define MinSignal 0
#define MaxVolt 5 /* in V */
#define MinVolt 0
#define BaudRate 115200
#define BufferSize 256

void setup();
void loop();
void GetRawEMG();
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
          GetRawEMG();
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
void GetRawEMG()
{
  /* If you use 2 EMGs uncomment the code */
  static int EMGValue0 = 0;
  /* static int EMGValue1 = 0; */
  
  EMGValue0 = analogRead(EMGPin0);
  /* EMGValue1 = analogRead(EMGPin1); */
  /* Send slider values to host PC */
  Serial.print("0");
  Serial.print(";");
  Serial.print(signal2deg(EMGValue0));
  /* Serial.print(";"); */
  /* Serial.print(signal2deg(EMGValue1)); */
  Serial.print("\n");
}

/* Convert signal to degrees */
double signal2deg(int Signal)
{
  return ((double)Signal*MaxVolt/(double)MaxSignal); /* in V */
}
