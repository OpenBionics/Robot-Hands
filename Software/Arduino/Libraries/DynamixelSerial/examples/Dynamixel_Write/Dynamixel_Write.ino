#include <DynamixelSerial.h>

void setup(){
Dynamixel.begin(1000000,2);  // Inicialize the servo at 1Mbps and Pin Control 2
delay(1000);
}

void loop(){

  Dynamixel.move(1,random(200,800));  // Move the Servo radomly from 200 to 800
  delay(1000);
  Dynamixel.moveSpeed(1,random(200,800),random(200,800));
  delay(2000);
  Dynamixel.setEndless(1,ON);
  Dynamixel.turn(1,RIGTH,1000);
  delay(3000);
  Dynamixel.turn(1,LEFT,1000);
  delay(3000);
  Dynamixel.setEndless(1,OFF);
  Dynamixel.ledStatus(1,ON);
  Dynamixel.moveRW(1,512);
  delay(1000);
  Dynamixel.action();
  Dynamixel.ledStatus(1,OFF);
 
delay(1000);

}
