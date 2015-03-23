#include <DynamixelSerial.h>

void setup(){
Dynamixel.begin(1000000,2);  // Inicialize the servo at 1Mbps and Pin Control 2
delay(1000);
}

void loop(){
  
  Dynamixel.ledStatus(1,OFF);
  Dynamixel.move(1,random(200,800));  // Move the Servo radomly from 200 to 800
  delay(1000);
  
  Dynamixel.setTempLimit(1,80);  // Set Max Temperature to 80 Celcius
  Dynamixel.setVoltageLimit(1,65,160);  // Set Operating Voltage from 6.5v to 16v
  Dynamixel.setMaxTorque(1,512);        // 50% of Torque
  Dynamixel.setSRL(1,2);                // Set the SRL to Return All
  
  Dynamixel.ledStatus(1,ON);           // Turn ON the LED

  delay(1000);
  
}
