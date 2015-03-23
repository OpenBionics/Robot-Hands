#include <DynamixelSerial.h>

int Temperature,Voltage,Position; 

void setup(){
Dynamixel.begin(1000000,2);  // Inicialize the servo at 1Mbps and Pin Control 2
delay(1000);
}

void loop(){
  Temperature = Dynamixel.readTemperature(1); // Request and Print the Temperature
  Voltage = Dynamixel.readVoltage(1);         // Request and Print the Voltage
  Position = Dynamixel.readPosition(1);       // Request and Print the Position 
 
  Dynamixel.move(1,random(200,800));  // Move the Servo radomly from 200 to 800
 
 Dynamixel.end();                 // End Servo Comunication
 Serial.begin(9600);              // Begin Serial Comunication
 
  Serial.print(" *** Temperature: ");   // Print the variables in the Serial Monitor
  Serial.print(Temperature);
  Serial.print(" Celcius  Voltage: ");
  Serial.print(Voltage);
  Serial.print("  Volts   Position: ");
  Serial.print(Position);
  Serial.println(" of 1023 resolution");
  
 Serial.end();                     // End the Serial Comunication
 Dynamixel.begin(1000000,2);         // Begin Servo Comunication
 
delay(1000);

}
