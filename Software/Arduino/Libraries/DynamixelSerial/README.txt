**********************************   README FILE   ***************************************

Contact: Savage Electronics - Electronics For Everyone
  Autor:  Josue Alejandro Savage
  E-mail: savageelectronics@gmail.com 
  Web:    http://savageelectrtonics.blogspot.com/ 

Version: Dynamixel Library 15/01/2012 GPL v2.1

 Dynamixel Serial Version 1.2.0

Requeriments:

Arduino IDE Version 1.0
Wiring IDE 0100

Installation:

Copy the files to your IDE libraries folder.

examples:

In the Program:
Copy "DynamixelSerial" folder to Arduino/Contents/Resources/Java/libraries. 

or

In the SketchBook:
Copy "DynamixelSerial" folder to Arduino/libraries.

License:

 Dynamixel Half Duplex USART Comunication
 Copyright (c) 2011 Savage Electronics.
 Created by Savage on 27/01/11.
 
 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.
 
 This library is distributed in the hope that it will be useful,  
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.
 
 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

ChangeLog:

 25/07/2011 - Eliminado la modificacion serial para ser modificada dentro del mismo Hardware Serial.
 25/07/2011 - Modificado la funcion setBD() para aceptar todas la velocidades sin PDF.
 25/07/2011 - Agregada la funcion de Rotacion Continua.
 26/07/2011 - Agregada la funcion begin sin seteo de Direction_Pin.
 25/07/2011 - Agregada la funcion Reset.
 26/07/2011 - Agregada la funcion Reg_Write en move y moveSpeed.
 26/07/2011 - Agregada la funcion Action.
 13/12/2011 - Arreglado el manejo y envio de variables.
 22/12/2011 - Compatible con la actualizacion Arduino 1.0.
 10/01/2012 - Utilizacion de Macros y eliminacion codigo no necesario.
 11/01/2012 - Agregadas las funciones:
              int setTempLimit(unsigned char ID, unsigned char Temperature);
              int setAngleLimit(unsigned char ID, int CWLimit, int CCWLimit);
              int setVoltageLimit(unsigned char ID, unsigned char DVoltage, unsigned char UVoltage);
              int setMaxTorque(unsigned char ID, int MaxTorque);
              int setSRL(unsigned char ID, unsigned char SRL);
              int setRDT(unsigned char ID, unsigned char RDT);
              int setLEDAlarm(unsigned char ID, unsigned char LEDAlarm);
              int setShutdownAlarm(unsigned char ID, unsigned char SALARM);
              int setCMargin(unsigned char ID, unsigned char CWCMargin, unsigned char CCWCMargin);
              int setCSlope(unsigned char ID, unsigned char CWCSlope, unsigned char CCWCSlope);
15/01/2012 - Agregadas las funciones:
              int setPunch(unsigned char ID, int Punch);
              int moving(unsigned char ID);
              int lockRegister(unsigned char ID);
              int RWStatus(unsigned char ID);
              int readSpeed(unsigned char ID);
              int readLoad(unsigned char ID);

 TODO:
 
 FUNCION SYNCWRITE.

******************************************************************************************
