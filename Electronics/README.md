# Electronics

This folder presents the robot hands electronics (the Schematic, the PCB Layout and the OpenBionics library).
In order to control the RC or the AX12 servo motor, we use the arduino Micro Pro microcontroller platform, combined with a custom "Shield".

More information about Arduino Micro can be found [here.](http://arduino.cc/en/Main/ArduinoBoardMicro)

# Pinout
The Pin Mapping of the Arduino Micro Pro "Shield".

![PCB](https://raw.github.com/zisi/openBionics/master/Pics/PCB1.png)


# Build of Materials
	Part           Value              Device             Package 
	ARDUINO_MICRO  -                  -		     -
	C1             100u               CPOL-EUE2.5-6      E2,5-6  
	IC1            74LS241N           74LS241N           DIL20 
	J1             -                  HEADER-2X3P-2.54   -
	J2             SCREW-TERMINAL-2P  SCREW-TERMINAL-2P  -
	J3             -                  HEADER-2P-65/35MIL -          
	J4             -                  HEADER-2P-65/35MIL -    
	R1             10k                RESISTOR1206       1206
	R2             10k                RESISTOR1206       1206
	R3             10k                RESISTOR1206       1206
	S1             -                  SWITCH-SPDTSMD2    -  

# Mount PCB in to Robot Hands

You can attach the PCB at the PCBMount.STL part of the robot hands, using two M2x8 screws and two M2 nuts.

![PCBMount](https://raw.github.com/zisi/openBionics/master/Pics/PCB2.jpg)
