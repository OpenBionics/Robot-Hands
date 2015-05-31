# Electronics

This folder presents the electronics for the robot hands and for the interfaces (the Schematic, the PCB Layout and the OpenBionics library).
In order to control the RC or the AX12 servo motor, we use the arduino Micro Pro microcontroller platform, combined with a custom "Shield".

More information about Arduino Micro can be found [here.](http://arduino.cc/en/Main/ArduinoBoardMicro)

# Pinout
The Pin Mapping of the Arduino Micro Pro "Shield".

<img src="https://raw.github.com/zisi/openBionics/master/Pics/PCB1.png" width="60%" height="60%" />

# Build of Materials of main PCB
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

<img src="https://raw.github.com/zisi/openBionics/master/Pics/PCB2.jpg" width="60%" height="60%" />

# Interfaces

#####RobotHandExtension
This interface requires push buttons and wires with the appropriate terminals.
The push buttons are placed in suitable pockets in RobotHandExtension part. Moreover, the  RobotHandExtension part has the necessary routing for the wires. The open button pin is B1 and it is placed on the top of the RobotHandExtension part. The close button pin is B2 and it is placed on the right hand side of the same part.

######Build of Materials
    2x push buttons
    2x wires terminals for HEADER-2P-65/35MIL (HEADER-2P)

#####SliderBox
This interface requires one arduino board and two linears potentiometers.
The potentiometers are connected to the analog inputs A0 and A1 of arduino board.

######Build of Materials
    2x linear potentiometer 10k (linear)
    1x Arduino board
    1x Jumper Wire Pack 