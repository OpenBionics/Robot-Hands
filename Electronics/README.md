# Electronics

This folder include Robot hands electronics (schematic, PCB Layout and openbionics library).
In order to control the RC or AX12 servo motor we use arduino micro pro with a custom "shield".

For more information about [Arduino Micro](http://arduino.cc/en/Main/ArduinoBoardMicro)

# Pinout
Pin Mapping of the Arduino Micro Pro "shield"

![PCB](https://raw.github.com/zisi/openBionics/master/Pics/PCB1.png)


# Build of Materials
	Part           Value              Device           Package 
	ARDUINO_MICRO  -                  -			       -
	C1             100u               CPOL-EUE2.5-6    E2,5-6  
	IC1            74LS241N           74LS241N         DIL20 
	J1             -                  HEADER-2X3P-2.54 -
	J2             SCREW-TERMINAL-2P SCREW-TERMINAL-2P -
	J3             -                  HEADER-2P-65/35MIL -          
	J4             -                  HEADER-2P-65/35MIL -    
	R1            10k                 RESISTOR1206      1206
	R2            10k                 RESISTOR1206      1206
	R3            10k                 RESISTOR1206      1206
	S1            -                   SWITCH-SPDTSMD2   -  

# Mount PCB in to Robot Hands

In order to place the PCB in Robot Hands, you can screw it in the PCBMount.STL part with two M2x8 and M2 nuts.

![PCBMount](https://raw.github.com/zisi/openBionics/master/Pics/PCB2.png)