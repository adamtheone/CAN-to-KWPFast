# CAN-to-KWPFast
This arduino code is converting CAN PID queries to KWPFast quries with the the help of K-Line and CAN interface ICs.
## Motivation
A situation where this can be useful is when there's a CAN "Master" (for example a OBD2 module) that is trying to communicate with a car that doesn't support CAN, however it supports KWPFast (or KWP2000 or K-Line). This code does the translation between the two protocols
## Useage
This code has been tested with Arduino Nano Every. Used libs (and modified parameters if needed):
 - https://github.com/sandeepmistry/arduino-CAN - (MCP2515.h): MCP2515_DEFAULT_CLOCK_FREQUENCY
 - https://github.com/iwanders/OBD9141 - (OBD9141.h): OBD9141_INIT_IDLE_BUS_BEFORE
## Hardware setup
CAN Master <--> MCP2515 (CAN interface) <--> Arduino Nano <--> MC33660BEFR2 (K-Line interface) <--> Car

