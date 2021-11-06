# Arduino-ODrive-CNC
Arduino sketch for my Home Converted CNC Milling Machine, using an Arduino Mega 2560 and the ODrive controller. The Arduino sends commands to the ODrive over serial and using the ASCII protocol.

BLDC_ODrive file is the pre HDMI display sketch for the Arduino Mega 2560, 2 x 16 x 4 LCD displays and 2 x 9-way keypads.

CNC_Control_and_Display sketch controls the Gameduino 3x Dazzler with an HDMI output onto a portrait orientated display, this from a Teensy 4.1 with integrated SD Card reader, although still has a 9-way keypad attached.
