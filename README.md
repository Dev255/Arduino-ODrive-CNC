# Arduino-ODrive-CNC
Arduino sketch for my Home Converted CNC Milling Machine, using an Arduino Mega 2560 and the ODrive controller. The Arduino sends commands to the ODrive over serial and using the ASCII protocol.

BLDC_ODrive file is the pre HDMI display sketch for the Arduino Mega 2560, 2 x 16 x 4 LCD displays and 2 x 9-way keypads.

CNC_Control_and_Display sketch controls the Gameduino 3x Dazzler with an HDMI output onto a portrait orientated display, this from a Teensy 4.1 with integrated SD Card reader, although still has a 9-way keypad attached.

# Latest Sketch = Ver 1_2_3
CNC Control and Display Ver 1_2_3 now has the following:-

Crash detection - constantly monitoring spindle speed while in GCode Run mode and stopping instantly when this goes below a set percentage (I will be linking this later to tool diameters)

ODrive controlled X and Y movement above 120mm/min - For smoother motion and single line G0 movements (not dog-legged). below 120mm/min is still controlled by the Teensy 4.1 to ensure highly accurate movement (ODrive above 120mm/min still under 0.01mm accuracy on my mill).

Faster program loops - The program loops have been tweaked to run faster for smoother operation and surface finishes

EEPROM non-volatile storage - The user-defined backlash, Max spindle speed and other parameters can now be set and will be stored on the Teensy's EEPROM and recalled during power-up.

Backlash measurement and setting - The machine menu now has an option to measure and store backlash values through simple on-screen instructions and automatic EEPROM updating.

Teensy 4.1 Wiring - All required Teensy pins are now listed at the top of the Sketch (CNC_Control_and_Display_-_Ver1_2_3.ino) with a description of their function.

Teensy 4.1 RTC - The Real Time Clock now holds the correct time updated when connected to a PC - this requires a 3V battery to maintain the time when not connected.

Better Serial diagnosis - To track what the mill is doing and follow the flow of the program (I just thought, I'll have to add a timestamp on the next update).

Keypad keys are not polled - The keypad is no longer polled when the machine runs as this increases loop time (can be re-instated if needed).

Manual stepped spindle speed increase - using extra buttons (which can be mapped to the keypad if you are using only this for control) the spindle speed can be increased by 250 RPM per press, up to its set maximum. this will then continue to spin at this locked speed while doing other operations such as jogging the mill (handy for edge finding at low speed)  or circle milling operations).

Controlled spindle Ramp Up and Down velocities - To ensure less current pulled on increase and passed to the brake resistor/PSU during decrease.

Button LED status indications - My machine now has buttons above with integral LEDs (Red, Green, Amber) to give simple status indications if running the machine with the monitor off.

GCode interpreter improvements - Such as spindle speed ramp up and down when commanded, G17/18/19 inclusion (although no Z-Control at present, working on that very soon). 

GCode pause/stop/safety interlock control - The machine will now instantly stop (even if mid GCode command) and pause until you are ready to carry on, where it will automatically ramp up the spindle speed, dwell for 2 seconds, then carry on from exactly where it left off as if not paused, so you can pause for dinner or tea without worry, or pause until the morning if it gets too late. The machine can also be stopped and taken out of GCode mode and the GCode run from the beginning or new file loaded.

EXTMEM use to store larger GCode Files - If your Teensy 4.1 has extra QSPI memory chips soldered on (16MB Max); this has increased the maximum GCode file size from 300KB to 15MB (theoretically, I still need to find a file big enough to try it).

Simultaneous Calibration - With one button press, all motors will now calibrate simultaneously, getting you up and running faster.

If you need any changes to this, feel free to have a play with the code or ask for the code to be aligned to your machine (such as, separate spindle or different encoders, etc).
