/*
Neil Devonshire - Dev255
YouTube Dev255
www.dev255.uk

This program is an on-going project and is still being worked on (updated 16/05/2022) and has not been fully optimized to run at maximum efficiency.

G-Code to CNC sketch to machine features from a main menu system
comments added to help describe the funcion of each statement (if required)

    Copyright (C) <2021>  <Neil Devonshire>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
    
*/

/* -------------------------------------- CNC Control and Display Sketch --------------------------------------------
 * 
 * I like to write code in this way as it aids learning each aspect without having to look through Library functions.
 * 
 * ## THINGS TO IMPLEMENT ##
 * # Put Version on main display or in Machine Menu - DONE
 * # Move all displays slowly to reduce screen burn - Possibly
 * # Fit battery to Teensy 4.1 and created a menu to change the time and date - DONE
 * # Use the RTC to determine Run Time and On Time - If it won't cause lag
 * # Show Run Time on main display
 * # Show On Time on main display
 * # Show Estimated Time on main display, although all GCode will need to be evaluated - Probably not
 * # If not too time consuming, create Line drawing from the GCode file to show what is being milled - Would love to do
 * # Work on Emergency Stop Mode - DONE, Emergency Stop is a Catagory 0 (All Off when button pressed)
 * # Have pause function display "PAUSING" in status and pause instantly - Almost done, issues with unpausing
 * # Setup Warning Pop up if not calibrated - Nice to have
 * # Setup Warning Pop up if not Zeroed - Nice to have
 * # Setup Warning Pop up if tool changed and requires tool legnth setup - Nice to have
 * # Setup Backlash setup in machine menu and get it working - DONE
 * # Use Endstop microswitches as rough Endstop, then use the AMT102 Z pulse to set each axis accurately - Do Endstop switches only as machine will need zeroing
 *    to parts anyway.
 * 
 * ## NOTES ON SKETCH ##
 * # Sketch has been written to read a GCode File creted in Fusion 360, select that file from SD Card and then copied to RAM
 *    so that the mill can be working while SD card is removed and new GCode loaded. The Arduino/Teensy no longer needs to be power cycled.
 * # The Settings can now be changed by the user in Non-Volatile EEPROM so that backlash amounts and maximum speeds can be setup in the
 *    machines menu system; this so backlash can be checked and adjusted easily (this was a long process before)
 */

/* ## GAMEDUINO NOTES ##
 */

/* ## TEENSY 4.1 WIRING ##
 * # Pin 0 - SERIAL 1 (115200bps) - RX from X & Y Axes ODrive (REQUIRED FOR MILL OPERATION)
 * # Pin 1 - SERIAL 1 (115200bps) - TX to X & Y Axes ODrive (REQUIRED FOR MILL OPERATION)
 * # Pin 2 - INPUT_PULLUP - GAMEDUINO interrupt (not yet used - probably won't be used)
 * # Pin 3 - INPUT_PULLUP - X Axis endstop microswitch (not yet connected)
 * # Pin 4 - INPUT_PULLUP - Y Axis endstop microswitch (not yet connected)
 * # Pin 5 - INPUT_PULLUP - Z Axis endstop microswitch (not yet connected)
 * # Pin 6 - OUTPUT - 24V Victron inverter remote on/off (via 5V relay board switching 12V to enable 240V output) - switches on the air blower
 * # Pin 7 - OUTPUT - Cabinet LED's power (via 5V relay board switching 24V)
 * # Pin 8 - OUTPUT - GAMEDUINO GPU select (not yet used - probably won't be used) 
 * # Pin 9 - OUTPUT - GAMEDUINO SD select (not yet used - probably won't be used)
 * # Pin 10 - SPI controlled - CS - GAMEDUINO Dazzler Chip Select (part of MOSI,MISO,CLK,CS - not set with pinMode as controlled by SPI)(REQUIRED FOR MILL OPERATION)
 * # Pin 11 - SPI controlled - MOSI - Master Out, Slave In - Gameduino and NRF24L01 radio are the Slave devices, Teensy is the Master (Dazzler CS = Pin 10, NRFL01 CS = Pin 17)(REQUIRED FOR MILL OPERATION)
 * # Pin 12 - SPI controlled - MISO - Master in, Slave out - Gameduino and NRF24L01 radio are the Slave devices, Teensy is the Master (Dazzler CS = Pin 10, NRFL01 CS = Pin 17)(REQUIRED FOR MILL OPERATION)
 * # Pin 13 - SPI controlled - SCK - Serial Clock - Produced by Teensy to sync communications over MISO,MOSI(REQUIRED FOR MILL OPERATION)
 * # Pin 14 - SERIAL 1 (115200bps) - TX to Z Axis &/or Spindle ODrive (REQUIRED FOR MILL OPERATION - depending on your setup, can be just Z-Axis, although spindle won't be monitored (crash detection))(REQUIRED FOR MILL OPERATION)
 * # Pin 15 - SERIAL 1 (115200bps) - TX from Z Axis &/or Spindle ODrive (REQUIRED FOR MILL OPERATION - depending on your setup, can be just Z-Axis, although spindle won't be monitored (crash detection))(REQUIRED FOR MILL OPERATION)
 * # Pin 16 - OUTPUT - GAMEDUINO Dazzler Reset - Set LOW for 500ms during switch on for a fresh reset of the Gameduino 3X Dazzler board, may occasionally need the connected monitor power cycled at switch on as it can hang(REQUIRED FOR MILL OPERATION)
 * # Pin 17 - OUTPUT - CE - NRF24L01 Chip Enable, this set to LOW as connected but not yet used (NOT yet required for mill operation, will be for remote control if required)
 * # Pin 18 - SDA - I2C communication - Connected to Adafruit 8x8 Matrix, can have more devices (NOT required for mill operation)
 * # Pin 19 - SCL - I2C communication - Connected to Adafruit 8x8 Matrix, can have more devices (NOT required for mill operation)
 * # Pin 20 - SERIAL 5 (115200bps) - TX to Mill remote control, just for when NRF24L01 is not in use (i.e. remote battery dead)(NOT required for mill operation as KeyPad can be used instead)
 * # Pin 21 - SERIAL 5 (115200bps) - RX from Mill remote control, just for when NRF24L01 is not in use (i.e. remote battery dead)(NOT required for mill operation as KeyPad can be used instead)
 * # Pin 22 - INPUT_PULLUP - Remote control Wire Sense (will be used to detect when the remote is hard wired so that the NRF24L01 radio can be disabled)(NOT required for mill operation)
 * # Pin 23 - INPUT_PULLUP - Remote control Data Available flag (will be used to flag when there is new data from the remote control)(NOT required for mill operation)
 * # Pin 24 - INPUT_PULLUP - Keypad matrix Y1 (REQUIRED FOR MILL OPERATION - if no other control input)
 * # Pin 25 - INPUT_PULLUP - Keypad matrix Y2 (REQUIRED FOR MILL OPERATION - if no other control input)
 * # Pin 26 - INPUT_PULLUP - Keypad matrix Y3 (REQUIRED FOR MILL OPERATION - if no other control input)
 * # Pin 27 - INPUT_PULLUP - Keypad matrix Y4 (REQUIRED FOR MILL OPERATION - if no other control input)
 * # Pin 28 - OUTPUT - Keypad matrix X1 (REQUIRED FOR MILL OPERATION - if no other control input)
 * # Pin 29 - OUTPUT - Keypad matrix X2 (REQUIRED FOR MILL OPERATION - if no other control input)
 * # Pin 30 - OUTPUT - Keypad matrix X3 (REQUIRED FOR MILL OPERATION - if no other control input)
 * # Pin 31 - OUTPUT - Cabinet fans via ULN2003A (darlinton pair array), used to easily control higher voltage, higher current devices. HIGH turns fans on.
 * # Pin 32 - NOT USED - Edge overhangs case on my setup to allow SD card to be removed
 * # Pin 33 - NOT USED - Edge overhangs case on my setup to allow SD card to be removed
 * # Pin 34 - INPUT_PULLUP - Safety interlock on mill cabinet door (reed switch on cabinet and magnet fixed to door)(NOT required for mill operation, although good safety addition)
 * # Pin 35 - INPUT_PULLUP - Start button input (NOT required for mill operation, although a lot easier to control)
 * # Pin 36 - OUTPUT - Start LED (green) via ULN2003A (darlington pair array) as LED's are 12V. HIGH turns LED on (NOT required for mill operation, although a lot easier to control)
 * # Pin 37 - SPI controlled - CS - NRF24L01 radio transiever Chip Select (part of MOSI,MISO,CLK,CS - not set with pinMode as controlled by SPI) (NOT required for mill operation, although a lot easier to control)
 * # Pin 38 - OUTPUT - Pause LED (amber) via ULN2003A (darlington pair array) as LED's are 12V. HIGH turns LED on (NOT required for mill operation, although a lot easier to control)
 * # Pin 39 - INPUT_PULLUP - Pause button input (NOT required for mill operation, although a lot easier to control)
 * # Pin 40 - OUTPUT - Stop LED (red) via ULN2003A (darlington pair array) as LED's are 12V. HIGH turns LED on (NOT required for mill operation, although a lot easier to control)
 * # Pin 41 - INPUT_PULLUP - Stop button input (NOT required for mill operation, although a lot easier to control)
 */
 
// Library header files to write to Teensy/Arduino
#include <DS1307RTC.h>              // Real Time Clock (RTC) on Teensy 4.1 Library
#include <SPI.h>                    // Standard Serial Periferal Interface Library (Built in)
#include <GD2.h>                    // Gameduino 3x Dazzler Library (https://excamera.com/sphinx/gameduino2/code.html#with-the-arduino-ide)
#include <SD.h>                     // Standard SD Card Library (built in)
#include <ODriveArduino.h>          // ODrive Motor Controller Library (https://docs.odriverobotics.com/ascii-protocol)
#include <Wire.h>                   // For I2C communication
#include <Adafruit_GFX.h>           // 8 x 8 Adafruit Dot Matrix Display
#include <Adafruit_LEDBackpack.h>   // 8 x 8 Adafruit Dot Matrix Display
#include <Math.h>                   // For more complicated Math calculations
#include <EEPROM.h>                 // To store non-volatile user settings to EEPROM (do not write to constantly as this will degrade over time)
#include <TimeLib.h>                // Real Time Clock (RTC) on Teensy 4.1 Library

// Sketch Version for Machine Menu Display
const char *cncArduinoSketchVer[] = {"CNC Control and Display - Ver 1_2_3"}; // ##### ANNOTATE VERSION HERE TO SHOW IN THE MACHINE MENU #####

// Teensy Non-Volatile EEPROM Memory Storage for User Adjusted Machine Settings and controlling Boolean
struct SettingsObject {short xBacklash_Set; short yBacklash_Set; long backlashSpeed_Set; short spindleRPM_Max_Set; long jogSpeed_Max_Set;};

// Constants for machine/Teensy configuration
const int feedSpeedAdjust_Cir = 200;// Affects mm/m Feed Speed within for Arc Moves
const int feedSpeedAdjust = 16384;   // Affects mm/m Feed Speed within GlobalMove(). A higher value is slower
const unsigned int loopTimeDisplayRefresh = 200; // Sets how fast the loop time is updated, currently set to 0.2 Seconds
const unsigned int screenRefreshDelay = 50;  // Only draws the main screen every 50 milliseconds (20Hz refresh) to speed up Axes update and smooth movement control
short spindleRPM_Max = 4860;        // Determined by spindle motor ODrive Voltage * 0.75 * Motor KV (set at 24V)
const int spindleCPR = 8192;        // Spindle encoder counts per revolution
const int speedStep = 250;          // RPM increase amount when manually increased
float crashFactor = 0.9f;           // Initial value for monitoring spindle crashes, i.e 40mm cutter at 4000RPM will be stopped if the spindle drops below 3600RPM, higher values for smaller cutters
const int gcLinesToDisplay = 19;    // Maximum number of GCode lines to display
const int gcCharsToDisplay = 150;   // Maximum number of GCode characters to display (does not include spaces)
const int gameduinoInterrupt = 2;
const int endStop_X = 3;
const int endStop_Y = 4;
const int endStop_Z = 5;
const int inverter = 6;
const int cabinetLED = 7;
const int gameduinoGPUsel = 8;
const int gameduinoSDsel = 9;
const int dazzlerRESET = 16;
const int nrf24L01_CE = 17;
const int remoteControl_WS = 22;
const int remoteControl_DA = 23;
const int keyPadY1 = 24;            // Keypad matrix Y1 connected to Teensy 4.1 pin 24
const int keyPadY2 = 25;            // Keypad matrix Y2 connected to Teensy 4.1 pin 25
const int keyPadY3 = 26;            // Keypad matrix Y3 connected to Teensy 4.1 pin 26
const int keyPadY4 = 27;            // Keypad matrix Y4 connected to Teensy 4.1 pin 27
const int keyPadX1 = 28;            // Keypad matrix X1 connected to Teensy 4.1 pin 28
const int keyPadX2 = 29;            // Keypad matrix X2 connected to Teensy 4.1 pin 29                                                                                                                                                                                                                                                                     
const int keyPadX3 = 30;            // Keypad matrix X3 connected to Teensy 4.1 pin 30
const int cabinetFans = 31;
const int safetyInterlock = 34;
const int startButton = 35;
const int startLED = 36;
const int pauseLED = 38;
const int pauseButton = 39;
const int stopLED = 40;
const int stopButton = 41;

// Input Variables
int startPress = HIGH;              // LOW = Start Button being pressed
int pausePress = HIGH;              // LOW = Pause Button being pressed
int stopPress = HIGH;               // LOW = Stop Button being pressed
int doorOpen = HIGH;                // LOW = Mill door closed or interlock defeated
int axisLimit_X = HIGH;             // LOW = X axis Endstop reached (Not yet installed)
int axisLimit_Y = HIGH;             // LOW = Y axis Endstop reached (Not yet installed)
int axisLimit_Z = HIGH;             // LOW = Z axis Endstop reached (Not yet installed)
int startLEDMode = 5;               // Mode 0 = LED Off, 1 = On, 2 = Slow Flash, 3 = Flash Fast, 4 = Blink Off, 5 = Blink On, 6 = Slow Flash Inverted
int pauseLEDMode = 0;               // Mode 0 = LED Off, 1 = On, 2 = Slow Flash, 3 = Flash Fast, 4 = Blink Off, 5 = Blink On, 6 = Slow Flash Inverted
int stopLEDMode = 4;                // Mode 0 = LED Off, 1 = On, 2 = Slow Flash, 3 = Flash Fast, 4 = Blink Off, 5 = Blink On, 6 = Slow Flash Inverted
bool spindleDropout = false;        // FLAG - true = spindle crash detected (spindle RPM dropped below crash factor)
int sequenceLEDs = 0;               // Used to control and sync LED & on-screen LED flash timing

// Program Flags
bool backlashAdjust = false;        // FLAG - true = write new backlash values to EEPROM once new values measured
bool nextTool = true;               // FLAG - Used to pause GCode and IDLE spindle to allow a Tool Change     
bool updateEEPROM = false;          // FLAG - New data to write to EEPROM
bool oDrive0m0CLC = false;          // FLAG - Is ODrive 0, motor 0 (Y-Axis) in Closed Loop Control
bool oDrive0m1CLC = false;          // FLAG - Is ODrive 0, motor 1 (X-Axis) in Closed Loop Control
bool oDrive1m0CLC = false;          // FLAG - Is ODrive 1, motor 0 (Spindle) in Closed Loop Control
bool oDrive1m1CLC = false;          // FLAG - Is ODrive 1, motor 1 (Z-Axis) in Closed Loop Control
bool cncEnd = false;                // FLAG - Used to escape out of GCode loop (from pause or M30/%)
bool gcodeToRAMDisplay = false;     // FLAG - true when a GCode file is loaded to EXTernal RAM
bool gCodePause = false;            // FLAG - true = gCode paused 
bool gCodeStartFromButton = false;  // FLAG - true = Start button held to start CNC mode from loaded GCode

// Memory Files
EXTMEM String memoryFile = "";      // Huge String to contain the GCode file to speed up writing to display
File gcFile;                        // Current GCode file object
File settingsFile;                  // Current Settings file object - Not yet used - will be linked to "Settings.TXT"
File rootDir;

// SD Card variables
const int chipSelect = BUILTIN_SDCARD; // Chip select value for the Teensy 4.1 SD Card
bool sDcardInserted = false;
int fileCount = 0;                  // Used to count number of files on the SD Card
unsigned long memoryFileLength = 0UL; // Used to count all characters into memoryFile when copying from SD to RAM
const unsigned long maxFileLength = 10000000UL; // maximum GCode file size to store in RAM (Bytes)
bool fileCopied = false;            // Used to prevent multiple files being copied in memory within subdirectories on SD Card

// Clock variables
int yr = 1977;                      // Temporary Year for the Teensy 4.1 Real Time Clock (RTC)
int mon = 3;                        // Temporary Month for the Teensy 4.1 (RTC)
int dy = 24;                        // Temporary Day for the Teensy 4.1 (RTC)
int hr = 0;                         // Temporary Hour for the Teensy 4.1 (RTC)
int mnt = 0;                        // Temporary Minute for the Teensy 4.1 (RTC)
int sec = 0;                        // Temporary Second for the Teensy 4.1 (RTC)
int rot = 0;                        // Rotate value for the Branding Bitmap (Dev255)

// Keypad variables
int keyVal = 12;                    // Current value of the key pressed, [0] to [9] for numbers, 10 for [*], 11 for [#] and 12 for 'no key pressed'
double keypadEnteredValue = 0.0d;   // This is the user entered number temporary storage aera, including decimal point.

//Menu variables
int menuSelect = 0;                 // Current menu to display on main screen, 0 = Main window with GCode, 1 = Machine Menu, 2 = Work Menu
int menu00Pos = 20;                 // Starting position of Menu 0, 20 is on screen and drawn
int menu01Pos = 760;                // Starting position of Menu 1, > 760 is off screen and not drawn
int menu02Pos = 740;                // Starting position of Menu 2, > 740 is off screen and not drawn
bool menu000 = true;                // Control logic for Menu 0 (main window)
bool menu001 = false;               // Control logic for Menu 1
bool menu002 = false;               // Control logic for Menu 2

// Window/Text/Graphic positions
int axDispHor = 0;                  // Horizontal position of the Axes display panel - linked to and moves with X-Axis 
int axDispVer = 0;                  // Vertical position of the Axes display panel - linked to and moves with Y-Axis
float clDispHor = 45.0f;            // Clock window Horizontal position - moves around to reduce screen burn
float clDispVer = 1235.0f;          // Clock window Vertical position - moves around to reduce screen burn
bool clDispVerDown = true;          // Used to control Vertical direction of Clock window
bool clDispHorRight = true;         // Used to control Horizontal direction of Clock window
int horPos = 10;                    // For SD File structure positioning when copying to RAM
int verPos = 10;                    // For SD File structure positioning when copying to RAM

// Timing variables
int loopTimeDisplay = 0;            // Stores time taken to run through 1 Main loop in all modes, measured in microseconds
int loopTime = millis();            // Used to calculate loopTimeDisplay above
unsigned int loopTimeFunction = millis(); // Used to calculate the time taken to press and hold keypad keys (for fast scroll functions)
unsigned int loopTimeDispDelay = millis();   // Used to time refresh rate of displayed loop time during encoder updates
unsigned int drawMainScreenDelay = millis(); // Used to time screen refresh rate during encoder updates
long sTime = millis();              // Used to determine spindle rotations over exact time in milliseconds 
long sTimeNow = millis();           // Used to determine spindle rotations over exact time in milliseconds
int sTimeElapsed = 0;               // Used to determine spindle rotations over exact time in milliseconds
unsigned long dev255LoopTime1 = 0UL; // Used to rotate 8x8 Matrix every 10 seconds
unsigned long dev255LoopTime2 = 0UL; // Used to rotate 8x8 Matrix every 10 seconds
byte dev255LoopCount = 2;           // Used to rotate 8x8 Matrix every 10 seconds
unsigned long globalMoveLoopTime = 0UL; // Used to determine step amount from elapsed time to ensure feed rate is maintained
unsigned long timeElapsed = 0UL;    // Used to determine step amount from elapsed time to ensure feed rate is maintained
unsigned int spindleTime = 0U;      // TEMPORARY timing to test spindle speeds over time

// Strings for Status Displays and their contolling integers
const char *cncModeStrings[] = {"G-CODE MANUAL Z","MANUAL CIRCLE","MANUAL LINE","MANUAL RECTANGLE","AXIS JOG"};
const char *cncStatStrings[] = {"Calibration Required","Calibrating","Idle","Running G-Code","Paused","Tool Change","EMERGENCY STOP","PAUSING"}; // If cncStatStrings [2], then "Idle" is displayed on screen
const char *dazStatStrings[] = {"720 x 1280 @ 60Hz","Error"}; // If dazStatStrings [0], then "720 x 1280 @ 60Hz" is displayed on screen
const char *tnyStatStrings[] = {"Loop microsec =","SD Card Full","Holding ODrive Reset"}; // If tnyStatStrings [1], then "SD Card Full" is displayed on screen
const char *oDvStatStrings[] = {"1/All Calibration Rqd","Spindle IDLE","Calibrating X Axis","Calibrating Y Axis","Calibrating Z Axis","Spindle Ready","Calibrating All","Closed Loop Control","Error","Paused"};                         // If oDvStatStrings [3], then "Closed Loop Control" is displayed on screen
const char *nextToolStrings[] = {"None","Pen","40mm Shell","10mm End","8mm End"};
int cncModeVal = 0;                 // Used in conjunction with cncModeStrings above to control the string that is displayed next to the Program Status Window
int cncStatVal = 0;                 // Used in conjunction with cncStatStrings above and cncStatLED below to control the string that is displayed and LED colour, i.e. cncStatStrings[cncStatVal]
int dazStatVal = 0;                 // Used in conjunction with dazStatStrings above and dazStatLED below to control the string that is displayed and LED colour, i.e. dazStatStrings[dazStatVal]
int tnyStatVal = 0;                 // Used in conjunction with tnyStatStrings above and tnyStatLED below to control the string that is displayed and LED colour, i.e. tnyStatStrings[tnyStatVal]
int oDvStatVal = 0;                 // Used in conjunction with oDvStatStrings above and oDvStatLED below to control the string that is displayed and LED colour, i.e. oDvStatStrings[oDvStatVal]
int xStatVal = 0;                   // Used in conjunction with xyzAxesCharLED below to control the LED colour, i.e. xyzAxesCharLED[xStatVal]
int yStatVal = 0;                   // Used in conjunction with xyzAxesCharLED below to control the LED colour, i.e. xyzAxesCharLED[yStatVal]
int zStatVal = 0;                   // Used in conjunction with xyzAxesCharLED below to control the LED colour, i.e. xyzAxesCharLED[zStatVal]
int axisXstate = 0;                 // Used in conjunction with oDvStatStrings above and oDvStatLED below to control the string that is displayed and LED colour, i.e. oDvStatStrings[axisXstate]
int axisYstate = 0;                 // Used in conjunction with oDvStatStrings above and oDvStatLED below to control the string that is displayed and LED colour, i.e. oDvStatStrings[axisYstate]
int axisZstate = 0;                 // Used in conjunction with oDvStatStrings above and oDvStatLED below to control the string that is displayed and LED colour, i.e. oDvStatStrings[axisZstate]
int axisSstate = 0;                 // Used in conjunction with oDvStatStrings above and oDvStatLED below to control the string that is displayed and LED colour, i.e. oDvStatStrings[axisSstate]

// LED Colours and Control
const int cncStatLED[] = {0xff8000,0xffff00,0x008000,0x00ff00,0xffff00,0xff8000,0xff0000}; // LED colour for CNC status
const int dazStatLED[] = {0x00ff00,0xff0000}; // LED colour for Gameduino 3x Dazzler status
const int tnyStatLED[] = {0x00ff00,0xff0000,0xff0000}; // LED colour for Teensy 4.1 status
const int oDvStatLED[] = {0xff8000,0xffff00,0xffff00,0xffff00,0xffff00,0x00ff00,0xffff00,0x00ff00,0xff0000,0xffff00}; // LED colour for ODrive motor contoller status
const int xyzAxesCharLED[] = {0x00ffff,0xff00ff,0xff0000}; // LED colour for X,Y & Z button status
int cncLEDSize = 75;                // Size of LED controlled to show (75) or not show (0) LED colour to mimic flashing
int dazLEDSize = 75;                // Size of LED controlled to show (75) or not show (0) LED colour to mimic flashing
int tnyLEDSize = 75;                // Size of LED controlled to show (75) or not show (0) LED colour to mimic flashing
int oDvLEDSize = 75;                // Size of LED controlled to show (75) or not show (0) LED colour to mimic flashing
int xDialLEDSize = 528;             // Size of LED controlled to show (528) or not show (0) LED colour to mimic flashing
int yDialLEDSize = 528;             // Size of LED controlled to show (528) or not show (0) LED colour to mimic flashing
int zDialLEDSize = 528;             // Size of LED controlled to show (528) or not show (0) LED colour to mimic flashing
unsigned int loopLEDflash = 0;      // Used to count milliseconds to flash LED once per second
bool ledOn = false;                 // Logic true if LED on
bool ledFastOn = false;             // Logic true if LED on
bool ledBlinkOn = false;            // Logic true if LED on

// GCode Variables and boolean
int gcTextVer = 0;                  // GCode Vertical start position, value changes with main window as it moves around 
int gcTextHor = 0;                  // GCode Horizontal start position, value changes with main window as it moves around
int gcTextLine = 0;                 // Used to control maximum number of lines to display on screen to prevent 'display list overflow' errors on Dazzler as only 2048 instructions per frame allowed
int gcCharCount = 0;                // Used to control maximum number of characters to display on screen to prevent 'display list overflow' errors on Dazzler as only 2048 instructions per frame allowed
int gcCurrLineNo = 0;               // Current GCode Line number chosen, stepped by user to look at each line. This will be used to step through GCode on CNC when sketches merged
int gcLiveCharPos = 0;              // Used to track GCode characters. This will be used to determine GCode Mode (G00,G01,G02,etc, X,Y,Z values and all other control for the CNC machine to use once sketches merged
int gcCurrLSP = 0;                  // Current Line Starting Point (LSP) that writes to the display from this line forward
int gcASCIIval = 0;                 // Current ASCII value read from the GCode file before being processed by this sketch
int gCodeMode = 100;                // Reflects the current GCode for GCode Run mode, so G00 = 0, G01 = 1, etc
int nextToolval = 0;                // Tool to change to, will need a tool list in the machine menu and char string to hold tools
int readCount = 0;                  // Used in GCode run to step through each character in the GCode file in RAM
double preX = 0.0d;                 // Previous X-Axis destination (current position), used in Absolute mode to add or subtract the new X value from
double preY = 0.0d;                 // Previous Y-Axis destination (current position), used in Absolute mode to add or subtract the new Y value from
double preZ = 0.0d;                 // Previous Z-Axis destination (current position), used in Absolute mode to add or subtract the new Z value from
double x = 0.0d;                    // New X-Axis destination value
double y = 0.0d;                    // New Y-Axis destination value
double z = 0.0d;                    // New Z-Axis destination value
double a = 0.0d;                    // New A-Axis destination value
double b = 0.0d;                    // New B-Axis destination value
double c = 0.0d;                    // New C-Axis destination value
double h = 0.0d;                    // Tool height offset in mm
double i = 0.0d;                    // Arc/Circle center in X-Axis
double j = 0.0d;                    // Arc/Circle center in Y-Axis
double k = 0.0d;                    // Arc/Circle center in Z-Axis
double r = 0.0d;                    // Radius of an arc (Not yet used in this sketch)
int f = 0;                          // Feed speed in mm/min
int p = 0;                          // Pause amount in seconds
//double q = 0.0d;
int s = 0;                          // Spindle speed set
double v = 0.0d;                    // Temporary Axis value while building float from characer Strings
int planeSelect = 17;               // 17 = X,Y   18 = Z,   19 = Z,
bool absArcCenMode = false;         // GCode Absolute Arc Mode
bool absPosMode = false;            // GCode Absolute Position Mode
String gcFirstString = "";
String gcCurrentString = "";

// VARIOUS VARIABLES FOR EACH AXIS
long backLashSpeed = 200000L;       // Constant - Speed for taking up backlash (X & Y axes affected at present), no backlash control on Z axis
float feedSpeed = 0.0f;             // Used in Circle Mode and eventually in Line Mode

// X-Axis variables
long xPos_m1 = 0L;                  // Commanded position of X-Axis Motor in encoder counts
long xEnc_m1 = 0L;                  // Current Estimated encoder counts of X-Axis rotary encoder
long xTab_m1 = 0L;                  // Current table position in encoder counts
long xDisplayOffset = 0L;           // Difference between acual Table position (xTab_m1) and displayed position for X-Axis in encoder counts
long xDisplay = 0L;                 // Displayed position of X-Axis in encoder counts, offset from xTab_m1 when X axis zeroed
long xDisplayAbs = 0L;              // Displayed Absolute (machine) position of X-Axis in encoder counts, never offset from xTab_m1
double x_mmDisplay = 0.0d;          // Displayed position of X-Axis in mm, offset from xTab_m1 when X axis zeroed
double x_mmDisplayAbs = 0.0d;       // Displayed Absolute (machine) position of X-Axis in mm, never offset from xTab_m1
long xPause_m1 = 0L;                // Used to store exact X position while pausing

// Y-Axis variables
long yPos_m0 = 0L;                  // Commanded position of Y-Axis Motor in encoder counts
long yEnc_m0 = 0L;                  // Current Estimated encoder counts of Y-Axis rotary encoder
long yTab_m0 = 0L;                  // Current table position in encoder counts
long yDisplayOffset = 0L;           // Difference between actual Table position (yTab_m0) and displayed position for Y-Axis in encoder counts
long yDisplay = 0L;                 // Displayed position of Y-Axis in encoder counts, offset from yTab_m0 when Y axis zeroed
long yDisplayAbs = 0L;              // Displayed Absolute (machine) position of Y-Axis in encoder counts, never offset from yTab_m0
double y_mmDisplay = 0.0d;          // Displayed position of Y-Axis in mm, offset from yTab_m0 when Y axis zeroed
double y_mmDisplayAbs = 0.0d;       // Displayed Absolute (machine) position of Y-Axis in mm, never offset from yTab_m0
long yPause_m0 = 0L;                // Used to store exact Y position while pausing

// Z-Axis variables
long zPos_m1 = 0L;                  // Commanded position of Z-Axis Motor in encoder counts
long zEnc_m1 = 0L;                  // Current Estimated encoder counts of Z-Axis rotary encoder
long zTab_m1 = 0L;                  // Current head position in encoder counts
long zDisplayOffset = 0L;           // Difference between actual head position (zTab_m1) and displayed position for Z-Axis in encoder counts
long zDisplay = 0L;                 // Displayed position of Z-Axis in encoder counts, offset from zTab_m1 when Z axis zeroed
long zDisplayAbs = 0L;              // Displayed Absolute (machine) position of Z-Axis in encoder counts, never offset from zTab_m1
double z_mmDisplay = 0.0d;          // Displayed position of z-Axis in mm, offset from zTab_m1 when Z axis zeroed
double z_mmDisplayAbs = 0.0d;       // Displayed Absolute (machine) position of Z-Axis in mm, never offset from zTab_m1
long zPause_m1 = 0L;                // Used to store exact Z position while pausing

// Spindle variables
int sEncCount = 0;
long sEnc_m0_New = 0L;
long sEnc_m0_Old = 0L;
long sEnc_m0 = 0L;
int spindleMode = 0;
int adequateRPM = 0;
int spindleRPM = 0;                 // Spindle RPM value. This will be linked to actual spindle speed once CNC sketch merged
double spinMotBmpRot = 0.0d;        // Motor Bitmap rotation value, value changes angle of Bitmap to show spindle motor spinning at different RPM's
int spindleSetSpeed = 2000;         // Used to command initial spindle speed
// long sPos_m0 = 0;                    // Commanded position of Spindle Motor in encoder counts, not used in Velocity Mode, may be used for Tapping position control and feel
int velocityMax = 0;

// X-Axis Backlash variables
short xBacklash = 0;                // Number of encoder counts to remove backlash, tune on table with .pos_gain from ODriveTools or via Machine Menu. Settings Stored in EEPROM (see above).
bool xBacklashIn = false;           // Set to true when X-Axis moving in plus (count increases) direction.
int xBacklash_mm = 0;
int xBacklash_New = 0;

// Y-Axis Backlash variables
short yBacklash = 0;                 // Number of encoder counts to remove backlash, tune on table with .pos_gain from ODriveTools or via Machine Menu. Settings Stored in EEPROM (see above).
bool yBacklashIn = false;            // Set to true when Y-Axis moving in plus (count increases) direction.
int yBacklash_mm = 0;
int yBacklash_New = 0;

// Z-Axis Backlash variables
short zBacklash = 0;                 // Number of encoder counts to remove backlash, tune on table with .pos_gain from ODriveTools or via Machine Menu. Settings Stored in EEPROM (see above).
bool zBacklashIn = false;            // Set to true when Z-Axis moving in plus (count increases) direction.
int zBacklash_mm = 0;
int zBacklash_New = 0;

//Global Move Function values
long xStepIncrement = 0L;
long xCNC_m1 = 0L;
long yStepIncrement = 0L;
long yCNC_m0 = 0L;
long zStepIncrement = 0L;
long zCNC_m1 = 0L;

// Circle function values and flags
bool internalCircle = true;
float circleDiameter = 0.0f;
float circleDepth = 0.0f;
float toolDiameter = 0.0f;
bool climbMill = true;
bool toolSet = false;
bool circleSet = false;
bool speedSet = false;
bool zeroSet = false;
bool toolDiaSet = false;
bool climbSet = false;
bool intSet = false;
bool circleDiaSet = false;
bool depSet = false;
bool xZero = false;
bool yZero = false;
bool zZero = false;
bool keypadValRequired = false;

// Jog function values and flags
double mmJogIncr = 1.0d;
long jogIncr = 16384L;
long yJogPos_m0 = 0L;
long xJogPos_m1 = 0L;
long jogSpeed = 300000L;
bool xfirstLoop = true;
bool yfirstLoop = true;
long xyzSpeed = 0L; // Value in counts/second
double circleXcoord = 0.0d;
double circleYcoord = 0.0d;
bool cirXset = false;
bool cirYset = false;
bool circleReady = false;
bool displayEnter = false;
int xyzSelect = 4;
int highlightHor = 0;
String combineChars = "";
String currentFeature = "";
bool zManualMove = false;
bool displayZAxisSet = false;
bool gCodeDwell = false;
int gCodeDwellVal = 0;
//String gcRunningString = "";
//char gcRunningCharArray[50];

// Adafruit 8x8 matrix and Constants
Adafruit_8x8matrix matrix = Adafruit_8x8matrix();
static const uint8_t PROGMEM
  dev255[] =
  { B00111000,
    B01111110,
    B11111111,
    B10111101,
    B10001001,
    B01101010,
    B00011100,
    B00000000 },
  emergencyStop[] =
  { B00111100,
    B01000110,
    B10001111,
    B10011101,
    B10111001,
    B11110001,
    B01100010,
    B00111100 },
  softStopY[] =
  { B11111111,
    B10000001,
    B10111101,
    B10100101,
    B10111101,
    B10000001,
    B10000001,
    B11111111 },
  millCircle[] =
  { B00111100,
    B01000010,
    B10011001,
    B10100101,
    B10100101,
    B10011001,
    B01000010,
    B00111100 },
  moveTo[] =
  { B11100111,
    B11000011,
    B10100101,
    B00000000,
    B00000000,
    B10100101,
    B11000011,
    B11100111 };

HardwareSerial & odrive_serial = Serial1; // Set the ODrive Serial Port to Serial 1
HardwareSerial & odrive_serial3 = Serial3; // Set the ODrive Serial Port to Serial 3

// Printing with stream operator - ODrive requirement for ASCII Protocol
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, double arg) { obj.print(arg, 4); return obj; }

ODriveArduino odrive(odrive_serial); // ODrive object
ODriveArduino odrive1(odrive_serial3); // ODrive object

void setup(){ //------------------------------------------------------------------------------------------------------------void setup()---
  setSyncProvider(getTeensy3Time);
  
  matrix.begin(0x70);  // Pass in the Adafruit 8x8 Matrix Address
  matrix.setBrightness(0);

  odrive_serial.begin(115200); // Serial Port 1 to 12V ODrive = 115200 baud
  odrive_serial3.begin(115200); // Serial Port 3 to 24V ODrive = 115200 baud
  
  Serial.begin(115200); // Serial speed to PC from Teensy 4.1
  Serial.println("Serial Started @ 115200 bps"); // Written to confirm Serial is working

  // Gameuino Reset - Pin 16 linked to Gameduino 3x Dazzler RESET pin to reset the Dazzler after powering on
  pinMode(dazzlerRESET, OUTPUT);
  digitalWrite(dazzlerRESET, LOW);
  delay(500);
  digitalWrite(dazzlerRESET, HIGH);
  delay(1000);

  // Set ODrive limits
  odrive_serial << "w axis" << 0 << ".motor.config.current_lim " << 40.0f << '\n';  // Y-Axis Motor
  odrive_serial << "w axis" << 0 << ".controller.config.vel_limit " << 36028.0f << '\n';  // Y-Axis Motor
  odrive_serial << "w axis" << 1 << ".motor.config.current_lim " << 40.0f << '\n';  // X-Axis Motor
  odrive_serial << "w axis" << 1 << ".controller.config.vel_limit " << 36028.0f << '\n';  // X-Axis Motor
  //odrive_serial3 << "w axis" << 0 << ".controller.config.control_mode " << CONTROL_MODE_VELOCITY_CONTROL << '\n';
  //odrive_serial3 << "w axis" << 0 << ".controller.config.vel_ramp_rate" << 0.5f << '\n';
  // Dispaly Dev255 logo on Adafruit Matrix
  matrix.clear();
  matrix.setRotation(2);
  matrix.drawBitmap(0, 0, dev255, 8, 8, LED_ON);
  matrix.writeDisplay();

  dev255LoopTime1 = millis();
  dev255LoopTime2 = millis();
  
  GD.begin(); // Start Dazzler
  GD.cmd_setrotate(2); // Rotate display to Portrait
  GD.ClearColorRGB(0x000000); // Colour to Clear all pixels to
  GD.Clear(); // Wipe the display with the Clear colour
  GD.VertexFormat(0); // points are scaled to screen pixels
  GD.cmd_text(10,10, 16, OPT_CENTERY, cncArduinoSketchVer[0]);
  if(SD.begin(chipSelect)){ // Start the SD Card and display on screen if Ok
    GD.cmd_text(10,20, 16, OPT_CENTERY, "SD Card Started ok");
    sDcardInserted = true;
  }else{
    GD.cmd_text(10,20, 16, OPT_CENTERY, "SD Card Not Found");
    sDcardInserted = false;
  }
  GD.swap(); // Draw the Display List to the display
  // Images are all pre preped and re-sized to ensure images are 1 pixel per display pixel 1:1, this to save Dazzler memory
  GD.BitmapHandle(0);
  GD.cmd_loadimage(0, 0);
  GD.load("dev255.png"); // Branding Image - Load to Dazzler memory

  GD.BitmapHandle(1);
  GD.cmd_loadimage(-1, 0); // -1 loads image to next available (concurrent) and tracked memory location
  GD.load("DialBevl.png"); // Axis Dial Bevel Image - Load to Dazzler memory

  GD.BitmapHandle(2);
  GD.cmd_loadimage(-2, 0);
  GD.load("DialBack.png"); // Axis Rotating Dial Back Image - Load to Dazzler memory

  GD.BitmapHandle(3);
  GD.cmd_loadimage(-3, 0);
  GD.load("DialButt.png"); // Axis Rotating Dial Button Image - Load to Dazzler memory

  GD.BitmapHandle(4);
  GD.cmd_loadimage(-4, 0);
  GD.load("BrushedA.jpg"); // Brushed Aluminium Image for panel backing - Load to Dazzler memory

  GD.BitmapHandle(5);
  GD.cmd_loadimage(-5, 0);
  GD.load("RGBLED.png"); // Unlit NeoPixel LED Image - Load to Dazzler memory

  GD.BitmapHandle(6);
  GD.cmd_loadimage(-6, 0);
  GD.load("AluFrm2.png"); // Brushed Aluminium Frame Image - Load to Dazzler memory - Ceter removed and corners rounded on Windows Paint 3D (with transparrent background to allow silloette to function on Axes display panel)
  //GD.load("DROBkdrp.jpg"); // Brushed Aluminium Backdrop with BallScrews to reduce number of images drawn - Load to Dazzler memory - Windows Paint 3D (with transparrent background)

  GD.BitmapHandle(7);
  GD.cmd_loadimage(-7, 0);
  GD.load("BallScrw.png"); // Ball Screw X Image - Load to Dazzler memory
  //GD.load("DROMvng.jpg"); // Ball Screw X Image - Load to Dazzler memory

  GD.BitmapHandle(8);
  GD.cmd_loadimage(-8, 0);
  GD.load("BalScrwY.png"); // Ball Screw Y Image (pre-rotated to save Dazzler display list rotation) - Load to Dazzler memory

  GD.BitmapHandle(9);
  GD.cmd_loadimage(-9, 0);
  GD.load("StatorSm.png"); // Spindle Motor Stator Image - Load to Dazzler memory

  GD.BitmapHandle(10);
  GD.cmd_loadimage(-10, 0);
  GD.load("RotorSm.png"); // Spindle Motor Rotor Image - Load to Dazzler memory

  GD.BitmapHandle(11);
  GD.cmd_loadimage(-11, 0);
  GD.load("RGBLED.jpg"); // Unlit NeoPixel LED Image - Load to Dazzler memory, loaded again and called, although not drawn, to prevent previous image corruption (may be something I programmed)

  // Set the KeyPad Y matrix pins to Input (with pull up resistors enabled) and X matrix pins to Output mode
  pinMode(keyPadY1, INPUT_PULLUP);
  pinMode(keyPadY2, INPUT_PULLUP);
  pinMode(keyPadY3, INPUT_PULLUP);
  pinMode(keyPadY4, INPUT_PULLUP);
  pinMode(keyPadX1, OUTPUT);
  pinMode(keyPadX2, OUTPUT);
  pinMode(keyPadX3, OUTPUT);

  pinMode(remoteControl_DA, INPUT_PULLUP);
  pinMode(remoteControl_WS, INPUT_PULLUP);
  pinMode(nrf24L01_CE, OUTPUT);
  pinMode(gameduinoSDsel, OUTPUT);
  pinMode(gameduinoGPUsel, OUTPUT);
  pinMode(gameduinoInterrupt, INPUT_PULLUP);
  pinMode(endStop_X, INPUT_PULLUP); // 0 Active - X Axis Endstop Microswitch
  pinMode(endStop_Y, INPUT_PULLUP); // 0 Active - Y Axis Endstop Microswitch
  pinMode(endStop_Z, INPUT_PULLUP); // 0 Active - Z Axis Endstop Microswitch
  pinMode(inverter, OUTPUT); // 0 Active - Mains Inverter Relay
  pinMode(cabinetLED, OUTPUT); // 0 Active - Cabinet LED PSU Relay
  pinMode(cabinetFans, OUTPUT); // 0 Active - Rear Fan control via ULN2003A
  pinMode(safetyInterlock, INPUT_PULLUP); // 0 Active - Safety Interlock (reed switch)
  pinMode(startButton, INPUT_PULLUP); // 0 Active - Start Button (Green)
  pinMode(startLED, OUTPUT); // 0 Active - Start LED (Green)
  pinMode(pauseLED, OUTPUT); // 0 Active - Pause LED (Yellow)
  pinMode(pauseButton, INPUT_PULLUP); // 0 Active - Pause Button (Yellow)
  pinMode(stopLED, OUTPUT); // 0 Active - Stop LED (Red)
  pinMode(stopButton, INPUT_PULLUP); // 0 Active - Stop Button (Red)

  // Set initial states
  
  digitalWrite(inverter, HIGH);
  digitalWrite(cabinetLED, LOW);
  digitalWrite(cabinetFans, LOW);
  digitalWrite(gameduinoGPUsel, LOW); //~~~~~~~~~~~~~~~~~~~~~~Changed off-line~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  digitalWrite(gameduinoSDsel, LOW); //~~~~~~~~~~~~~~~~~~~~~~Changed off-line~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  digitalWrite(nrf24L01_CE, HIGH); //~~~~~~~~~~~~~~~~~~~~~~Changed off-line~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  ManageEEPROM();
}

void loop(){ //--------------------------------------------------------------------------------------------------------------void loop()---
  if(cncModeVal == 0){ // If machine in mode 0 (idle) rotate the adafruit display with dev255 graphic
    Dev255Rotate();
  }
  EncoderPosition(); // Gets ODrive encoder positions (X,Y,Z and Spindle speed) and updates Main Display at 30Hz Refresh
  LEDandStatusUpdate(); // Update the main display status of Teensy, ODrive, Dazzler and CNC Mill
  GCodeTextScroll(); // Scroll through the currently loaded GCode
  MenuSystem(); // Selects menus to display, scrolling menus and allows selection of each function
  TopButtons();
}

time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}

void TopButtons (void){
  int startPressTime = 0;
  startPress = digitalRead(startButton);
  pausePress = digitalRead(pauseButton);
  stopPress = digitalRead(stopButton);
  if (startPress == LOW){
    if(fileCopied == true){
      while(startPress == LOW){
        if (gCodeStartFromButton == true){
          startLEDMode = 1;
        }else{
          startLEDMode = 3;
        }
        EncoderPosition();
        startPress = digitalRead(startButton);
        delay(1);
        startPressTime++;
        if(startPressTime > 100){
          gCodeStartFromButton = true;
          startPressTime = 0;
          startLEDMode = 1;
        }
      }
    }
    if(gCodeStartFromButton == false){
      switch(spindleMode){
        case 0:
          SimultaneousCalibration();
        break;
        case 1:
          SpindleMode(3);
          stopLEDMode = 0;
          startLEDMode = 4;
        break;
        case 2:
          SpindleMode(3);
        break;
        case 3:
          spindleSetSpeed = speedStep;
          SpindleMode(4,spindleSetSpeed);
          digitalWrite(inverter, LOW);
          digitalWrite(cabinetFans, HIGH);
          stopLEDMode = 5;
          pauseLEDMode = 5;
          startLEDMode = 1;
        break;
        case 4:
          spindleSetSpeed = spindleSetSpeed + speedStep;
          Serial.print("spindleSetSpeed = ");
          Serial.println(spindleSetSpeed);
          if(spindleSetSpeed >= spindleRPM_Max){
            spindleSetSpeed = speedStep;
            SpindleRampDown(spindleSetSpeed);
          }else{
            SpindleRampUp(spindleSetSpeed);
          }
        break;
      }
      delay(20);
      while(startPress == LOW){
        EncoderPosition();
        startPress = digitalRead(startButton);
      }
    }
  }
  if (stopPress == LOW && axisSstate == 7){
    SpindleRampDown(0);
    SpindleStop();
    digitalWrite(inverter, HIGH);
    digitalWrite(cabinetFans, LOW);
    stopLEDMode = 1;
    pauseLEDMode = 5;
    startLEDMode = 5;
    delay(20);
    while(stopPress == LOW){
      stopLEDMode = 1;
      EncoderPosition();
      stopPress = digitalRead(stopButton);
    }
  }
  doorOpen = digitalRead(safetyInterlock);
  if (doorOpen == HIGH && axisSstate == 7){
    SpindleRampDown(0);
    SpindleStop();
    digitalWrite(inverter, HIGH);
    digitalWrite(cabinetFans, LOW);
    stopLEDMode = 5;
    pauseLEDMode = 3;
    startLEDMode = 5;
    delay(20);
    while(doorOpen == HIGH){
      EncoderPosition();
      doorOpen = digitalRead(safetyInterlock);
    }
    stopLEDMode = 5;
    pauseLEDMode = 2;
    startLEDMode = 5;
  }
}

void MenuSystem(void){
unsigned int loopTimeSpindle = 0U;
// Menu 0 - Option 5 - Run currently loaded GCode from keypad
  if (KeyPadRetrieve() == 5 && menu000 == true && fileCopied == true){
    KeyHeldWait();
    GCodeRun();
  }
// Menu 0 - Option 5 - Run currently loaded GCode from start button
  if (gCodeStartFromButton == true && menu000 == true && fileCopied == true){
    gCodeStartFromButton = false;
    GCodeRun();
  }
// Menu 1 - Option 1 - Run Jog and Zero mode 
  if (KeyPadRetrieve() == 1 && menu001 == true){
    cncModeVal = 4;
    menuSelect = 0;
    menu00Pos = 20;
    menu01Pos = 760;
    menu02Pos = 740;
    menu000 = true;
    menu001 = false;
    menu002 = false;
    KeyHeldWait();
    JogMode();
  }
// Menu 1 - Option 3 - Select manual circle mill
  if(KeyPadRetrieve() == 3 && menu001 == true){
    cncModeVal = 1;
    menuSelect = 0;
    menu00Pos = 20;
    menu01Pos = 760;
    menu02Pos = 740;
    menu000 = true;
    menu001 = false;
    menu002 = false;
    KeyHeldWait();
    CircleMill();
  }
// Menu 1 - Option 8 - Select GCode file to load from SD Card to Teensy 4.1 RAM
  if (KeyPadRetrieve() == 8 && menu001 == true){
    KeyHeldWait();
    GCodeLoadtoRAM();
    menuSelect = 0;
    MenuMove();
  }
// Menu 2 - Option 1 - Calibrate ODrive 0 Motor 1 (X-Axis)
  if (KeyPadRetrieve() == 1 && menu002 == true && cncModeVal == 0){
    FullCalibration(0,1);
    KeyHeldWait();
  }
// Menu 2 - Option 2 - Calibrate ODrive 0 Motor 0 (Y-Axis)
  if (KeyPadRetrieve() == 2 && menu002 == true && cncModeVal == 0){
    FullCalibration(0,0);
    KeyHeldWait();
  }
// Menu 2 - Option 3 - Calibrate ODrive 1 Motor 1 (Z-Axis)
  if (KeyPadRetrieve() == 3 && menu002 == true && cncModeVal == 0){
    FullCalibration(1,1);
    KeyHeldWait();
  }
// Menu 2 - Option 4 - Calibrate ODrive 1 Motor 0 (Spindle)
  if (KeyPadRetrieve() == 4 && menu002 == true && cncModeVal == 0){
    FullCalibration(1,0);
    KeyHeldWait();
  }
// Menu 2 - Option 5 - Stop All ODrive Motors
  if (KeyPadRetrieve() == 5 && menu002 == true && cncModeVal == 0){
    AllStop();
    KeyHeldWait();
  }
  // Menu 2 - Option 8 - Stop All ODrive Motors
  if (KeyPadRetrieve() == 8 && menu002 == true && cncModeVal == 0){
    menuSelect = 1;
    MenuMove();
    menuSelect = 0;
    MenuMove();
    digitalWrite(cabinetFans, HIGH);
    SpindleStart(spindleSetSpeed);
    spindleTime = millis();
    delay(10);
    while(spindleTime + 30000 > loopTimeSpindle){
      EncoderPosition();
      loopTimeSpindle = millis();
    }
    SpindleStop();
    digitalWrite(cabinetFans, LOW);
    KeyHeldWait();
  }
// Menu 2 - Option 9 - Adjust X and Y Backlash amounts
  if (KeyPadRetrieve() == 9 && menu002 == true && cncModeVal == 0){
    KeyHeldWait();
    backlashAdjust = true;
    ManageEEPROM();
  }

// Select the next menu if [#] is pressed on the keypad, menu 0 (Main Display) and menu 1 (Machine Menu) detirmine the next menu
  if (KeyPadRetrieve() == 11){
    if (menuSelect == 0){
      menuSelect = 1;
    }else{
      menuSelect = 2;
    }
    KeyHeldWait();
  }
// Select the previous menu if [*] is pressed on the keypad, menu 0 (Main Display) and menu 1 (Machine Menu) detirmine the next menu
  if (KeyPadRetrieve() == 10){
    if (menuSelect == 2){
      menuSelect = 1;
    }else{
      menuSelect = 0;
    }
    KeyHeldWait();
  }
  MenuMove();
}

void MenuMove(void){
// Move Menus off and on screen - Determines which menu to display on screen from the menu selected with [#] or [*] keys
// and the menu's position (menu's off screen are not drawn outside of a set value to save Display List Overflows)
  if (menuSelect == 1 || menu001 == true){
    if(menu01Pos > 20 && menu001 == false){
      for(int stopPos = 120; stopPos < menu01Pos; menu01Pos = menu01Pos-50, menu00Pos = menu00Pos-50){ // Right to Left Move both menu 0 off the screen and menu 1 on the screen at fast rate
        EncoderPosition(); // Gets ODrive encoder positions (X,Y,Z and Spindle speed) and updates display
      }
      for(int stopPos = 20; stopPos < menu01Pos; menu01Pos = menu01Pos-5, menu00Pos = menu00Pos-5){ // Right to Left Move both menu 0 off the screen and menu 1 on the screen at slow rate
        EncoderPosition(); // Gets ODrive encoder positions (X,Y,Z and Spindle speed) and updates display
      }
      menu001 = true;
      menu000 = false;
    }
    if(menu01Pos < 20 && menu001 == false){
      for(int stopPos = -50; stopPos > menu01Pos; menu01Pos = menu01Pos+50, menu02Pos = menu02Pos+50){ // Left to Right Move both menu 1 on the screen and menu 2 off the screen at fast rate
        EncoderPosition(); // Gets ODrive encoder positions (X,Y,Z and Spindle speed) and updates display
      }
      for(int stopPos = 20; stopPos > menu01Pos; menu01Pos = menu01Pos+5, menu02Pos = menu02Pos+5){ // Left to Right Move both menu 1 on the screen and menu 2 off the screen at slow rate
        EncoderPosition(); // Gets ODrive encoder positions (X,Y,Z and Spindle speed) and updates display
      }
      menu001 = true;
      menu002 = false;
    }
    if(menuSelect == 0){
      for(int stopPos = 700; stopPos > menu01Pos; menu01Pos = menu01Pos+50, menu00Pos = menu00Pos+50){ // Left to Right Move both menu 0 on the screen and menu 1 off the screen at fast rate
        EncoderPosition(); // Gets ODrive encoder positions (X,Y,Z and Spindle speed) and updates display
      }
      for(int stopPos = 760; stopPos > menu01Pos; menu01Pos = menu01Pos+5, menu00Pos = menu00Pos+5){ // Left to Right Move both menu 0 on the screen and menu 1 off the screen at slow rate
        EncoderPosition(); // Gets ODrive encoder positions (X,Y,Z and Spindle speed) and updates display
      }
      menu001 = false;
      menu000 = true;
    }
    if(menuSelect == 2){
      for(int stopPos = -620; stopPos < menu01Pos; menu01Pos = menu01Pos-50, menu02Pos = menu02Pos-50){ // Right to Left Move both menu 1 off the screen and menu 2 on the screen at fast rate
        EncoderPosition(); // Gets ODrive encoder positions (X,Y,Z and Spindle speed) and updates display
      }
      for(int stopPos = -700; stopPos < menu01Pos; menu01Pos = menu01Pos-5, menu02Pos = menu02Pos-5){ // Right to Left Move both menu 1 off the screen and menu 2 on the screen at slow rate
        EncoderPosition(); // Gets ODrive encoder positions (X,Y,Z and Spindle speed) and updates display
      }
      menu001 = false;
      menu002 = true;
    }
  }
}

void GCodeTextScroll(void){
// Pressing keypad 8 increases gcCurrLineNo by 1, forcing the GCode line to step forwards and read from the next return value (ASCII 13)
// Holding keypad no 8 for 0.5 second steps gcCurrLineNo once per screen refresh (fast scrolling) until there are no more characters to read
  if(KeyPadRetrieve() == 8 && cncModeVal == 0 && menu000 == true){
    loopTimeFunction = millis();
    gcASCIIval = memoryFile.charAt(gcLiveCharPos+1);
    if(gcASCIIval < 10 || gcASCIIval > 126){
      // Do not advance
    }else{
      gcCurrLineNo++;
    }
    while(KeyPadRetrieve() < 12){
      EncoderPosition(); // Gets ODrive encoder positions (X,Y,Z and Spindle speed) and updates display      if(millis() > loopTimeFunction + 1000){
      if(millis() > loopTimeFunction + 500){
        while(KeyPadRetrieve() == 8){
          if(gcCurrLSP < gcCurrLineNo || gcASCIIval < 10 || gcASCIIval > 126){
            EncoderPosition(); // Gets ODrive encoder positions (X,Y,Z and Spindle speed) and updates display
          }else{
            gcCurrLineNo++; // Get it to look backwards to count back (gCodePosVal-2) then read and -2 until it gets to a 13 value, then display that line, all to save the full count from the beginning, then work on press and hold to fast forward
            EncoderPosition(); // Gets ODrive encoder positions (X,Y,Z and Spindle speed) and updates display
          }
        }
      }
    }
  }
// Pressing keypad 2 decreases gcCurrLineNo by 1, forcing the GCode line to step backwards and read from the previous return value (ASCII 13)
// Holding keypad no 2 for 0.5 second steps gcCurrLineNo once per screen refresh (fast scrolling) until line 0 is reached
  if  (KeyPadRetrieve() == 2 && cncModeVal == 0 && menu000 == true){
    if(gcLiveCharPos > 0){
      loopTimeFunction = millis();
      gcCurrLineNo--; // Get it to look backwards to count back (gCodePosVal-2) then read and -2 until it gets to a 13 value, then display that line, all to save the full count from the beginning, then work on press and hold to fast forward
      while(KeyPadRetrieve() < 12){
        EncoderPosition(); // Gets ODrive encoder positions (X,Y,Z and Spindle speed) and updates display
        if(millis() > loopTimeFunction + 500){
          while(KeyPadRetrieve() == 2){
            if(gcCurrLSP > gcCurrLineNo){
              EncoderPosition(); // Gets ODrive encoder positions (X,Y,Z and Spindle speed) and updates display
            }else{
              if(gcLiveCharPos > 0){
                gcCurrLineNo--; // Get it to look backwards to count back (gCodePosVal-2) then read and -2 until it gets to a 13 value, then display that line, all to save the full count from the beginning, then work on press and hold to fast forward
              }
              EncoderPosition(); // Gets ODrive encoder positions (X,Y,Z and Spindle speed) and updates display
            }
          }
        }
      }
    }
  }
}

// Status LED and descriptions updated here
void LEDandStatusUpdate(void){
  if(oDrive0m0CLC == true && oDrive0m1CLC == true){
    cncStatVal = 2;
    axisXstate = 7;
    axisYstate = 7;
    axisZstate = 7;
  }else{
    if(oDrive0m0CLC == false && oDrive0m1CLC == true){
      cncStatVal = 0;
      oDvStatVal = 0;
      axisXstate = 7;
      axisYstate = 0;
      axisZstate = 7;    
    }else{
      if(oDrive0m0CLC == true && oDrive0m1CLC == false){
        cncStatVal = 0;
        oDvStatVal = 0;
        
        axisXstate = 0;
        axisYstate = 7;
        axisZstate = 7;
      }else{
        cncStatVal = 0;
        oDvStatVal = 0;
        axisXstate = 0;
        axisYstate = 0;
        axisZstate = 7;
      }
    }
  }
  if(oDrive1m0CLC == true){
    axisSstate = 7;
  }else{
    axisSstate = 0;
  }
}

void DrawMainScreen(void){ //--------------------------------------------------------------------------------------void DrawMainScreen()---
  GD.ClearColorRGB(0x000020); // Dark blue background clear colour
  GD.Clear();
  GD.VertexFormat(0); // points are scaled to screen pixels
  AxesPanelMove(); // float value to move the Axes Display in Horizontal and Vertical plane, 0 = No Move.
  AxesPanel(axDispHor,axDispVer); // horizontal min = 10, max = 320, vertical min = 10, max = 100 ---- set axDispHor 154, axDispVer 70 for center
  DevRotate(665,10); // Corner dev255 graphic rotation
  ClockDisplayMove (0.0166); // float value to move the Clock Display in Horizontal and Vertical plane, 0 = No Move.
  TimeWindow(clDispHor,clDispVer); // horizontal min = 13, max = 580, vertical min = 1150, max = 1230 ---- set axDispHor 576, axDispVer 1226 for center
  if(menuSelect == 0 || menu00Pos >-700){
    Menu_MainDisplay(menu00Pos,400);
    if (menuSelect == 0 && menu00Pos == 20 && cncModeVal == 0){
      GCodeToDisplay(menu00Pos+132,622);
    }
    if (menuSelect == 0 && menu00Pos == 20 && cncModeVal == 1){
      CircleToDisplay(menu00Pos+132,622);
    }
  }
  if((menuSelect == 1) || (menu01Pos <720 && menu01Pos >-700)){
    Menu_Work(menu01Pos,400);
  }
  if((menuSelect == 2) || (menu02Pos <740 && menu02Pos >-700)){
    Menu_Machine(menu02Pos,400);
  }
  GD.swap();
}

void Menu_MainDisplay(int hor, int ver){ //----------------------------------------------------------------------void Menu_MainDisplay()---
  GD.ColorRGB(0x000000);
  GD.Begin(RECTS);
  GD.LineWidth(15*16);
  GD.Vertex2f(hor,ver);
  GD.Vertex2f(hor+680,ver+450);
  GD.ColorRGB(0xe0e0e0);
  GD.Begin(BITMAPS);
  GD.cmd_loadidentity();
  GD.cmd_scale(F16(1.5),F16(1.5));
  GD.cmd_setmatrix();
  GD.BitmapHandle(4);
  GD.BitmapSize(NEAREST, BORDER, BORDER, 750, 525);
  GD.LineWidth(16 * 20);
  GD.ColorMask(0,0,0,1);
  GD.BlendFunc(ONE, ONE);
  GD.ColorMask(1,1,1,0);
  GD.BlendFunc(DST_ALPHA, ONE_MINUS_DST_ALPHA);
  GD.Vertex2f(hor-15,ver-15);
  GD.ColorMask(1,1,1,1);
  GD.BlendFunc(SRC_ALPHA, ONE_MINUS_SRC_ALPHA);
  GD.cmd_loadidentity();
  GD.cmd_scale(F16(1),F16(1));
  GD.cmd_setmatrix();
  GD.Begin(BITMAPS);
  GD.BitmapHandle(5);
  GD.BitmapSize(NEAREST, BORDER, BORDER, 12, 12);
  GD.Vertex2f(hor+310, ver+424);
  GD.Vertex2f(hor+310, ver+444);
  GD.Vertex2f(hor+660, ver+424);
  GD.Vertex2f(hor+660, ver+444);
  GD.Begin(RECTS);
  GD.LineWidth(1*16);
  GD.ColorRGB(0x101050);
  GD.Vertex2f(hor-2,ver+138);
  GD.Vertex2f(hor+300,ver+138);
  GD.Vertex2f(hor+421,ver+138);
  GD.Vertex2f(hor+680,ver+138);
  GD.LineWidth(5*16);
  CreateBevel(hor+128,ver+218,hor+650,ver+413,0xf0f0f0,0x102010); // GCode Window
  CreateBevel(hor+55,ver+220,hor+115,ver+228,0xf0f0f0,0x102010); // GCode Line No Window
  CreateBevel(hor+128,ver+150,hor+250,ver+165,0xffffff,0x101040); // Current Mode Window
  CreateBevel(hor+128,ver+200,hor+650,ver+208,0xf0f0f0,0x102010); // Active GCode Window
  CreateBevel(hor+128,ver+182,hor+350,ver+190,0xf0f0f0,0x102010); // Current Feature Window
  CreateBevel(hor+478,ver+182,hor+650,ver+190,0xf0f0f0,0x102010); // Current Tool Window
  CreateBevel(hor+128,ver+423,hor+300,ver+433,0xffffff,0x101040); // CNC Status: Background
  CreateBevel(hor+128,ver+443,hor+300,ver+453,0xffffff,0x101040); // Dazzler Status: Background
  CreateBevel(hor+478,ver+423,hor+650,ver+433,0xffffff,0x101040); // Teensy Status: Background
  CreateBevel(hor+478,ver+443,hor+650,ver+453,0xffffff,0x101040); // ODrive Status: Background
  CreateBevel(hor+455,ver+31,hor+580,ver+55,0xf0f0f0,0x301500); // Spindle Speed Background
  SpindleRotate(hor+610,ver+10,spinMotBmpRot);
  GD.BitmapHandle(11);
  GD.ColorRGB(0xff8000);
  GD.cmd_number(hor+540,ver+26, 30, OPT_RIGHTX | OPT_SIGNED, int(spindleRPM));
  GD.cmd_text(hor+575,ver+48, 16, OPT_RIGHTX, "RPM");
  LEDflash(hor,ver);
  GD.ColorA(160);
  if (cncStatVal == 0 || cncStatVal == 1 || cncStatVal == 4 || cncStatVal == 5 || cncStatVal == 6){
    if(ledOn == false){
      cncLEDSize = 0;
    }else{
      cncLEDSize = 75;
    }
  }
  if (cncStatVal == 2 || cncStatVal == 3){
    cncLEDSize = 75;
  }
  GD.PointSize(cncLEDSize);
  GD.ColorRGB(cncStatLED[cncStatVal]);
  GD.Vertex2f(hor+315,ver+429);
  if (dazStatVal == 1){
    if(ledOn == false){
      dazLEDSize = 0;
    }else{
      dazLEDSize = 75;
    }
  }
  if (dazStatVal == 0){
    dazLEDSize = 75;
  }
  GD.PointSize(dazLEDSize);
  GD.ColorRGB(dazStatLED[dazStatVal]);
  GD.Vertex2f(hor+315,ver+449);
  if (tnyStatVal == 1 || tnyStatVal == 2){
    if(ledOn == false){
      tnyLEDSize = 0;
    }else{
      tnyLEDSize = 75;
    }
  }
  if (tnyStatVal == 0){
    tnyLEDSize = 75;
  }
  GD.PointSize(tnyLEDSize);
  GD.ColorRGB(tnyStatLED[tnyStatVal]);
  GD.Vertex2f(hor+665,ver+429);
  if (oDvStatVal == 0 || oDvStatVal == 1 || oDvStatVal == 2 || oDvStatVal == 3 || oDvStatVal == 4 || oDvStatVal == 5 || oDvStatVal == 6 || oDvStatVal == 8 || oDvStatVal == 9){
    if(ledOn == false){
      oDvLEDSize = 0;
    }else{
      oDvLEDSize = 75;
    }
  }
  if (oDvStatVal == 7){
    oDvLEDSize = 75;
  }
  GD.PointSize(oDvLEDSize);
  GD.ColorRGB(oDvStatLED[oDvStatVal]);
  GD.Vertex2f(hor+665,ver+449);
  GD.ColorA(255);
  GD.ColorRGB(0x101050);
  GD.cmd_text(hor+120,ver+425, 16, OPT_RIGHTX, "CNC STATUS:");
  GD.cmd_text(hor+120,ver+445, 16, OPT_RIGHTX, "DAZZLER STATUS:");
  GD.cmd_text(hor+470,ver+425, 16, OPT_RIGHTX, "TEENSY STATUS:");
  GD.cmd_text(hor+470,ver+445, 16, OPT_RIGHTX, "ODRIVE STATUS:");
  GD.cmd_text(hor+361,ver+138, 16, OPT_CENTER, "PROGRAM STATUS");
  GD.cmd_text(hor+518,ver+17, 16, OPT_CENTER, "SPINDLE SPEED");
  GD.cmd_text(hor+47,ver+220, 16, OPT_RIGHTX, "LINE:");
  GD.cmd_text(hor+120,ver+183, 16, OPT_RIGHTX, "CURRENT FEATURE:");
  GD.cmd_text(hor+470,ver+183, 16, OPT_RIGHTX, "CURRENT TOOL:");
  GD.cmd_text(hor+120,ver+201, 16, OPT_RIGHTX, "ACTIVE G-CODE:");
  GD.cmd_text(hor+120,ver+150, 18, OPT_RIGHTX, "CURRENT MODE:");
  GD.ColorRGB(0xc0c0f0);
  GD.cmd_text(hor+130,ver+159, 18, OPT_CENTERY, cncModeStrings[cncModeVal]);
  GD.cmd_text(hor+480,ver+187, 16, OPT_CENTERY, nextToolStrings[nextToolval]);
  GD.cmd_text(hor+135,ver+429, 16, OPT_CENTERY, cncStatStrings[cncStatVal]);
  GD.cmd_text(hor+135,ver+449, 16, OPT_CENTERY, dazStatStrings[dazStatVal]);
  GD.cmd_text(hor+485,ver+429, 16, OPT_CENTERY, tnyStatStrings[tnyStatVal]);
  GD.cmd_text(hor+485,ver+449, 16, OPT_CENTERY, oDvStatStrings[oDvStatVal]);
  if(cncModeVal == 0){
    GD.cmd_number(hor+112,ver+221, 16, OPT_RIGHTX, gcCurrLSP);
  }
  if(tnyStatVal == 0){
    GD.cmd_number(hor+610,ver+430, 16, OPT_CENTERY, loopTimeDisplay);
  }
}

void Menu_Machine(int hor, int ver){ //------------------------------------------------------------------------------void Menu_Machine()---
  GD.ColorRGB(0x000000);
  GD.Begin(RECTS);
  GD.LineWidth(15*16);
  GD.Vertex2f(hor,ver);
  GD.Vertex2f(hor+680,ver+450);
  GD.ColorRGB(0x505050);
  GD.Begin(BITMAPS);
  GD.cmd_loadidentity();
  GD.cmd_scale(F16(1.5),F16(1.5));
  GD.cmd_setmatrix();
  GD.BitmapHandle(4);
  GD.BitmapSize(NEAREST, BORDER, BORDER, 750, 525);
  GD.LineWidth(16 * 20);
  GD.ColorMask(0,0,0,1);
  GD.BlendFunc(ONE, ONE);
  GD.ColorMask(1,1,1,0);
  GD.BlendFunc(DST_ALPHA, ONE_MINUS_DST_ALPHA);
  GD.Vertex2f(hor-15,ver-15);
  GD.ColorMask(1,1,1,1);
  GD.BlendFunc(SRC_ALPHA, ONE_MINUS_SRC_ALPHA);
  GD.cmd_loadidentity();
  GD.cmd_scale(F16(1),F16(1));
  GD.cmd_setmatrix();
  LEDflash(hor,ver);
  GD.ColorRGB(0xcccccc);
  GD.cmd_text(hor+350,ver+20,28,OPT_CENTER,"MACHINE MENU");
  GD.ColorRGB(0x808080);
  GD.cmd_text(hor+349,ver+21,28,OPT_CENTER,"MACHINE MENU");
  GD.ColorRGB(0xcccccc);
  GD.cmd_text(hor+5,ver+450,18,OPT_CENTERY, cncArduinoSketchVer[0]);
  if (backlashAdjust == true){
    GD.ColorRGB(0xffffff);
    GD.cmd_text(hor+5,ver+230,16,OPT_CENTERY,"X & Y Backlash set to Zero");
    GD.cmd_text(hor+5,ver+245,16,OPT_CENTERY,"1. Move dial indicator to touch an edge & zero indicator");
    GD.cmd_text(hor+5,ver+260,16,OPT_CENTERY,"2. Move Axis in opposite direction until indicator just moves");
    GD.cmd_text(hor+5,ver+275,16,OPT_CENTERY,"3. Jog Axis display back the indicated amount (indicator & table should not move)");
    GD.cmd_text(hor+5,ver+290,16,OPT_CENTERY,"4. Press [3] to set X Axis or [9] to set Y Axis");
    GD.cmd_text(hor+5,ver+320,16,OPT_CENTERY,"Note: Dial LED will turn Green once backlash set");
    GD.cmd_text(hor+5,ver+335,16,OPT_CENTERY,"Press [#] to exit");
    GD.cmd_text(hor+5,ver+365,16,OPT_CENTERY,"Current X =");
    GD.cmd_text(hor+5,ver+380,16,OPT_CENTERY,"Current Y =");
    GD.cmd_text(hor+170,ver+365,16,OPT_CENTERY,"New =");
    GD.cmd_text(hor+170,ver+380,16,OPT_CENTERY,"New =");
    GD.cmd_text(hor+125,ver+365,16,OPT_CENTERY,"um");
    GD.cmd_number(hor+120,ver+361,16,OPT_RIGHTX, int(xBacklash_mm));
    GD.cmd_text(hor+125,ver+380,16,OPT_CENTERY,"um");
    GD.cmd_number(hor+120,ver+376,16,OPT_RIGHTX, int(yBacklash_mm));
    xBacklash_New = (xBacklash * 1000) / 16384;
    yBacklash_New = (yBacklash * 1000) / 16384;
    GD.cmd_number(hor+240,ver+360,16,OPT_RIGHTX, int(xBacklash_New));
    GD.cmd_text(hor+245,ver+365,16,OPT_CENTERY,"um");
    GD.cmd_number(hor+240,ver+376,16,OPT_RIGHTX, int(yBacklash_New));
    GD.cmd_text(hor+245,ver+380,16,OPT_CENTERY,"um");
  }else{
    GD.ColorRGB(0x8080dd);
    GD.cmd_text(hor+70,ver+70,28,OPT_CENTERY,"1. Calibrate ODrive X-Axis");
    GD.cmd_text(hor+70,ver+100,28,OPT_CENTERY,"2. Calibrate ODrive Y-Axis");
    GD.cmd_text(hor+70,ver+130,28,OPT_CENTERY,"3. Calibrate ODrive Z-Axis");
    GD.cmd_text(hor+70,ver+160,28,OPT_CENTERY,"4. Calibrate ODrive Spindle");
    GD.cmd_text(hor+70,ver+190,28,OPT_CENTERY,"5. Stop all Motors");
    GD.cmd_text(hor+390,ver+70,28,OPT_CENTERY,"6. Tool Change");
    GD.cmd_text(hor+390,ver+100,28,OPT_CENTERY,"7. Tool Height Measure");
    GD.cmd_text(hor+390,ver+130,28,OPT_CENTERY,"8. Set Max Spidle Speed");
    GD.cmd_text(hor+390,ver+160,28,OPT_CENTERY,"9. Adjust X, Y backlash");
    GD.cmd_text(hor+390,ver+190,28,OPT_CENTERY,"10. Setup Soft End Stops");
  }
}

void Menu_Work(int hor, int ver){ //------------------------------------------------------------------------------------void Menu_Work()---
  GD.ColorRGB(0x000000);
  GD.Begin(RECTS);
  GD.LineWidth(15*16);
  GD.Vertex2f(hor,ver);
  GD.Vertex2f(hor+680,ver+450);
  GD.ColorRGB(0x704070);
  GD.Begin(BITMAPS);
  GD.cmd_loadidentity();
  GD.cmd_scale(F16(1.5),F16(1.5));
  GD.cmd_setmatrix();
  GD.BitmapHandle(4);
  GD.BitmapSize(NEAREST, BORDER, BORDER, 750, 525);
  GD.LineWidth(16 * 20);
  GD.ColorMask(0,0,0,1);
  GD.BlendFunc(ONE, ONE);
  GD.ColorMask(1,1,1,0);
  GD.BlendFunc(DST_ALPHA, ONE_MINUS_DST_ALPHA);
  GD.Vertex2f(hor-15,ver-15);
  GD.ColorMask(1,1,1,1);
  GD.BlendFunc(SRC_ALPHA, ONE_MINUS_SRC_ALPHA);
  GD.cmd_loadidentity();
  GD.cmd_scale(F16(1),F16(1));
  GD.cmd_setmatrix();
  LEDflash(hor,ver);
  GD.ColorRGB(0xcccccc);
  GD.cmd_text(hor+350,ver+20,28,OPT_CENTER,"WORK MENU");
  GD.ColorRGB(0x808080);
  GD.cmd_text(hor+349,ver+21,28,OPT_CENTER,"WORK MENU");
  GD.ColorRGB(0x8080dd);
  GD.cmd_text(hor+70,ver+70,28,OPT_CENTERY,"1. Jog + Zero Axes");
  GD.cmd_text(hor+70,ver+100,28,OPT_CENTERY,"2. No Z - Dry Run");
  GD.cmd_text(hor+70,ver+130,28,OPT_CENTERY,"3. Cut Basic Circle");
  GD.cmd_text(hor+70,ver+160,28,OPT_CENTERY,"4. Cut Basic Rectangle");
  GD.cmd_text(hor+70,ver+190,28,OPT_CENTERY,"5. Cut Basic Line Shape");
  GD.cmd_text(hor+390,ver+70,28,OPT_CENTERY,"6. Hole Drilling");
  GD.cmd_text(hor+390,ver+100,28,OPT_CENTERY,"7. Hole Tapping");
  GD.cmd_text(hor+390,ver+130,28,OPT_CENTERY,"8. Choose GCode File");
  GD.cmd_text(hor+390,ver+160,28,OPT_CENTERY,"9. Option 9");
  GD.cmd_text(hor+390,ver+190,28,OPT_CENTERY,"10. Option 10");
}

//WRITE THE CIRCLE PROGRAM ON TO PROGRAM STATUS DISPLAY
void CircleToDisplay(int hor, int ver){ //------------------------------------------------------------------------void CircleToDisplay()---
  GD.ColorRGB(0xffffff);
  GD.cmd_text(hor,ver+20, 16, OPT_CENTERY, "Circle Setup");
  GD.cmd_text(hor+20,ver+40, 16, OPT_CENTERY, "1 Diameter mm = ");
  GD.cmd_number(hor+240,ver+40, 16, OPT_CENTERY, circleDiameter);  
  GD.cmd_text(hor+20,ver+50, 16, OPT_CENTERY, "2 Int/Ext = ");
  if(internalCircle == true){
    GD.cmd_text(hor+240,ver+50, 16, OPT_CENTERY, "Internal");
  }else{
    GD.cmd_text(hor+240,ver+50, 16, OPT_CENTERY, "External");
  }
  GD.cmd_text(hor+20,ver+60, 16, OPT_CENTERY, "3 Depth mm = ");
  GD.cmd_number(hor+240,ver+60, 16, OPT_CENTERY, circleDepth);
  GD.cmd_text(hor+20,ver+70, 16, OPT_CENTERY, "4 Center X mm = ");
  GD.cmd_number(hor+240,ver+70, 16, OPT_CENTERY, circleXcoord);
  GD.cmd_text(hor+20,ver+80, 16, OPT_CENTERY, "5 Center Y mm = ");
  GD.cmd_number(hor+240,ver+80, 16, OPT_CENTERY, circleYcoord);
  GD.cmd_text(hor+20,ver+90, 16, OPT_CENTERY, "6 Direction = ");
  if(climbMill == true){
    GD.cmd_text(hor+240,ver+90, 16, OPT_CENTERY, "Climb");
  }else{
    GD.cmd_text(hor+240,ver+90, 16, OPT_CENTERY, "Conventional");
  }
  GD.cmd_text(hor+20,ver+100, 16, OPT_CENTERY, "7 Tool Diameter mm = ");
  GD.cmd_number(hor+240,ver+100, 16, OPT_CENTERY, toolDiameter);
  GD.cmd_text(hor+20,ver+110, 16, OPT_CENTERY, "8 Feed mm/min = ");
  GD.cmd_number(hor+240,ver+110, 16, OPT_CENTERY, feedSpeed);
  GD.cmd_text(hor+20,ver+120, 16, OPT_CENTERY, "9 Use Above Values");
  if(keypadValRequired == true){
    EnteredValueToDisplay(hor,ver+150);
  }
  GD.ColorRGB(0xffff00);
  if(circleDiaSet == false){
    GD.cmd_text(hor,ver+40, 16, OPT_CENTERY, "!");
  }
  if(intSet == false){
    GD.cmd_text(hor,ver+50, 16, OPT_CENTERY, "!");
  }
  if(depSet == false){
    GD.cmd_text(hor,ver+60, 16, OPT_CENTERY, "!");
  }
  if(cirXset == false){
    GD.cmd_text(hor,ver+70, 16, OPT_CENTERY, "!");
  }
  if(cirYset == false){
    GD.cmd_text(hor,ver+80, 16, OPT_CENTERY, "!");
  }
  if(climbSet == false){
    GD.cmd_text(hor,ver+90, 16, OPT_CENTERY, "!");
  }
  if(toolDiaSet == false){
    GD.cmd_text(hor,ver+100, 16, OPT_CENTERY, "!");
  }
  if(speedSet == false){
    GD.cmd_text(hor,ver+110, 16, OPT_CENTERY, "!");
  }
  if(circleDiaSet == false || intSet == false || depSet == false || cirXset == false || cirYset == false || climbSet == false || toolDiaSet == false || speedSet == false){
    circleReady = false;
    displayEnter = false;
    GD.cmd_text(hor,ver+130, 16, OPT_CENTERY, "! PREVIOUS VALUES - [9] TO ACCEPT");    
  }else{
    displayEnter = true;
    circleReady = true;
  }
  if(displayEnter == true){
    GD.ColorRGB(0x00ff00);
    GD.cmd_text(hor+190,ver+160, 16, OPT_CENTERY, "PRESS [#] TO START");
  }
}

void GCodeToDisplay(int hor, int ver){ //--------------------------------------------------------------------------void GCodeToDisplay()---
  unsigned int featureLength = 0;
  int cfASCIIval = 0;
  unsigned long gcStartCharPos = 0;
       
  if(displayZAxisSet == true){
    GD.ColorRGB(0x00ffff);
    GD.cmd_text(hor+150,ver+170,16, OPT_CENTERY, "SET Z AXIS TO: ");
    GD.cmd_number(hor+275,ver+166,16, OPT_RIGHTX | OPT_SIGNED, int(z));
    GD.cmd_text(hor+277,ver+170,16,OPT_CENTERY,".");
    GD.cmd_number(hor+287,ver+166,16,4,int(10000 * abs(z)));
    GD.cmd_text(hor+325,ver+170, 16, OPT_CENTERY, "THEN PRESS [#] TO RESUME");
    GD.ColorRGB(0xc0c0f0);
  }
  if(cncStatVal == 3){
    //gcRunningString.toCharArray(gcRunningCharArray,50);
    GD.ColorRGB(0x00ffff);
    //GD.cmd_text(hor,ver-21, 16, OPT_CENTERY, gcRunningCharArray);
    GD.cmd_text(hor+50,ver+185, 16, OPT_CENTERY, "AMBER/RED = PAUSE");
    GD.ColorRGB(0xc0c0f0);
  }
  if(cncStatVal == 4){
    GD.ColorRGB(0x00ffff);
    GD.cmd_text(hor+50,ver+185, 16, OPT_CENTERY, "MACHINE PAUSED, PRESS GREEN TO RESUME, OR RED TO QUIT");
    GD.ColorRGB(0xc0c0f0);
  }
  if(cncStatVal == 5){
    GD.ColorRGB(0x00ffff);
    GD.cmd_text(hor+50,ver+185, 16, OPT_CENTERY, "CHANGE TOOL MANUALLY");
    GD.ColorRGB(0xc0c0f0);
  }
  gcTextHor = hor;
  gcTextVer = ver;
  gcCharCount = 0;
  gcTextLine = 0;
  GD.BitmapSource(0x2000e8);// Memory location 0x2000e8 aligns to the ASCII Table in memory for use in TXT8X8, only when buttons are drawn on the screen though
  GD.BitmapSize(NEAREST, BORDER, BORDER, 8, 8);
  GD.BitmapLayout(TEXT8X8,1,1);
  GD.Begin(BITMAPS);
  
  gcASCIIval = memoryFile.charAt(gcLiveCharPos);
  if(gcCurrLineNo != gcCurrLSP){ // Move displayed GCode by one line if gcCurrentLineNo has been changed by user or GCode Run
    if(gcCurrLineNo < gcCurrLSP){ // Move displayed GCode back by one line
      gcLiveCharPos = gcLiveCharPos - 2; // Go back 2 characters past the return (ASCII 13)
      do{
        gcASCIIval = memoryFile.charAt(gcLiveCharPos);
        gcLiveCharPos --;
      }while(gcASCIIval != 13 && gcLiveCharPos > 0);
      if(gcLiveCharPos > 0){
        gcLiveCharPos = gcLiveCharPos + 2; // Go to first character after (ASCII 13 and 10)
      }else{
        gcLiveCharPos = 0; // If at the beginning of the file
      }
      gcCurrLSP = gcCurrLineNo;
    }else{ // Move displayed GCode forward by one line
      do{
        gcASCIIval = memoryFile.charAt(gcLiveCharPos);
        gcLiveCharPos ++;
      }while(gcASCIIval != 13);
      gcCurrLSP = gcCurrLineNo;
    }
  }
  gcStartCharPos = gcLiveCharPos;
  while(gcCharCount < gcCharsToDisplay && gcTextLine < gcLinesToDisplay){ // Maximum 150 amount of characters to prevent "display list error" and maximum lines, whichever comes first
    gcASCIIval = memoryFile.charAt(gcLiveCharPos);
    gcLiveCharPos ++;
    if (gcASCIIval == 10){
    }else{
      if (gcASCIIval == 13){
        gcTextVer = gcTextVer + 10;
        gcTextHor = hor;
        gcTextLine++;
      }else{
        if (gcASCIIval == 32){
          gcTextHor = gcTextHor + 8;
        }else{
          GD.Cell(gcASCIIval);
          GD.Vertex2f(gcTextHor,gcTextVer);
          gcTextHor = gcTextHor + 10;
          gcCharCount++;
        }
      }
    }
  }
  gcLiveCharPos = gcStartCharPos;
  featureLength = currentFeature.length();
  for(unsigned int currentChar = 0; featureLength > currentChar; currentChar++){
    cfASCIIval = byte(currentFeature.charAt(currentChar));
    GD.Cell(cfASCIIval);
    GD.Vertex2f(hor,ver-39);
    hor = hor + 10;
  }
}

void AxesPanel(int hor, int ver){ //------------------------------------------------------------------------------------void AxesPanel()---
  GD.ColorRGB(0x8080ff);
  GD.Begin(BITMAPS);
  GD.BitmapHandle(7);
  GD.ColorRGB(0xbbbbbb);
  GD.Vertex2f(20,175);
  GD.BitmapHandle(8);
  GD.Vertex2f(360,18);
  GD.BitmapHandle(6);
  GD.ColorRGB(0x6060dd);
  //GD.ColorRGB(0xffffff);
  GD.Vertex2f(7,7);
  GD.ColorRGB(0x4080ff);
  GD.Begin(RECTS);
  GD.LineWidth(15*16);
  GD.Vertex2f(hor+2,ver+2);
  GD.Vertex2f(hor+385,ver+193);
  GD.Begin(BITMAPS);
  GD.BitmapHandle(4);
  GD.LineWidth(16 * 20);
  GD.ColorMask(0,0,0,1);
  GD.BlendFunc(ONE, ONE);
  GD.ColorMask(1,1,1,0);
  GD.BlendFunc(DST_ALPHA, ONE_MINUS_DST_ALPHA);
  GD.BitmapSize(NEAREST, BORDER, BORDER, 412, 220);
  GD.Vertex2f(hor-12,ver-12);
  GD.ColorMask(1,1,1,1);
  GD.BlendFunc(SRC_ALPHA, ONE_MINUS_SRC_ALPHA);
  GD.Begin(RECTS);
  GD.LineWidth(5*16);
  CreateBevel(hor+40,ver+5,hor+310,ver+37,0xf0f0f0,0x103010); // X Axis LCD Background
  CreateBevel(hor+40,ver+75,hor+310,ver+107,0xf0f0f0,0x103010); // Y Axis LCD Background
  CreateBevel(hor+40,ver+145,hor+310,ver+177,0xf0f0f0,0x103010); // Z Axis LCD Background
  CreateBevel(hor+200,ver+45,hor+300,ver+55,0xffffff,0x101040); // Machine X Axis LCD Background
  CreateBevel(hor+200,ver+115,hor+300,ver+125,0xffffff,0x101040); // Machine Y Axis LCD Background
  CreateBevel(hor+200,ver+185,hor+300,ver+195,0xffffff,0x101040); // Machine Z Axis LCD Background
  switch(xyzSelect){
    case 0:
      GD.cmd_fgcolor(0x00a0a0);
      GD.ColorRGB(xyzAxesCharLED[xStatVal]);
      GD.cmd_button(hor,ver+5,30,30,28,0,"X");
      GD.ColorRGB(xyzAxesCharLED[yStatVal]);
      GD.cmd_button(hor,ver+75,30,30,28,0,"Y");
      GD.ColorRGB(xyzAxesCharLED[zStatVal]);
      GD.cmd_button(hor,ver+145,30,30,28,0,"Z");
      break;
    case 1:
      GD.cmd_fgcolor(0x00a0a0);
      GD.ColorRGB(xyzAxesCharLED[xStatVal]);
      GD.cmd_button(hor,ver+5,30,30,28,0,"X");
      GD.cmd_fgcolor(0x004080);
      GD.ColorRGB(xyzAxesCharLED[yStatVal]);
      GD.cmd_button(hor,ver+75,30,30,28,0,"Y");
      GD.ColorRGB(xyzAxesCharLED[zStatVal]);
      GD.cmd_button(hor,ver+145,30,30,28,0,"Z");
      break;
    case 2:
      GD.cmd_fgcolor(0x00a0a0);
      GD.ColorRGB(xyzAxesCharLED[yStatVal]);
      GD.cmd_button(hor,ver+75,30,30,28,0,"Y");
      GD.cmd_fgcolor(0x004080);
      GD.ColorRGB(xyzAxesCharLED[xStatVal]);
      GD.cmd_button(hor,ver+5,30,30,28,0,"X");
      GD.ColorRGB(xyzAxesCharLED[zStatVal]);
      GD.cmd_button(hor,ver+145,30,30,28,0,"Z");
      break;
    case 3:
      GD.cmd_fgcolor(0x00a0a0);
      GD.ColorRGB(xyzAxesCharLED[zStatVal]);
      GD.cmd_button(hor,ver+145,30,30,28,0,"Z");
      GD.cmd_fgcolor(0x004080);
      GD.ColorRGB(xyzAxesCharLED[xStatVal]);
      GD.cmd_button(hor,ver+5,30,30,28,0,"X");
      GD.ColorRGB(xyzAxesCharLED[yStatVal]);
      GD.cmd_button(hor,ver+75,30,30,28,0,"Y");
      break;
    case 4:
      GD.cmd_fgcolor(0x004080);
      GD.ColorRGB(xyzAxesCharLED[xStatVal]);
      GD.cmd_button(hor,ver+5,30,30,28,0,"X");
      GD.ColorRGB(xyzAxesCharLED[yStatVal]);
      GD.cmd_button(hor,ver+75,30,30,28,0,"Y");
      GD.ColorRGB(xyzAxesCharLED[zStatVal]);
      GD.cmd_button(hor,ver+145,30,30,28,0,"Z");
      break;
  }
  AxesDisplay (hor+160,ver-3,31,x_mmDisplay);
  AxesDisplay (hor+160,ver+67,31,y_mmDisplay);
  AxesDisplay (hor+160,ver+137,31,z_mmDisplay);
  
  if(cncModeVal == 4){
    if(mmJogIncr < 10){
      if(mmJogIncr < 1){
        if(mmJogIncr < 0.1){
          if(mmJogIncr < 0.01){
            if(mmJogIncr < 0.001){
              highlightHor = 245;
            }else{
              highlightHor = 221;
            }
          }else{
            highlightHor = 197;
          }
        }else{
          highlightHor = 173;
        }
      }else{
        highlightHor = 140;
      }
    }else{
      highlightHor = 116;
    }
    GD.Begin(RECTS);
    GD.LineWidth(50);
    GD.ColorRGB(0x00ffff);
    GD.ColorA(100);
    switch(xyzSelect){
      case 0:
        GD.Vertex2f(hor+highlightHor,ver+6);
        GD.Vertex2f(hor+highlightHor+16,ver+35);
        GD.Vertex2f(hor+highlightHor,ver+76);
        GD.Vertex2f(hor+highlightHor+16,ver+105);
        GD.Vertex2f(hor+highlightHor,ver+146);
        GD.Vertex2f(hor+highlightHor+16,ver+175);
      break;
      case 1:
        GD.Vertex2f(hor+highlightHor,ver+6);
        GD.Vertex2f(hor+highlightHor+16,ver+35);
      break;
      case 2:
        GD.Vertex2f(hor+highlightHor,ver+76);
        GD.Vertex2f(hor+highlightHor+16,ver+105);
      break;
      case 3:
        GD.Vertex2f(hor+highlightHor,ver+146);
        GD.Vertex2f(hor+highlightHor+16,ver+175);
      break;
    }
    GD.ColorA(255);
  }
  
  GD.ColorRGB(0x00a000);
  GD.cmd_text(hor+270,ver+16,28,0,"mm");
  GD.cmd_text(hor+270,ver+86,28,0,"mm");
  GD.cmd_text(hor+270,ver+156,28,0,"mm");
  AxesDisplaySmall (hor+235,ver+47,16,x_mmDisplayAbs);
  AxesDisplaySmall (hor+235,ver+117,16,y_mmDisplayAbs);
  AxesDisplaySmall (hor+235,ver+187,16,z_mmDisplayAbs);
  GD.ColorRGB(0x808090);
  GD.cmd_text(hor+280,ver+47,16,0,"mm");
  GD.cmd_text(hor+280,ver+117,16,0,"mm");
  GD.cmd_text(hor+280,ver+187,16,0,"mm");
  GD.ColorRGB(0x000040);
  GD.cmd_text(hor+115,ver+51, 16, OPT_CENTERY, "Machine X:");
  GD.cmd_text(hor+115,ver+121, 16, OPT_CENTERY, "Machine Y:");
  GD.cmd_text(hor+115,ver+191, 16, OPT_CENTERY, "Machine Z:");
  GD.Begin(POINTS);
  GD.ColorRGB(0x000000);
  GD.PointSize(16 * 34);
  GD.Vertex2f((hor + 357), (ver + 26));
  GD.Vertex2f((hor + 357), (ver + 96));
  GD.Vertex2f((hor + 357), (ver + 166));
  GD.PointSize(xDialLEDSize);
  GD.ColorRGB(oDvStatLED[axisXstate]);
  GD.Vertex2f((hor + 357), (ver + 26));
  GD.PointSize(yDialLEDSize);
  GD.ColorRGB(oDvStatLED[axisYstate]);
  GD.Vertex2f((hor + 357), (ver + 96));
  GD.PointSize(zDialLEDSize);
  GD.ColorRGB(oDvStatLED[axisZstate]);
  GD.Vertex2f((hor + 357), (ver + 166));
  GD.ColorRGB(0x000000);
  GD.PointSize(16 * 30);
  GD.Vertex2f((hor + 357), (ver + 26));
  GD.Vertex2f((hor + 357), (ver + 96));
  GD.Vertex2f((hor + 357), (ver + 166));
  AxesDial(hor+325,ver-5,x_mmDisplay,0);
  AxesDial(hor+325,ver+65,y_mmDisplay,1);
  AxesDial(hor+325,ver+135,z_mmDisplay,2);
}

// Create large X,Y and Z Axis values to display with decimals
void AxesDisplay(int hor, int ver, int font, double axisVal){ //------------------------------------------------------void AxesDisplay()---
  GD.ColorRGB(0x00ff00);
  GD.cmd_number(hor,ver, font, OPT_RIGHTX | OPT_SIGNED, int(axisVal));
  GD.cmd_text(hor+2,ver,font,0,".");
  GD.cmd_number(hor+10,ver,font,4,int(10000 * abs(axisVal)));
  if (axisVal < 0 && axisVal > -1){
    GD.cmd_text(hor-41,ver,font,0,"-");
  }
  if (axisVal > 0 && axisVal < 10){
    GD.cmd_text(hor-50,ver,font,0,"+");
  }
  if (axisVal >= 10 && axisVal < 100){
    GD.cmd_text(hor-75,ver,font,0,"+");
  }
  if (axisVal >= 100 && axisVal < 1000){
    GD.cmd_text(hor-100,ver,font,0,"+");
  }
}

// Create small X,Y and Z Axis values to display with decimals (machine absolute positions)
void AxesDisplaySmall(int hor, int ver, int font, double axisVal){ //--------------------------------------------void AxesDisplaySmall()---
  GD.ColorRGB(0xc0c0f0);
  GD.cmd_number(hor,ver, font, OPT_RIGHTX | OPT_SIGNED, int(axisVal));
  GD.cmd_text(hor+2,ver,font,0,".");
  GD.cmd_number(hor+10,ver,font,4,int(10000 * abs(axisVal)));
  if (axisVal < 0 && axisVal > -1){
    GD.cmd_text(hor-15,ver,font,0,"-");
  }
  if (axisVal > 0 && axisVal < 10){
    GD.cmd_text(hor-15,ver,font,0,"+");
  }
  if (axisVal >= 10 && axisVal < 100){
    GD.cmd_text(hor-24,ver,font,0,"+");
  }
  if (axisVal >= 100 && axisVal < 1000){
    GD.cmd_text(hor-33,ver,font,0,"+");
  }
}

// Draw axes dials and rotate according to axis value being drawn
void AxesDial(int hor, int ver, double axisVal, int axisState){ //-------------------------------------------------------void AxesDial()---
  GD.Begin(BITMAPS);
  GD.ColorRGB(0xffffff);
  GD.BitmapHandle(1);
  GD.Vertex2f(hor, ver);
  GD.cmd_translate(F16(32.2),F16(30.9));
  GD.cmd_rotate(DEGREES(axisVal*327.68));
  GD.cmd_translate(F16(-32.2),F16(-30.9));
  GD.cmd_setmatrix();
  GD.BitmapHandle(2);
  GD.Vertex2f(hor, ver);
  GD.BitmapHandle(3);
  GD.Vertex2f(hor, ver);
  GD.cmd_loadidentity();
  GD.cmd_setmatrix();
}

// CONTROL THE MOVEMENT AND ENDSTOPS OF AXES DISPLAY
// Edges Horizontal = 35 to 297 (262 total) --- Vertical = 32 to 156 (124 total)
void AxesPanelMove(){ //--------------------------------------------------------------------------------------------void AxesPanelMove()---
  axDispHor = (-x_mmDisplayAbs*0.873333) + 166; // Center display = 166 (300mm travel = times by factor of 0.873333, keep answer as an interger as display doesnt require float) neg x_mmDisplay reverses panel movement direction
  axDispVer = (y_mmDisplayAbs*0.826666) + 94; // Center display = 94 (150mm travel = times by factor of 0.826666, keep answer as an interger as display doesnt require float)
  if(x_mmDisplayAbs < 0.05 && x_mmDisplayAbs > -0.05){
    axDispHor = 166;
  }
  if(y_mmDisplayAbs < 0.05 && y_mmDisplayAbs > -0.05){
    axDispVer = 94;
  }
}

void SpindleRotate(int hor, int ver, double spinPos){ //------------------------------------------------------------void SpindleRotate()---
  GD.Begin(POINTS);
  GD.ColorRGB(0xf0f0f0);
  GD.PointSize(16 * 32);
  GD.Vertex2f(hor + 30, ver + 29);
  GD.ColorRGB(0x000000);
  GD.PointSize(16 * 32);
  GD.Vertex2f(hor + 29, ver + 29);
  GD.Begin(BITMAPS);
  GD.ColorRGB(0xffffff);
  GD.BitmapHandle(9);
  GD.Vertex2f(hor+4, ver+4);
  GD.cmd_translate(F16(29.5),F16(29.4));
  GD.cmd_rotate(DEGREES(spinPos));
  GD.cmd_translate(F16(-29.5),F16(-29.4));
  GD.cmd_setmatrix();
  GD.BitmapHandle(10);
  GD.Vertex2f(hor, ver);
  GD.cmd_loadidentity();
  GD.cmd_setmatrix();
}

void DevRotate(int hor, int ver){ //------------------------------------------------------------------------------------void DevRotate()---
  GD.ColorRGB(0xffffff);
  GD.Begin(BITMAPS);
  GD.cmd_translate(F16(25),F16(21));
  GD.cmd_rotate(DEGREES(rot));
  GD.cmd_translate(F16(-25),F16(-21));
  GD.cmd_setmatrix();
  GD.BitmapHandle(0);
  GD.BitmapSize(NEAREST, BORDER, BORDER, 64, 64);
  GD.Vertex2f(hor, ver);
  rot++;
  if (rot > 359){
    rot = 0;
  }
  GD.cmd_loadidentity();
  GD.cmd_setmatrix();
}

void Dev255Rotate(void){ //------------------------------------------------------------------------------------------void Dev255Rotate()---
  if(dev255LoopTime1 + 10000 < millis()){
    if(dev255LoopTime2 + 100 < millis() && dev255LoopCount < 9){
      matrix.clear();
      matrix.setRotation(dev255LoopCount);
      matrix.drawBitmap(0, 0, dev255, 8, 8, LED_ON);
      matrix.writeDisplay();
      dev255LoopTime2 = millis();
      dev255LoopCount++;
      if(dev255LoopCount > 8){
        dev255LoopCount = 0;
        matrix.clear();
        matrix.setRotation(2);
        matrix.drawBitmap(0, 0, dev255, 8, 8, LED_ON);
        matrix.writeDisplay();
        dev255LoopTime1 = millis();
      }
    }
  }
}

void TimeWindow (int hor, int ver){ //---------------------------------------------------------------------------------void TimeWindow()---
  yr = year();
  mon = month();
  dy = day();
  hr = hour();
  mnt = minute();
  sec = second();

  GD.Begin(RECTS);
  GD.LineWidth(15*16);
  GD.Vertex2f(hor-30,ver-30);
  GD.Vertex2f(hor+120,ver+30);
  GD.ColorRGB(0x8030f0);
  GD.Begin(BITMAPS);
  GD.BitmapHandle(4);
  GD.BitmapSize(NEAREST, BORDER, BORDER, 184, 94);
  GD.LineWidth(16 * 20);
  GD.ColorMask(0,0,0,1);
  GD.BlendFunc(ONE, ONE);
  GD.ColorMask(1,1,1,0);
  GD.BlendFunc(DST_ALPHA, ONE_MINUS_DST_ALPHA);
  GD.Vertex2f(hor-47,ver-47);
  GD.ColorMask(1,1,1,1);
  GD.BlendFunc(SRC_ALPHA, ONE_MINUS_SRC_ALPHA);
  GD.ColorRGB(0xffffff);
  GD.cmd_clock(hor,ver,40,0,hr,mnt,sec,0);
  GD.cmd_number(hor+68,ver-14, 16, 2, hr);
  GD.cmd_text(hor+85,ver-10, 16, OPT_CENTER, ":");
  GD.cmd_number(hor+88,ver-14, 16, 2, mnt);
  GD.cmd_number(hor+55,ver+10, 16, OPT_CENTER, dy);
  GD.cmd_text(hor+67,ver+10, 16, OPT_CENTER, "/");
  GD.cmd_number(hor+79,ver+10, 16, OPT_CENTER, mon);
  GD.cmd_text(hor+92,ver+10, 16, OPT_CENTER, "/");
  GD.cmd_number(hor+113,ver+10, 16, OPT_CENTER, yr);
}

void ClockDisplayMove(double stepVal){ //------------------------------------------------------------------------void ClockDisplayMove()---
  if(clDispHor < 250 && clDispHorRight == true){
    clDispHor = clDispHor + stepVal;
  }else{
    if(int (clDispHor) == 250 && clDispHorRight == true){
      clDispHorRight = false;
    }
  }
  if(clDispHor > 45 && clDispHorRight == false){
    clDispHor = clDispHor - stepVal;
  }else{
    if(int (clDispHor) == 45 && clDispHorRight == false){
      clDispHorRight = true;
    }
  }
  if(clDispVer < 1235 && clDispVerDown == true){
    clDispVer = clDispVer + stepVal;
  }else{
    if(int (clDispVer) == 1235 && clDispVerDown == true){
      clDispVerDown = false;
    }
  }
  if(clDispVer > 955 && clDispVerDown == false){
    clDispVer = clDispVer - stepVal;
  }else{
    if(int (clDispVer) == 955 && clDispVerDown == false){
      clDispVerDown = true;
    }
  }
}

// Create a beveled box from top left (tl) and bottom right (br) corners
void CreateBevel(int tlPosX, int tlPosY, int brPosX, int brPosY, int colourA, int colourB){ //------------------------void CreateBevel()---
  GD.ColorRGB(colourA);
  GD.Vertex2f((brPosX-5),(tlPosY+2));
  GD.Vertex2f(brPosX,(brPosY-2));
  GD.ColorRGB(colourB);
  GD.Vertex2f((tlPosX+2),(tlPosY+2));
  GD.Vertex2f((brPosX-2),(brPosY-2));
}

void LEDflash(int hor, int ver){ //--------------------------------------------------------------------------------------void LEDflash()---
  GD.Begin(POINTS);
  if(loopLEDflash + 250 < millis()){
    loopLEDflash = millis();
    sequenceLEDs++;
    switch(sequenceLEDs){
      case 1:
        ledBlinkOn = true;
        ledFastOn = false;
        ledOn = false;
      break;
      case 2:
        ledBlinkOn = true;
        ledFastOn = true;
        ledOn = false;
      break;
      case 3:
        ledBlinkOn = true;
        ledFastOn = false;
        ledOn = true;
      break;
      case 4:
        ledBlinkOn = true;
        ledFastOn = true;
        ledOn = true;
      break;
      case 5:
        ledBlinkOn = false;
        ledFastOn = false;
        ledOn = false;
      break;
      case 6:
        ledBlinkOn = true;
        ledFastOn = true;
        ledOn = false;
      break;
      case 7:
        ledBlinkOn = true;
        ledFastOn = false;
        ledOn = true;
      break;
      case 8:
        ledBlinkOn = true;
        ledFastOn = true;
        ledOn = true;
        sequenceLEDs = 0;
      break;
  }
  if (axisXstate == 0 || axisXstate == 1 || axisXstate == 2 || axisXstate == 3 || axisXstate == 4 || axisXstate == 5 || axisXstate == 6 || axisXstate == 8 || axisXstate == 9){
    if(ledOn == false){
      xDialLEDSize = 0;
    }else{
      xDialLEDSize = 528;
    }
  }
  if (axisXstate == 7){
    xDialLEDSize = 528;
  }

  if (axisYstate == 0 || axisYstate == 1 || axisYstate == 2 || axisYstate == 3 || axisYstate == 4 || axisYstate == 5 || axisYstate == 6 || axisYstate == 8 || axisYstate == 9){
    if(ledOn == false){
      yDialLEDSize = 0;
    }else{
      yDialLEDSize = 528;
    }
  }
  if (axisYstate == 7){
    yDialLEDSize = 528;
  }
  if (axisZstate == 0 || axisZstate == 1 || axisZstate == 2 || axisZstate == 3 || axisZstate == 4 || axisZstate == 5 || axisZstate == 6 || axisZstate == 8 || axisZstate == 9){
    if(ledOn == false){
      zDialLEDSize = 0;
    }else{
      zDialLEDSize = 528;
    }
  }
  if (axisZstate == 7){
    zDialLEDSize = 528;
  }
  
  switch(startLEDMode){
    case 0:
      digitalWrite(startLED, LOW);
    break;
    case 1:
      digitalWrite(startLED, HIGH);
    break;
    case 2:
      if(ledOn == true){
        digitalWrite(startLED, HIGH);
      }else{
        digitalWrite(startLED, LOW);
      }
    break;
    case 3:
      if(ledFastOn == true){
        digitalWrite(startLED, HIGH);
      }else{
        digitalWrite(startLED, LOW);
      }
    break;
    case 4:
      if(ledBlinkOn == true){
        digitalWrite(startLED, HIGH);
      }else{
        digitalWrite(startLED, LOW);
      }
    break;
    case 5:
      if(ledBlinkOn == true){
        digitalWrite(startLED, LOW);
      }else{
        digitalWrite(startLED, HIGH);
      }
    break;
    case 6:
      if(ledOn == true){
        digitalWrite(startLED, LOW);
      }else{
        digitalWrite(startLED, HIGH);
      }
    break;
  }
  switch(pauseLEDMode){
    case 0:
      digitalWrite(pauseLED, LOW);
    break;
    case 1:
      digitalWrite(pauseLED, HIGH);
    break;
    case 2:
      if(ledOn == true){
        digitalWrite(pauseLED, HIGH);
      }else{
        digitalWrite(pauseLED, LOW);
      }
    break;
    case 3:
      if(ledFastOn == true){
        digitalWrite(pauseLED, HIGH);
      }else{
        digitalWrite(pauseLED, LOW);
      }
    break;
    case 4:
      if(ledBlinkOn == true){
        digitalWrite(pauseLED, HIGH);
      }else{
        digitalWrite(pauseLED, LOW);
      }
    break;
    case 5:
      if(ledBlinkOn == true){
        digitalWrite(pauseLED, LOW);
      }else{
        digitalWrite(pauseLED, HIGH);
      }
    break;
    case 6:
      if(ledOn == true){
        digitalWrite(pauseLED, LOW);
      }else{
        digitalWrite(pauseLED, HIGH);
      }
    break;
  }
  switch(stopLEDMode){
    case 0:
      digitalWrite(stopLED, LOW);
    break;
    case 1:
      digitalWrite(stopLED, HIGH);
    break;
    case 2:
      if(ledOn == true){
        digitalWrite(stopLED, HIGH);
      }else{
        digitalWrite(stopLED, LOW);
      }
    break;
    case 3:
      if(ledFastOn == true){
        digitalWrite(stopLED, HIGH);
      }else{
        digitalWrite(stopLED, LOW);
      }
    break;
    case 4:
      if(ledBlinkOn == true){
        digitalWrite(stopLED, HIGH);
      }else{
        digitalWrite(stopLED, LOW);
      }
    break;
    case 5:
      if(ledBlinkOn == true){
        digitalWrite(stopLED, LOW);
      }else{
        digitalWrite(stopLED, HIGH);
      }
    break;
    case 6:
      if(ledOn == true){
        digitalWrite(stopLED, LOW);
      }else{
        digitalWrite(stopLED, HIGH);
      }
    break;
    }
  }
}

// NOT YET USED
void GCodeDraw(void){ //------------------------------------------------------------------------------------------------void GCodeDraw()---
  GD.Begin(LINE_STRIP);
  GD.ColorRGB(0xffffff);
  GD.LineWidth(5);
  GD.Vertex2f(300,1000);
  GD.Vertex2f(300,1050);
  GD.Vertex2f(350,1050);
  GD.Vertex2f(350,1000);
  GD.Vertex2f(300,1000);
  GD.End();
}

// JOG MODE AND ZERO AXES, BOTH INDIVIDUALLY AND ALL TOGETHER
void JogMode(void){ //----------------------------------------------------------------------------------------------------void JogMode()---
  Serial.println("JogMode");
  xyzSelect = 0;
  bool case0FirstTime = true;
  bool xSet = false;
  bool ySet = false;
  xJogPos_m1 = xTab_m1;
  yJogPos_m0 = yTab_m0;
  xfirstLoop = true;
  yfirstLoop = true;
  
  while(KeyPadRetrieve() !=11){
    switch(KeyPadRetrieve()){
      case 1:
        KeyHeldWait();
        mmJogIncr = mmJogIncr * 10;
        if (mmJogIncr > 10){
          mmJogIncr = 10;
        }
        jogIncr = mmJogIncr * 16384;
      break;
      case 2:
        KeyHeldWait();
        if(xyzSelect == 0 || xyzSelect == 2){
          yJogPos_m0 = yTab_m0 + jogIncr;
          GlobalMove(jogSpeed,xJogPos_m1,jogSpeed,yJogPos_m0,0,0,yfirstLoop);
          yfirstLoop = false;
        }
      break;
      case 4:
        KeyHeldWait();
        if(xyzSelect == 0 || xyzSelect == 1){
          xJogPos_m1 = xTab_m1 - jogIncr;
          GlobalMove(jogSpeed,xJogPos_m1,jogSpeed,yJogPos_m0,0,0,xfirstLoop);
          xfirstLoop = false;
        }
      break;
      case 5:
        xyzSelect++;
        if (xyzSelect > 3){
          xyzSelect = 0;
        }
        EncoderPosition();
        KeyHeldWait();
      break;
      case 6:
        KeyHeldWait();
        if(xyzSelect == 0 || xyzSelect == 1){
          xJogPos_m1 = xTab_m1 + jogIncr;
          GlobalMove(jogSpeed,xJogPos_m1,jogSpeed,yJogPos_m0,0,0,xfirstLoop);
          xfirstLoop = false;
        }
      break;
      case 7:
        KeyHeldWait();
        mmJogIncr = mmJogIncr / 10;
        if (mmJogIncr < 0.0001){
          mmJogIncr = 0.0001;
        }
        jogIncr = mmJogIncr * 16384;
      break;
      case 8:
        KeyHeldWait();
        if(xyzSelect == 0 || xyzSelect == 2){
          yJogPos_m0 = yTab_m0 - jogIncr;
          GlobalMove(jogSpeed,xJogPos_m1,jogSpeed,yJogPos_m0,0,0,yfirstLoop);
          yfirstLoop = false;
        }
      break;
      case 11:
        KeyHeldWait();
      break;
      default:
        EncoderPosition();
      break;
    }

    if (backlashAdjust == true && oDrive0m0CLC == true && oDrive0m1CLC == true){
      switch (KeyPadRetrieve()){
        case 3: // Accept New X Backlash Value
          KeyHeldWait();
          if (xDisplay >= -16384 && xDisplay <= 16384 && xSet == false){
            if (xDisplay < 0){
              xDisplay = map(xDisplay, -1, -16384, 1, 16384);
            }
            xBacklash = xDisplay;
            if (xBacklash == 0){ // To prevent divide by zero errors
              xBacklash = 1;
            }
          }
          axisXstate = 7;
          xSet = true;
        break;
        case 9: // Accept New Y Backlash Value
          KeyHeldWait();
          if (yDisplay >= -16384 && yDisplay <= 16384 && ySet == false){
            if (yDisplay < 0){
              yDisplay = map(yDisplay, -1, -16384, 1, 16384);
            }
            yBacklash = yDisplay;
            if (xBacklash == 0){ // To prevent divide by zero errors
              yBacklash = 1;
            }
          }
          axisYstate = 6;
          ySet = true;
        break;
      }
    }
    
    switch (xyzSelect){
      default:
        EncoderPosition();
      break;
      case 0: // X,Y & Z-Axes Selected
        if (case0FirstTime == true){
          case0FirstTime = false;
        }
        if (KeyPadRetrieve() == 0){
          KeyHeldWait();
          xDisplay = 0;
          xDisplayOffset = xDisplay - xTab_m1;
          yDisplay = 0;
          yDisplayOffset = yDisplay - yTab_m0;
          EncoderPosition();
          xStatVal = 1;
          xZero = true;
          yStatVal = 1;
          yZero = true;
          zStatVal = 1;
          zZero = true;
        }
      break;
      case 1:                        // X-Axis Selected
        case0FirstTime = true;
        if (KeyPadRetrieve() == 0){
          KeyHeldWait();
          xDisplay = 0;
          xDisplayOffset = xDisplay - xTab_m1;
          EncoderPosition();
          xStatVal = 1;
          xZero = true;
        }
      break;
      case 2:                        // Y-Axis Selected
        case0FirstTime = true;
        if (KeyPadRetrieve() == 0){
          KeyHeldWait();
          yDisplay = 0;
          yDisplayOffset = yDisplay - yTab_m0;
          EncoderPosition();
          yStatVal = 1;
          yZero = true;
        }
      break;
      case 3:                        // Z-Axis Selected
        case0FirstTime = true;
        zStatVal = 0;
        zZero = true;
      break;
    }
  }
  if (xZero == true && yZero == true && zZero == true){
    zeroSet = true;
  }
  KeyHeldWait();
  cncModeVal = 0;
  xyzSelect = 4;
}

void GCodeRun(void){ //--------------------------------------------------------------------------------------void GCodeRun()---
  int gcRunCharPos = 0;
  int subReadCount = 0;
  s = 0; // Spindle RPM set to 0
  readCount = gcLiveCharPos;
  gCodeMode = 0;
  cncEnd = false;
  
  gcRunCharPos = gcLiveCharPos;
  gcASCIIval = memoryFile.charAt(gcRunCharPos);
  cncStatVal = 3;
  //gcRunningString = "";
  
  while(KeyPadRetrieve() !=10 && cncEnd == false){
    stopLEDMode = 0;
    pauseLEDMode = 0;
    startLEDMode = 1;
    
    GCodePause();
    //Serial.print("GCode First String = ");
    //Serial.println(gcFirstString);
 
    gcASCIIval = memoryFile.charAt(readCount);
    readCount++;
    if(cncEnd == true){
      gcASCIIval = 100; // To prevent the following switch carrying our GCode operations
      gcLiveCharPos = 0;
      gcCurrLineNo = 0;
      gcCurrLSP = 0;
      readCount = 0;
    }
    Serial.print("GCode LINE No: ");
    Serial.println(gcCurrLineNo);
    switch(gcASCIIval){
      case 10: // 'LF' line feed, used with 'CR'(13) carriage return (both denote a new line similar to '/n')
                      Serial.println("-LF--");
      break;
      case 13: // 'CR' carriage return, used with 'LF' above
                      Serial.print("--CR-");
        gcCurrLineNo++;
        EncoderPosition();
        preX = x;
        preY = y;
        preZ = z;
      break;
      case 32: // 'space' used to seperate commands and values
        Serial.print("_");
        gcFirstString += (char)gcASCIIval;
      break;
      case 37: // '%' used at the end of the full GCode program
        Serial.println("% - Used to stop Fans, Blower, Change Lights");
        digitalWrite(inverter, HIGH); // Stop air pump via 240V inverter
        digitalWrite(cabinetFans, LOW); // Stop the cabinet fans
        SpindleRampDown(0);
        cncEnd = true;
        gcLiveCharPos = 0;
        gcCurrLineNo = 0;
        gcCurrLSP = 0;
        readCount = 0;
      break;
      case 40: // '(' used at the beginning of a GCode section description, put the contents of braces into CurrentFeature string for display
                      Serial.print("(");
        combineChars = "";
        do{
          gcASCIIval = memoryFile.charAt(readCount);
          readCount++;
          gcFirstString += (char)gcASCIIval;
          combineChars += (char)gcASCIIval;
          subReadCount++;
        }while(gcASCIIval != 41);
        Serial.print(combineChars);
        Serial.println("");
        gcFirstString.remove(subReadCount-1,1);
        combineChars.remove(subReadCount-1,1);
        currentFeature = combineChars;
        subReadCount = 0;
      break;
      case 65: // 'A' Denotes 4th Axis movement followed by move end value
                      Serial.println("GCode A - Denotes 4th Axis movement followed by move end value");
        a = GCodeValueBuild();
        gcASCIIval = memoryFile.charAt(readCount);
        readCount++;
        if(gcASCIIval == 13){ // Execute move only if it's the last value on that line
          Serial.println("----------");
          ExecuteGCodeMove();
          gcCurrentString = gcFirstString;
        }
        readCount--;
      break;
      case 66: // 'B' Denotes 5th Axis movement followed by move end value
                      Serial.println("GCode B - Denotes 5th Axis movement followed by move end value");
        b = GCodeValueBuild();
        gcASCIIval = memoryFile.charAt(readCount);
        readCount++;
        if(gcASCIIval == 13){ // Execute move only if it's the last value on that line
          Serial.println("----------");
          ExecuteGCodeMove();
        }
        readCount--;
      break;
      case 67: // 'C' Denotes 6th Axis movement followed by move end value
                      Serial.println("GCode C - Denotes 6th Axis movement followed by move end value");
        c = GCodeValueBuild();
        gcASCIIval = memoryFile.charAt(readCount);
        readCount++;
        if(gcASCIIval == 13){ // Execute move only if it's the last value on that line
          Serial.println("----------");
          ExecuteGCodeMove();
        }
        readCount--;
      break;
      case 68: // 'D' Tool diameter offset used
                      Serial.println("GCode D - Tool diameter offset used");
      break;
      case 70: // 'F' Feed rate
                      Serial.print("GCode F - Feed rate = ");
        f = int(GCodeValueBuild());
        Serial.println(f);
        gcASCIIval = memoryFile.charAt(readCount);
        readCount++;
        if(gcASCIIval == 13){ // Execute move only if it's the last value on that line
          Serial.println("----------");
          ExecuteGCodeMove();
        }
        readCount--;
      break;
      case 71: // 'G' Preparatory function, G followed by a numerical code, specifies machining modes and functions
        gcASCIIval = memoryFile.charAt(readCount);
        readCount++;
        switch(gcASCIIval){
          case 48:
            gcASCIIval = memoryFile.charAt(readCount);
            if(gcASCIIval != 32){
            }
            readCount++;
            switch(gcASCIIval){
              case 48:// G00 - Rapid Move
                      Serial.println("GCode G00 - Rapid Move");
                gCodeMode = 0;
              break;
              case 49:// G01 - Linear Feed Move
                      Serial.println("GCode G01 - Linear Feed Move");
                gCodeMode = 1;
              break;
              case 50:// G02 - Clockwise Arc Feed Move
                      Serial.println("GCode G02 - Clockwise Arc Feed Move");
                gCodeMode = 2;
              break;
              case 51:// G03 - Counter Clockwise Arc Feed Move
                      Serial.println("GCode G03 - Counter Clockwise Arc Feed Move");
                gCodeMode = 3;
              break;
              case 52:// G04 - Dwell
                      Serial.println("GCode G04 - Dwell");
                gCodeDwell = true;
              break;
              case 57:// G09 - Exact stop
                      Serial.println("GCode G09 - Exact stop");
              break;
              default:// G0 - Rapid Move G00, most GCode miss the preceding 0
                      Serial.println("GCode G0 - Rapid Move G00");
                gCodeMode = 0;
                readCount--;
              break;
            }
          break;
          case 49:
            gcASCIIval = memoryFile.charAt(readCount);
            if(gcASCIIval != 32){
            }
            readCount++;
            switch(gcASCIIval){
              case 48:// G10 - Fixture and Tool Offset Setting
                      Serial.println("GCode G10 - Fixture and Tool Offset Setting");
              break;
              case 50:// G12 - Clockwise Circle
                      Serial.println("GCode G12 - Clockwise Circle");
                gCodeMode = 12;
              break;
              case 51:// G13 - Counter Clockwise Circle
                      Serial.println("GCode G13 - Counter Clockwise Circle");
                gCodeMode = 13;
              break;
              case 53:// G15 - Polar Coordinate Cancel
                      Serial.println("GCode G15 - Polar Coordinate Cancel");
              break;
              case 54:// G16 - Polar Coordinate
                      Serial.println("GCode G16 - Polar Coordinate");
              break;
              case 55:// G17 - XY Plane Select
                      Serial.println("GCode G17 - XY Plane Select");
                planeSelect = 17;
              break;
              case 56:// G18 - ZX Plane Select
                      Serial.println("GCode G18 - ZX Plane Select");
                planeSelect = 18;
              break;
              case 57:// G19 - YZ Plane Select
                      Serial.println("GCode G19 - YZ Plane Select");
                planeSelect = 19;
              break;
              default:// G1 - Linear Feed Move G01, most GCode miss the preceding 0
                      Serial.println("GCode G1 - Linear Feed Move G01");
                gCodeMode = 1;
                readCount--;
              break;
            }
          break;
          case 50:
            gcASCIIval = memoryFile.charAt(readCount);
            if(gcASCIIval != 32){
            }
            readCount++;
            switch(gcASCIIval){
              case 48:// G20 - Inch Unit Measurement
                      Serial.println("GCode G20 - Inch Unit Measurement");
              break;
              case 49:// G21 - Millimeter Unit Measurement
                      Serial.println("GCode G21 - Millimeter Unit Measurement");
              break;
              case 56:// G28 - Zero Return
                      Serial.println("GCode G28 - Zero Return");
                gCodeMode = 28;
              break;
              default:// G2 - Clockwise Arc Feed Move G02, most GCode miss the preceding 0
                      Serial.println("GCode G2 - Clockwise Arc Feed Move");
                gCodeMode = 2;
                readCount--;
              break;
            }
          break;
          case 51:
            gcASCIIval = memoryFile.charAt(readCount);
            if(gcASCIIval != 32){
            }
            readCount++;
            switch(gcASCIIval){
              case 48:// G30 - 2nd, 3rd, 4th Zero Return
                Serial.println("GCode G30 - 2nd, 3rd, 4th Zero Return");
                gCodeMode = 30;
              break;
              case 49:// G31 - Probe Function
                Serial.println("GCode G31 - Probe Function");
                gCodeMode = 31;
              break;
              case 50:// G32 - Threading
                Serial.println("GCode G32 - Threading");
                gCodeMode = 32;
              break;
              default:// G3 - Counter Clockwise Arc Feed Move G03, most GCode miss the preceding 0
                Serial.println("GCode G3 - Counter Clockwise Arc Feed Move");
                gCodeMode = 3;
                readCount--;
              break;
            }
          break;
          case 52:
            gcASCIIval = memoryFile.charAt(readCount);
            if(gcASCIIval != 32){
            }
            readCount++;
            switch(gcASCIIval){
              case 48:// G40 - Cutter Compensation Cancel
                Serial.println("GCode G40 - Cutter Compensation Cancel");
              break;
              case 49:// G41 - Cutter Compensation Left
                Serial.println("GCode G41 - Cutter Compensation Left");
              break;
              case 50:// G42 - Cutter Compensation Right
                Serial.println("GCode G42 - Cutter Compensation Right");
              break;
              case 51:// G43 - Tool Length Offset + Enable
                Serial.println("GCode G43 - Tool Length Offset + Enable");
              break;
              case 52:// G44 - Tool Length Offset - Enable
                Serial.println("GCode G44 - Tool Length Offset - Enable");
              break;
              case 57:// G49 - Tool Length Offset Cancel
                Serial.println("GCode G49 - Tool Length Offset Cancel");
              break;
              default:// G4 - Dwell G04, most GCode miss the preceding 0
                Serial.println("GCode G4 - Dwell");
                gCodeDwell = true;
              break;
            }
          break;
          case 53:
            gcASCIIval = memoryFile.charAt(readCount);
            if(gcASCIIval != 32){
            }
            readCount++;
            switch(gcASCIIval){
              case 48:// G50 - Cancel Scaling
                Serial.println("GCode G50 - Cancel Scaling");
              break;
              case 49:// G51 - Scale Axes
                Serial.println("GCode G51 - Scale Axes");
              break;
              case 50:// G52 - Local Coordinate System Shift
                Serial.println("GCode G52 - Local Coordinate System Shift");
              break;
              case 51:// G53 - Machine Coordinate System
                Serial.println("GCode G53 - Machine Coordinate System");
              break;
              case 52:// G54 - Fixture Offset 1
                Serial.println("GCode G54 - Fixture Offset 1");
                gcASCIIval = memoryFile.charAt(readCount);
                readCount++;
                if(gcASCIIval == 46){
                  gcASCIIval = memoryFile.charAt(readCount);
                  readCount++;
                  switch(gcASCIIval){
                    case 49:// G54.1 - Additional Fixture Offsets
                      Serial.println("GCode G54.1 - Additional Fixture Offsets");
                    break;
                  }
                }else{
                  readCount--;
                }
              break;
              case 53:// G55 - Fixture Offset 2
                      Serial.println("GCode G55 - Fixture Offset 2");
              break;
              case 54:// G56 - Fixture Offset 3
                      Serial.println("GCode G56 - Fixture Offset 3");
              break;
              case 55:// G57 - Fixture Offset 4
                      Serial.println("GCode G57 - Fixture Offset 4");
              break;
              case 56:// G58 - Fixture Offset 5
                      Serial.println("GCode G58 - Fixture Offset 5");
              break;
              case 57:// G59 - Fixture Offset 6
                      Serial.println("GCode G59 - Fixture Offset 6");
              break;
            }
          break;
          case 54:
            gcASCIIval = memoryFile.charAt(readCount);
            if(gcASCIIval != 32){
            }
            readCount++;
            switch(gcASCIIval){
              case 48:// G60 - Unidirectional Approach
                      Serial.println("GCode G60 - Unidirectional Approach");
              break;
              case 49:// G61 - Exact Stop Mode
                      Serial.println("GCode G61 - Exact Stop Mode");
              break;
              case 52:// G64 - Cutting Mode (Constant Velocity)
                      Serial.println("GCode G64 - Cutting Mode (Constant Velocity)");
              break;
              case 53:// G65 - Macro Call
                      Serial.println("GCode G65 - Macro Call");
              break;
              case 54:// G66 - Macro Modal Call
                      Serial.println("GCode G66 - Macro Modal Call");
              break;
              case 55:// G67 - Macro Modal Call Cancel
                      Serial.println("GCode G67 - Macro Modal Call Cancel");
              break;
              case 56:// G68 - Coordinate System Rotation
                      Serial.println("GCode G68 - Coordinate System Rotation");
              break;
              case 57:// G69 - Coordinate System Rotation Cancel
                      Serial.println("GCode G69 - Coordinate System Rotation Cancel");
              break;
            }
          break;
          case 55:
            gcASCIIval = memoryFile.charAt(readCount);
            if(gcASCIIval != 32){
            }
            readCount++;
            switch(gcASCIIval){
              case 51:// G73 - High Speed Peck Drilling
                      Serial.println("GCode G73 - High Speed Peck Drilling");
              break;
              case 52:// G74 - LH Tapping
                      Serial.println("GCode G74 - LH Tapping");
              break;
              case 54:// G76 - Fine Boring
                      Serial.println("GCode G76 - Fine Boring");
              break;
            }
          break;
          case 56:
            gcASCIIval = memoryFile.charAt(readCount);
            if(gcASCIIval != 32){
            }
            readCount++;
            switch(gcASCIIval){
              case 48:// G80 - Canned Cycle Cancel
                      Serial.println("GCode G80 - Canned Cycle Cancel");
              break;
              case 49:// G81 - Hole Drilling
                      Serial.println("GCode G81 - Hole Drilling");
              break;
              case 50:// G82 - Spot Face
                      Serial.println("GCode G82 - Spot Face");
              break;
              case 51:// G83 - Deep Hole Peck Drilling
                      Serial.println("GCode G83 - Deep Hole Peck Drilling");
              break;
              case 52:// G84 - RH Tapping
                Serial.println("GCode G84 - RH Tapping");
                gcASCIIval = memoryFile.charAt(readCount);
                readCount++;
                if(gcASCIIval == 46){
                  gcASCIIval = memoryFile.charAt(readCount);
                  readCount++;
                  switch(gcASCIIval){
                    case 50:// G84.2 - Additional Fixture Offsets
                      Serial.println("GCode G84.2 - Additional Fixture Offsets");
                    break;
                    case 51:// G84.3 - Additional Fixture Offsets
                      Serial.println("GCode G84.3 - Additional Fixture Offsets");
                    break;
                  }
                }else{
                  readCount--;
                }
              break;
              case 53:// G85 - Boring, Retract at Feed, Spindle On
                      Serial.println("GCode G85 - Boring, Retract at Feed, Spindle On");
              break;
              case 54:// G86 - Boring, Retract at Rapid, Spindle Off
                      Serial.println("GCode G86 - Boring, Retract at Rapid, Spindle Off");
              break;
              case 55:// G87 - Back Boring
                      Serial.println("GCode G87 - Back Boring");
              break;
              case 56:// G88 - Boring, Manual Retract
                      Serial.println("GCode G88 - Boring, Manual Retract");
              break;
              case 57:// G89 - Boring, Dwell, Retract at Feed, Spindle On
                      Serial.println("GCode G89 - Boring, Dwell, Retract at Feed, Spindle On");
              break;
            }
          break;
          case 57:
            gcASCIIval = memoryFile.charAt(readCount);
            if(gcASCIIval != 32){
            }
            readCount++;
            switch(gcASCIIval){
              case 48:// G90 - Absolute Position Mode
                Serial.println("GCode G90 - Absolute Position Mode");
                gcASCIIval = memoryFile.charAt(readCount);
                readCount++;
                if(gcASCIIval == 46){
                  gcASCIIval = memoryFile.charAt(readCount);
                  readCount++;
                  switch(gcASCIIval){
                    case 49:// G90.1 - Arc Center Absolute Mode
                      Serial.println("GCode G90.1 - Arc Center Absolute Mode");
                      absArcCenMode = true;
                    break;
                  }
                }else{
                  readCount--;
                  gcASCIIval = memoryFile.charAt(readCount);
                  absPosMode = true;
                }
              break;
              case 49:// G91 - Incremental Position Mode
                Serial.println("GCode G91 - Incremental Position Mode");
                gcASCIIval = memoryFile.charAt(readCount);
                readCount++;
                if(gcASCIIval == 46){
                  gcASCIIval = memoryFile.charAt(readCount);
                  readCount++;
                  switch(gcASCIIval){
                    case 49:// G91.1 - Arc Center Incremental Mode
                      Serial.println("GCode G91.1 - Arc Center Incremental Mode");
                      absArcCenMode = false;
                    break;
                  }
                }else{
                  readCount--;
                  gcASCIIval = memoryFile.charAt(readCount);
                  absPosMode = false;
                }
              break;
              case 50:// G92 - Local Coordinate System Setting
                Serial.println("GCode G92 - Local Coordinate System Seting");
                gcASCIIval = memoryFile.charAt(readCount);
                readCount++;
                if(gcASCIIval == 46){
                  gcASCIIval = memoryFile.charAt(readCount);
                  readCount++;
                  switch(gcASCIIval){
                    case 49:// G92.1 - Local Coordinate System Cancel
                      Serial.println("GCode G92.1 - Local Coordinate System Cancel");
                    break;
                  }
                }else{
                  readCount--;
                }
              break;
              case 51:// G93 - Inverse Time Feed
                      Serial.println("GCode G93 - Inverse Time Feed");
              break;
              case 52:// G94 - Feed per Minute
                      Serial.println("GCode G94 - Feed per Minute");
              break;
              case 53:// G95 - Feed per Revolution
                      Serial.println("GCode G95 - Feed per Revolution");
              break;
              case 54:// G96 - Constant Surface Speed
                      Serial.println("GCode G96 - Constant Surface Speed");
              break;
              case 55:// G97 - Constant Speed
                      Serial.println("GCode G97 - Constant Speed");
              break;
              case 56:// G98 - Initial Point Return
                      Serial.println("GCode G98 - Initial Point Return");
              break;
              case 57:// G99 - R Point Return
                      Serial.println("GCode G99 - R Point Return");
              break;
              default:// G9 - Exact Stop G09, most GCode miss the preceding 0
                      Serial.println("GCode G9 - Exact Stop");
              break;
            }
          break;
        }
      break;
      case 72: // 'H' Tool height offset
        Serial.print("GCode H - Height Offset = ");
        h = GCodeValueBuild();
        gcASCIIval = memoryFile.charAt(readCount);
        readCount++;
        Serial.println(h);
        if(gcASCIIval == 13){ // Execute move only if it's the last value on that line
          Serial.println("----------");
          ExecuteGCodeMove();
        }
        readCount--;
      break;
      case 73: // 'I' Used in Arc mode (G02, G03) as center of circle on X axis
        Serial.print("GCode I - Arc = ");
        i = GCodeValueBuild();
        gcASCIIval = memoryFile.charAt(readCount);
        readCount++;
        Serial.println(i);
        if(gcASCIIval == 13){ // Execute move only if it's the last value on that line
          Serial.println("----------");
          ExecuteGCodeMove();
        }
        readCount--;
      break;
      case 74: // 'J' Used in Arc mode (G02, G03) as center of circle on Y axis
        Serial.print("GCode J - Arc = ");
        j = GCodeValueBuild();
        gcASCIIval = memoryFile.charAt(readCount);
        readCount++;
        Serial.println(j);
        if(gcASCIIval == 13){ // Execute move only if it's the last value on that line
          Serial.println("----------");
          ExecuteGCodeMove();
        }
        readCount--;
      break;
      case 75: // 'K' Used in Arc mode (G02, G03) as center of circle on Z axis
        Serial.print("GCode K - Arc = ");
        k = GCodeValueBuild();
        gcASCIIval = memoryFile.charAt(readCount);
        readCount++;
        Serial.println(k);
        if(gcASCIIval == 13){ // Execute move only if it's the last value on that line
          Serial.println("----------");
          ExecuteGCodeMove();
        }
        readCount--;
      break;
      case 77: // 'M' Miscellaneous functions, M followed by a numerical code - Coolant, Tool Change, M30 end of program
        gcASCIIval = memoryFile.charAt(readCount);
        readCount++;
        switch(gcASCIIval){
          case 48: // GCode M0 - Mandatory Program Stop
          break;
          case 49:// GCode M1 - Optional Program Stop
          break;
          case 50:// GCode M2 - Program End 
          break;
          case 51:// GCode M3 - Spindle Forward/Clockwise
            gcASCIIval = memoryFile.charAt(readCount);
            readCount++;
            if(gcASCIIval == 48){ // GCode M30 - GCode End
              Serial.println("GCode M30 - GCode End");
              cncEnd = true;
              gcLiveCharPos = 0;
              gcCurrLineNo = 0;
              gcCurrLSP = 0;
              readCount = 0;
              gcLiveCharPos = readCount;
            }else{
              readCount--;
              Serial.println("GCode M3 - Used here to Start Fans, Blower, Change Lights");
              digitalWrite(cabinetFans, HIGH); // Start the cabinet fans
              digitalWrite(inverter, LOW); // Start air pump via 240V inverter
            }
          break;
          case 52:// GCode M4 - Spindle Reverse/Counterclockwise
          break;
          case 53:// GCode M5 - Spindle Stop 
          break;
          case 54:// GCode M6 - Tool Change
            Serial.println("GCode M6 - Tool Change");
            nextTool = false;
          break;
          case 55:// GCode M7 - Mist Coolant On
          break;
          case 56:// GCode M8 - Flood Coolant On
          break;
          case 57:// GCode M9 - All Coolant Off
          break;
        }
      break;
      case 78: // 'N' Sequence numbers to goto and run if required and used
                      Serial.println("GCode N - Sequence numbers to goto and run if required and used");
      break;
      case 80: // 'P' P followed by a numerical value denotes Dwell (G04) time in seconds
        Serial.print("GCode P - Dwell for ");
        p = int(GCodeValueBuild());
        gCodeDwellVal = p * 1000;
        Serial.print(p);
        Serial.println(" seconds");
      break;
      case 82: // 'R' Radius of a circle if using this feture in G02/G03
        Serial.println("GCode R - Radius");
        r = GCodeValueBuild();
        gcASCIIval = memoryFile.charAt(readCount);
        readCount++;
        if(gcASCIIval == 13){ // Execute move only if it's the last value on that line
          Serial.println("----------");
          ExecuteGCodeMove();
        }
        readCount--;
      break;
      case 83: // 'S' Spindle speed followed by numerical value in RPM
        Serial.print("GCode S - Spindle = ");
        s = GCodeValueBuild();
        gcASCIIval = memoryFile.charAt(readCount);
        readCount++;
        Serial.print(s);
        Serial.println(" RPM");
        if(s < spindleRPM - (s/1.01)){
          SpindleRampDown(s);
          if(s == 0){
            SpindleIdle();
          }
        }else{
          SpindleMode(4,s);
        }
        if(gcASCIIval == 13){ // Execute move only if it's the last value on that line
          Serial.println("----------");
          ExecuteGCodeMove();
        }
        readCount--;
      break;
      case 84: // 'T' Tool call, followed by next tool number, to prepare tool ready for change
        Serial.println("GCode T - Next Tool");
        gcASCIIval = memoryFile.charAt(readCount);
        readCount++;
        switch(gcASCIIval){
          case 48:
          nextTool = true;
          nextToolval = 0;
          break;
          case 49:
          nextTool = true;
          nextToolval = 1;
          break;
          case 50:
          nextTool = true;
          nextToolval = 2;
          break;
        }
      break;
      case 88: // 'X' Denotes 1st Axis movement followed by move end value
        Serial.print("GCode X - X-Axis = ");
        v = GCodeValueBuild();
        if(absPosMode == false){
          x = x + v;
        }else{
          x = v;
        }
        gcASCIIval = memoryFile.charAt(readCount);
        readCount++;
        Serial.println(x);
        if(gcASCIIval == 13){ // Execute move only if it's the last value on that line
          Serial.println("----------");
          ExecuteGCodeMove();
        }
        readCount--;
      break;
      case 89: // 'Y' Denotes 2nd Axis movement followed by move end value
        Serial.print("GCode Y - Y-Axis = ");
        v = GCodeValueBuild();
        if(absPosMode == false){
          y = y + v;
        }else{
          y = v;
        }
        gcASCIIval = memoryFile.charAt(readCount);
        readCount++;
        Serial.println(y);
        if(gcASCIIval == 13){ // Execute move only if it's the last value on that line
          Serial.println("----------");
          ExecuteGCodeMove();
        }
        readCount--;
      break;
      case 90: // 'Z' Denotes 3rd Axis movement followed by move end value
        Serial.print("GCode Z - Z-Axis = ");
        zManualMove = true;
        z = GCodeValueBuild();
        gcASCIIval = memoryFile.charAt(readCount);
        readCount++;
        Serial.println(z);
        if(gcASCIIval == 13){ // Execute move only if it's the last value on that line
          Serial.println("----------");
          ExecuteGCodeMove();
        }
        readCount--;
      break;
      default:
      break;
    }
    gcLiveCharPos = readCount;
  }
}

void GCodePause(void){
  int preSpindleRPM = spindleRPM;
  pausePress = digitalRead(pauseButton);
  stopPress = digitalRead(stopButton);
  doorOpen = digitalRead(safetyInterlock);
  if(pausePress == LOW || stopPress == LOW || doorOpen == HIGH || spindleRPM < adequateRPM){ // PAUSE GCODE
    odrive_serial << "r axis" << 1 << ".encoder.pos_estimate " << '\n';
    xEnc_m1 = odrive.readFloat();
    odrive.SetPosition(1, xEnc_m1);
    odrive_serial << "r axis" << 0 << ".encoder.pos_estimate " << '\n';
    yEnc_m0 = odrive.readFloat();
    odrive.SetPosition(0, yEnc_m0);
    //odrive_serial3 << "r axis" << 1 << ".encoder.pos_estimate " << '\n';
    //zEnc_m1 = odrive1.readFloat();
    //odrive1.SetPosition(1, zEnc_m1);
    gCodePause = true;
    cncStatVal = 4;
    Serial.println("----MACHINE PAUSED----");
    Serial.print("Pause = ");
    Serial.print(pausePress);
    Serial.print(", Stop = ");
    Serial.print(stopPress);
    Serial.print(", door = ");
    Serial.print(doorOpen);
    Serial.print(", Spindle Dropout = ");
    Serial.println(spindleDropout);
    digitalWrite(inverter, HIGH);
    digitalWrite(cabinetFans, LOW);
    SpindleRampDown(0);
    SpindleIdle();
    if(pausePress == LOW){
      while(pausePress == LOW){
        EncoderPosition();
        pauseLEDMode = 1;
        pausePress = digitalRead(pauseButton);
      }
    }
    if(stopPress == LOW){
      while(stopPress == LOW){
        EncoderPosition();
        stopLEDMode = 1;
        stopPress = digitalRead(stopButton);
      }
    }
    delay(100); // Switch De-Bounce
    startLEDMode = 2;
    pauseLEDMode = 6;
    stopLEDMode = 5;
    if(preSpindleRPM < adequateRPM){ // Crash causing spindle to slow and machine to pause
      Serial.println("SPINDLE CRASH DETECTED");
      Serial.print("preSpindleRPM = ");
      Serial.print(preSpindleRPM);
      Serial.print(", adequateRPM = ");
      Serial.println(adequateRPM);
      spindleDropout = true;
      stopLEDMode = 3;  // Fast flash Red LED
      startLEDMode = 5;
      pauseLEDMode = 3;
    }else{
      spindleDropout = false;
    }
    while(gCodePause == true){
      if(doorOpen == HIGH){
        pauseLEDMode = 3;
        SpindleRampDown(0);
        while(doorOpen == HIGH){
          EncoderPosition();
          doorOpen = digitalRead(safetyInterlock);
        }
        pauseLEDMode = 6;
      }else{
        pauseLEDMode = 6;
      }
      EncoderPosition();
      startPress = digitalRead(startButton);
      stopPress = digitalRead(stopButton);
      doorOpen = digitalRead(safetyInterlock);
      if(startPress == LOW){
        gCodePause = false;
        cncStatVal = 3;
        pauseLEDMode = 0;
        startLEDMode = 4;
        stopLEDMode = 5;
      }
      if(stopPress == LOW){
        gCodePause = false;
        cncEnd = true;
        gCodeMode = 100; // To stop any execution of GCode
        gcLiveCharPos = 0;
        gcCurrLineNo = 0;
        gcCurrLSP = 0;
        readCount = 0;
        pauseLEDMode = 0;
        startLEDMode = 5;
        stopLEDMode = 4;
      }
    }
    Serial.println("End of Paused while loop");
    if(cncEnd == true){
      digitalWrite(inverter, HIGH);
      digitalWrite(cabinetFans, LOW);
      cncStatVal = 2;
    }else{
      digitalWrite(inverter, LOW);
      digitalWrite(cabinetFans, HIGH);
      SpindleMode(4,s);
      odrive.SetPosition(1, xCNC_m1);
      //odrive1.SetPosition(1, zEnc_m1);
      odrive.SetPosition(0, yCNC_m0);
    }
  }
}

void SpindleMode(int mode){
  Serial.print("SpindleMode, spindleMode = ");
  Serial.println(mode);
  switch(mode){
    case 0: // Spindle needs calibrating
    break;
    case 1: // Spindle stopped and CW position cancelled (can't spin)
    case 2: // Spindle Idle
      SpindleIdle();
    break;
    case 3: // Spindle in Closed Loop Contol and stationary (must be previously calibrated)
      SpindleReady();
    break;
    case 4: // Spindle Running
      SpindleStart(spindleSetSpeed);
    break;
    case 5: // Spindle in position control for tapping (Not Yet Setup)##############
    break;
    default:
    break;
  }
}

void SpindleMode(int mode, int newSpeed){
  Serial.print("SpindleMode 2, spindleMode = ");
  Serial.print(mode);
  Serial.print(", newSpeed = ");
  Serial.println(newSpeed);
  switch(mode){
    case 4: // Spindle Running
      SpindleRampUp(newSpeed);
    break;
    default:
    break;
  }
}

void SpindleReady(void){
  Serial.println("SpindleReady");
  int requested_state;  
  requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
  odrive1.run_state(0, requested_state, false);
  spindleMode = 3;
  oDvStatVal = 5;
  axisSstate = 5;
}

void SpindleStart(int requestedRPM){
  Serial.println("SpindleStart");
  int requested_state;
  int requestedVelocity = 0;
  requestedVelocity = requestedRPM * spindleCPR / 60; 
  requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
  odrive1.run_state(0, requested_state, false);
  delay(10);
  Serial.print("requestedVelocity = ");
  Serial.println(requestedVelocity);
  odrive1.SetVelocity(0, requestedVelocity);
  while(spindleRPM * 1.1 < requestedRPM){
    EncoderPosition();
  }
  Serial.print("SpindleStart, spindleRPM = ");
  Serial.println(spindleRPM);
  spindleMode = 4;
  oDvStatVal = 7;
  axisSstate = 7;
  adequateRPM = requestedRPM * crashFactor; // Used for crash detection
}

void SpindleStop(void){
  unsigned int spindleDownTime = 0;
  Serial.println("SpindleStop - AXIS_STATE_IDLE");
  int requested_state;
  requested_state = ODriveArduino::AXIS_STATE_IDLE;
  odrive1.run_state(0, requested_state, false);
  spindleDownTime = millis() + 5000;
  Serial.println("SpindleStop - Spindle run down");
  while(spindleRPM != 0 || spindleDownTime < millis()){
    EncoderPosition();
  }
  odrive1.SetVelocity(0, 0);
  spindleMode = 1;
  oDvStatVal = 1;
  axisSstate = 1;
  adequateRPM = 0; // No crash detection
}

void SpindleIdle(void){
  Serial.println("SpindleIdle");
  int requested_state;
  requested_state = ODriveArduino::AXIS_STATE_IDLE;
  odrive1.run_state(0, requested_state, false);
  spindleMode = 2;
  oDvStatVal = 1;
  axisSstate = 1;
  adequateRPM = 0; // No crash detection
}

void SpindleRampUp(int rampUpRPM){
  Serial.print("SpindleRampUp, rampUpRPM = ");
  Serial.println(rampUpRPM);
  double gradIncrease = 0;
  gradIncrease = (spindleRPM * spindleCPR) / 60;
  Serial.print("rampUpRPM = ");
  Serial.println(rampUpRPM);
  velocityMax = (spindleRPM_Max * spindleCPR) / 60;
  odrive_serial3 << "w axis" << 0 << ".controller.config.vel_limit " << velocityMax << '\n';
  odrive1.SetVelocity(0, gradIncrease);
  if(spindleRPM == 0){
    SpindleStart(speedStep);
    gradIncrease = (speedStep * spindleCPR) / 60;
  }
  while(spindleRPM * 1.01 < rampUpRPM){
    Serial.print("  gradIncrease = ");
    Serial.println(gradIncrease);
    gradIncrease = gradIncrease + 10000;
    odrive1.SetVelocity(0, gradIncrease);
    delay(1);
    EncoderPosition();
    if(gradIncrease > velocityMax){
      break;
    }
  }
  gradIncrease = (rampUpRPM * spindleCPR) / 60;
  odrive1.SetVelocity(0, gradIncrease);
  spindleMode = 4;
  while(spindleRPM < rampUpRPM){
    delay(1);
    EncoderPosition();
  }
  unsigned int spindleSettleDwell = millis() + 2000;
  if(gCodeMode != 100){
    while(spindleSettleDwell > millis()){
      delay(1);
      EncoderPosition();
    }
  }
  adequateRPM = rampUpRPM * crashFactor; // Used for crash detection
}

void SpindleRampDown(int rampDownRPM){
  Serial.print("SpindleRampDown, rampDownRPM = ");
  Serial.println(rampDownRPM);
  float gradDecrease = spindleRPM / 60 * spindleCPR;
  while(spindleRPM > rampDownRPM && gradDecrease > 16000){
    Serial.print("SpindleRampDown, gradDecrease = ");
    Serial.println(gradDecrease);
    gradDecrease = gradDecrease - 15000;
    odrive1.SetVelocity(0, gradDecrease);
    delay (10);    
    EncoderPosition();
  }
  if(rampDownRPM < 1 && rampDownRPM > -1){
    odrive1.SetVelocity(0, 0);
  }
  adequateRPM = rampDownRPM * crashFactor; // No crash detection when rampDownRPM = 0
}

void ExecuteGCodeMove(void){
  Serial.println("EXECUTE GCODE MOVE");
  unsigned long pauseTime = 0;
  if(zManualMove == true){
    startLEDMode = 2;
    stopLEDMode = 5;
    startPress = digitalRead(startButton);
    while(startPress == HIGH){
      EncoderPosition();
      displayZAxisSet = true;
      stopPress = digitalRead(stopButton);
      if(stopPress == LOW){
        cncEnd = true;
        gCodeMode = 100; // Stops the Execution of GCode move
        gcLiveCharPos = 0;
        gcCurrLineNo = 0;
        gcCurrLSP = 0;
        readCount = 0;
        break;
      }
      startPress = digitalRead(startButton);
    }
    KeyHeldWait();
    displayZAxisSet = false;
    zManualMove = false;
  }
  pauseTime = millis();
  if(gCodeDwell == true){
    startLEDMode = 4;
    pauseLEDMode = 2;
    while(gCodeDwellVal + pauseTime > millis()){
      EncoderPosition();
    }
    gCodeDwell = false;
    startLEDMode = 1;
    pauseLEDMode = 5;
  }
  switch(gCodeMode){
    case 0:
      G00(x,y,z);
    break;
    case 1:
      G01(x,y,z,f);
    break;
    case 2:
      if(absArcCenMode == false){
        i = i + preX;
        j = j + preY;
        k = k + preZ;
      }
      G02(x,y,z,i,j,k,f);
      i = 0;
      j = 0;
      k = 0;
    break;
    case 3:
      if(absArcCenMode == false){
        i = i + preX;
        j = j + preY;
        k = k + preZ;
      }
      G03(x,y,z,i,j,k,f);
      i = 0;
      j = 0;
      k = 0;
    break;
  }
}
  
double GCodeValueBuild (void){
  String valueString = "";
  double value = 0.0d;
  gcASCIIval = memoryFile.charAt(readCount);
  readCount++;
  while(gcASCIIval != 32 && gcASCIIval != 13){
    if((gcASCIIval >= 48 && gcASCIIval <= 57) || gcASCIIval == 43 || gcASCIIval == 45 || gcASCIIval == 46){
      valueString += char(gcASCIIval);
      gcFirstString += (char)gcASCIIval;
    }
    gcASCIIval = memoryFile.charAt(readCount);
    readCount++;
  }
  readCount--;
  value = valueString.toFloat();
  return value;
}

//===\/ G CODE COMMANDS \/=========================================================================================\/ G CODE COMMANDS \/===

void G00 (double xDestination_mm, double yDestination_mm, double zDestination_mm){
  LineMill (327680, xDestination_mm, 327680, yDestination_mm, 0, zDestination_mm);
}

void G01 (double xDestination_mm, double yDestination_mm, double zDestination_mm, double feedSpeed_mmpm){
  xyzSpeed = (long)(feedSpeed_mmpm * feedSpeedAdjust / 60); // -16384-
  LineMill (xyzSpeed, xDestination_mm, xyzSpeed, yDestination_mm, xyzSpeed, zDestination_mm);
}

void G02 (double xDestination_mm, double yDestination_mm, double zDestination_mm, double iCenter_mm, double jCenter_mm, double kCenter_mm, double feedSpeed_mmpm){
  double xDiffStart = 0.0d;
  double yDiffStart = 0.0d;
  double zDiffStart = 0.0d;
  double xDiffEnd = 0.0d;
  double yDiffEnd = 0.0d;
  double zDiffEnd = 0.0d;
  double radius = 0.0d;
  double radiusStart = 0.0d;
  double radiusEnd = 0.0d;
  double radiusSq = 0.0d;
  double startRadians = 0.0d;
  double endRadians = 0.0d;
  double xEndOfArc_mm = 0.0d;
  double yEndOfArc_mm = 0.0d;
  double zEndOfArc_mm = 0.0d;
  long gfeedSpeed = 0;

  switch(planeSelect){
    case 17: // XY Plane Selected
      // Calculates the Current Position in mm
      xDiffStart = (double) xTab_m1 + (double) xDisplayOffset;
      yDiffStart = (double) yTab_m0 + (double) yDisplayOffset;
      xDiffStart = xDiffStart / 16384;
      yDiffStart = yDiffStart / 16384;
      xDiffStart = xDiffStart - iCenter_mm;
      yDiffStart = yDiffStart - jCenter_mm;
      startRadians = atan2 (yDiffStart,xDiffStart);
      
      // Calculates Radius (Hypotenuse) from Opposite & Adjacent from start position
      xDiffStart = sq(xDiffStart);
      yDiffStart = sq(yDiffStart);
      radiusSq = xDiffStart + yDiffStart;
      radiusStart = sqrt(radiusSq);
    
      // Calculates Radius (Hypotenuse) from Opposite & Adjacent from end position
      xDiffEnd = xDestination_mm - iCenter_mm;
      yDiffEnd = yDestination_mm - jCenter_mm;
      xEndOfArc_mm = xDiffEnd;
      yEndOfArc_mm = yDiffEnd;
      endRadians = atan2 (yDiffEnd,xDiffEnd);
      xDiffEnd = sq(xDiffEnd);
      yDiffEnd = sq(yDiffEnd);
      radiusSq = xDiffEnd + yDiffEnd;
      radiusEnd = sqrt(radiusSq);
    break;
    case 18: // ZX Plane Selected
      // Calculates the Current Position in mm
      zDiffStart = (double) zTab_m1 + (double) zDisplayOffset;
      xDiffStart = (double) xTab_m1 + (double) xDisplayOffset;
      zDiffStart = zDiffStart / 16384;
      xDiffStart = xDiffStart / 16384;
      zDiffStart = zDiffStart - kCenter_mm;
      xDiffStart = xDiffStart - iCenter_mm;
      startRadians = atan2 (zDiffStart,xDiffStart);
      
      // Calculates Radius (Hypotenuse) from Opposite & Adjacent from start position
      zDiffStart = sq(zDiffStart);
      xDiffStart = sq(xDiffStart);
      radiusSq = zDiffStart + xDiffStart;
      radiusStart = sqrt(radiusSq);
    
      // Calculates Radius (Hypotenuse) from Opposite & Adjacent from end position
      zDiffEnd = zDestination_mm - kCenter_mm;
      xDiffEnd = xDestination_mm - iCenter_mm;
      zEndOfArc_mm = zDiffEnd;
      xEndOfArc_mm = xDiffEnd;
      endRadians = atan2 (zDiffEnd,xDiffEnd);
      zDiffEnd = sq(zDiffEnd);
      xDiffEnd = sq(xDiffEnd);
      radiusSq = zDiffEnd + xDiffEnd;
      radiusEnd = sqrt(radiusSq);
    break;
    case 19: // YZ Plane Selected
      // Calculates the Current Position in mm
      yDiffStart = (double) yTab_m0 + (double) yDisplayOffset;
      zDiffStart = (double) zTab_m1 + (double) zDisplayOffset;
      yDiffStart = yDiffStart / 16384;
      zDiffStart = zDiffStart / 16384;
      yDiffStart = yDiffStart - jCenter_mm;
      zDiffStart = zDiffStart - kCenter_mm;
      startRadians = atan2 (zDiffStart,yDiffStart);
      
      // Calculates Radius (Hypotenuse) from Opposite & Adjacent from start position
      yDiffStart = sq(yDiffStart);
      zDiffStart = sq(zDiffStart);
      radiusSq = yDiffStart + zDiffStart;
      radiusStart = sqrt(radiusSq);
    
      // Calculates Radius (Hypotenuse) from Opposite & Adjacent from end position
      yDiffEnd = yDestination_mm - jCenter_mm;
      zDiffEnd = zDestination_mm - kCenter_mm;
      yEndOfArc_mm = yDiffEnd;
      zEndOfArc_mm = zDiffEnd;
      endRadians = atan2 (zDiffEnd,yDiffEnd);
      yDiffEnd = sq(yDiffEnd);
      zDiffEnd = sq(zDiffEnd);
      radiusSq = yDiffEnd + zDiffEnd;
      radiusEnd = sqrt(radiusSq);
    break;    
  }

  gfeedSpeed = feedSpeed_mmpm * feedSpeedAdjust_Cir; // -16384- 273 set for ar timing around a circle
  
  // Checks the Arc movement is in line with the last move (G01, then G02)
  if (radiusStart < radiusEnd + 0.0001 || radiusStart > radiusEnd - 0.0001){
    radius = radiusStart;
    ArcMillG02 (xEndOfArc_mm, yEndOfArc_mm, zEndOfArc_mm, iCenter_mm, jCenter_mm, kCenter_mm, startRadians, endRadians, radius, gfeedSpeed);
  }
}

void G03 (double xDestination_mm, double yDestination_mm, double zDestination_mm, double iCenter_mm, double jCenter_mm, double kCenter_mm, double feedSpeed_mmpm){
  double xDiffStart = 0.0d;
  double yDiffStart = 0.0d;
  double zDiffStart = 0.0d;
  double xDiffEnd = 0.0d;
  double yDiffEnd = 0.0d;
  double zDiffEnd = 0.0d;
  double radius = 0.0d;
  double radiusStart = 0.0d;
  double radiusEnd = 0.0d;
  double radiusSq = 0.0d;
  double startRadians = 0.0d;
  double endRadians = 0.0d;
  double xEndOfArc_mm = 0.0d;
  double yEndOfArc_mm = 0.0d;
  double zEndOfArc_mm = 0.0d;
  long gfeedSpeed = 0;

  switch(planeSelect){
    case 17: // XY Plane Selected
      // Calculates the Current Position in mm
      xDiffStart = (double) xTab_m1 + (double) xDisplayOffset;
      yDiffStart = (double) yTab_m0 + (double) yDisplayOffset;
      xDiffStart = xDiffStart / 16384;
      yDiffStart = yDiffStart / 16384;
      xDiffStart = xDiffStart - iCenter_mm;
      yDiffStart = yDiffStart - jCenter_mm;
      startRadians = atan2 (yDiffStart,xDiffStart);
      
      // Calculates Radius (Hypotenuse) from Opposite & Adjacent from start position
      xDiffStart = sq(xDiffStart);
      yDiffStart = sq(yDiffStart);
      radiusSq = xDiffStart + yDiffStart;
      radiusStart = sqrt(radiusSq);
    
      // Calculates Radius (Hypotenuse) from Opposite & Adjacent from end position
      xDiffEnd = xDestination_mm - iCenter_mm;
      yDiffEnd = yDestination_mm - jCenter_mm;
      xEndOfArc_mm = xDiffEnd;
      yEndOfArc_mm = yDiffEnd;
      endRadians = atan2 (yDiffEnd,xDiffEnd);
      xDiffEnd = sq(xDiffEnd);
      yDiffEnd = sq(yDiffEnd);
      radiusSq = xDiffEnd + yDiffEnd;
      radiusEnd = sqrt(radiusSq);
    break;
    case 18: // ZX Plane Selected
      // Calculates the Current Position in mm
      zDiffStart = (double) zTab_m1 + (double) zDisplayOffset;
      xDiffStart = (double) xTab_m1 + (double) xDisplayOffset;
      zDiffStart = zDiffStart / 16384;
      xDiffStart = xDiffStart / 16384;
      zDiffStart = zDiffStart - kCenter_mm;
      xDiffStart = xDiffStart - iCenter_mm;
      startRadians = atan2 (zDiffStart,xDiffStart); // swap arguments if undesired answer #######
      
      // Calculates Radius (Hypotenuse) from Opposite & Adjacent from start position
      zDiffStart = sq(zDiffStart);
      xDiffStart = sq(xDiffStart);
      radiusSq = zDiffStart + xDiffStart;
      radiusStart = sqrt(radiusSq);
    
      // Calculates Radius (Hypotenuse) from Opposite & Adjacent from end position
      zDiffEnd = zDestination_mm - kCenter_mm;
      xDiffEnd = xDestination_mm - iCenter_mm;
      zEndOfArc_mm = zDiffEnd;
      xEndOfArc_mm = xDiffEnd;
      endRadians = atan2 (zDiffEnd,xDiffEnd);
      zDiffEnd = sq(zDiffEnd);
      xDiffEnd = sq(xDiffEnd);
      radiusSq = zDiffEnd + xDiffEnd;
      radiusEnd = sqrt(radiusSq);
    break;
    case 19: // YZ Plane Selected
      // Calculates the Current Position in mm
      yDiffStart = (double) yTab_m0 + (double) yDisplayOffset;
      zDiffStart = (double) zTab_m1 + (double) zDisplayOffset;
      yDiffStart = yDiffStart / 16384;
      zDiffStart = zDiffStart / 16384;
      yDiffStart = yDiffStart - jCenter_mm;
      zDiffStart = zDiffStart - kCenter_mm;
      startRadians = atan2 (zDiffStart,yDiffStart);
      
      // Calculates Radius (Hypotenuse) from Opposite & Adjacent from start position
      yDiffStart = sq(yDiffStart);
      zDiffStart = sq(zDiffStart);
      radiusSq = yDiffStart + zDiffStart;
      radiusStart = sqrt(radiusSq);
    
      // Calculates Radius (Hypotenuse) from Opposite & Adjacent from end position
      yDiffEnd = yDestination_mm - jCenter_mm;
      zDiffEnd = zDestination_mm - kCenter_mm;
      yEndOfArc_mm = yDiffEnd;
      zEndOfArc_mm = zDiffEnd;
      endRadians = atan2 (zDiffEnd,yDiffEnd); // swap arguments if undesired answer #######
      yDiffEnd = sq(yDiffEnd);
      zDiffEnd = sq(zDiffEnd);
      radiusSq = yDiffEnd + zDiffEnd;
      radiusEnd = sqrt(radiusSq);
    break;    
  }
  
  gfeedSpeed = feedSpeed_mmpm * feedSpeedAdjust_Cir; // -16384- 200 set for timing around a circle
  
  if (radiusStart < radiusEnd + 0.0001 || radiusStart > radiusEnd - 0.0001){
    radius = radiusStart;
    ArcMillG03 (xEndOfArc_mm, yEndOfArc_mm, zEndOfArc_mm, iCenter_mm, jCenter_mm, kCenter_mm, startRadians, endRadians, radius, gfeedSpeed);
  }
}

//===/\ G CODE COMMANDS /\=========================================================================================/\ G CODE COMMANDS /\===

void ArcMillG02 (double xEndOfArc_mm, double yEndOfArc_mm, double zEndOfArc_mm, double iCentre_mm, double jCentre_mm, double kCentre_mm, double startRadians, double endRadians, double radius, long amfeedSpeed){
  double sin_n = 0.0d;
  double cos_n = 0.0d;
  double radStep = 0.0d;
  long xCirPos_m1 = 0;
  long yCirPos_m0 = 0;
  long zCirPos_m1 = 0;
  bool firstCircleLoop = true;
  long sin_cps = 0;
  long cos_cps = 0;
  double phase = 0.0d;

  startRadians = startRadians + 4*PI;
  endRadians = endRadians + 4*PI;

  if (startRadians <= endRadians){
    endRadians = endRadians - 2*PI;
  }
  phase = startRadians;
  
  xCirPos_m1 = iCentre_mm * 16384;
  yCirPos_m0 = jCentre_mm * 16384;
  zCirPos_m1 = kCentre_mm * 16384;
  xCirPos_m1 = xCirPos_m1 - xDisplayOffset;
  yCirPos_m0 = yCirPos_m0 - yDisplayOffset;
  zCirPos_m1 = zCirPos_m1 - zDisplayOffset;

  radStep = -PI * amfeedSpeed * 0.00000032 / radius; // 0.00000022 is the factor for radian steps to set speed in mm/s (tweak to adjust)
  
  if (amfeedSpeed < 32768){
    amfeedSpeed = 32768;
  }

  while (phase > endRadians && cncEnd == false) {
    if (phase + radStep < endRadians){
      phase = endRadians;
    }else{
      phase += radStep;
    }
    cos_n = radius * cos(phase);
    sin_n = radius * sin(phase);
    cos_n = cos_n * 16384;
    sin_n = sin_n * 16384;
    cos_cps = (long) cos_n;
    sin_cps = (long) sin_n;
    switch(planeSelect){
      case 17: // Any changes to 17, 18 or 19, change in Arc Mill 03 also #######
        xCirPos_m1 = xCirPos_m1 + cos_cps;
        yCirPos_m0 = yCirPos_m0 + sin_cps;
        GlobalMove(amfeedSpeed,xCirPos_m1,amfeedSpeed,yCirPos_m0,0,zCirPos_m1,firstCircleLoop);
        firstCircleLoop = false;
        EncoderPosition();
        GCodePause();
        xCirPos_m1 = xCirPos_m1 - cos_cps;
        yCirPos_m0 = yCirPos_m0 - sin_cps;
      break;
      case 18:
        zCirPos_m1 = zCirPos_m1 + cos_cps;
        xCirPos_m1 = xCirPos_m1 + sin_cps;
        GlobalMove(amfeedSpeed,xCirPos_m1,amfeedSpeed,yCirPos_m0,0,zCirPos_m1,firstCircleLoop);
        firstCircleLoop = false;
        EncoderPosition();
        GCodePause();
        zCirPos_m1 = zCirPos_m1 - cos_cps;
        xCirPos_m1 = xCirPos_m1 - sin_cps;
      break;
      case 19:
        yCirPos_m0 = yCirPos_m0 + cos_cps;
        zCirPos_m1 = zCirPos_m1 + sin_cps;
        GlobalMove(amfeedSpeed,xCirPos_m1,amfeedSpeed,yCirPos_m0,0,zCirPos_m1,firstCircleLoop);
        firstCircleLoop = false;
        EncoderPosition();
        GCodePause();
        yCirPos_m0 = yCirPos_m0 - cos_cps;
        zCirPos_m1 = zCirPos_m1 - sin_cps;
      break;
    }
  }
}

void ArcMillG03 (double xEndOfArc_mm, double yEndOfArc_mm, double zEndOfArc_mm, double iCentre_mm, double jCentre_mm, double kCentre_mm, double startRadians, double endRadians, double radius, long amfeedSpeed){
  double sin_n = 0.0d;
  double cos_n = 0.0d;
  double radStep = 0.0d;
  long xCirPos_m1 = 0;
  long yCirPos_m0 = 0;
  long zCirPos_m1 = 0;
  bool firstCircleLoop = true;
  long sin_cps = 0;
  long cos_cps = 0;
  double phase = 0.0d;

  startRadians = startRadians + 4*PI;
  endRadians = endRadians + 4*PI;

  if (startRadians >= endRadians){
    endRadians = endRadians + 2*PI;
  }
  phase = startRadians;
  
  xCirPos_m1 = iCentre_mm * 16384;
  yCirPos_m0 = jCentre_mm * 16384;
  zCirPos_m1 = kCentre_mm * 16384;
  xCirPos_m1 = xCirPos_m1 - xDisplayOffset;
  yCirPos_m0 = yCirPos_m0 - yDisplayOffset;
  zCirPos_m1 = zCirPos_m1 - zDisplayOffset;

  radStep = PI * amfeedSpeed * 0.00000032 / radius; // 0.00000022 is the factor for radian steps, used to set speed in mm/s (tweak to adjust)
  
  if (amfeedSpeed < 32768){
    amfeedSpeed = 32768;
  }

  while (phase < endRadians && cncEnd == false) {
    if (phase + radStep > endRadians){
      phase = endRadians;
    }else{
      phase += radStep;
    }
    cos_n = radius * cos(phase);
    sin_n = radius * sin(phase);
    cos_n = cos_n * 16384;
    sin_n = sin_n * 16384;
    cos_cps = (long) cos_n;
    sin_cps = (long) sin_n;
    switch(planeSelect){
      case 17: // Any changes to 17, 18 or 19, change in Arc Mill 03 also #######
        xCirPos_m1 = xCirPos_m1 + cos_cps;
        yCirPos_m0 = yCirPos_m0 + sin_cps;
        GlobalMove(amfeedSpeed,xCirPos_m1,amfeedSpeed,yCirPos_m0,0,zCirPos_m1,firstCircleLoop);
        firstCircleLoop = false;
        EncoderPosition();
        GCodePause();
        xCirPos_m1 = xCirPos_m1 - cos_cps;
        yCirPos_m0 = yCirPos_m0 - sin_cps;
      break;
      case 18:
        zCirPos_m1 = zCirPos_m1 + cos_cps;
        xCirPos_m1 = xCirPos_m1 + sin_cps;
        GlobalMove(amfeedSpeed,xCirPos_m1,amfeedSpeed,yCirPos_m0,0,zCirPos_m1,firstCircleLoop);
        firstCircleLoop = false;
        EncoderPosition();
        GCodePause();
        zCirPos_m1 = zCirPos_m1 - cos_cps;
        xCirPos_m1 = xCirPos_m1 - sin_cps;
      break;
      case 19:
        yCirPos_m0 = yCirPos_m0 + cos_cps;
        zCirPos_m1 = zCirPos_m1 + sin_cps;
        GlobalMove(amfeedSpeed,xCirPos_m1,amfeedSpeed,yCirPos_m0,0,zCirPos_m1,firstCircleLoop);
        firstCircleLoop = false;
        EncoderPosition();
        GCodePause();
        yCirPos_m0 = yCirPos_m0 - cos_cps;
        zCirPos_m1 = zCirPos_m1 - sin_cps;
      break;
    }
  }
}

void CircleMill(void){ //----------------------------------------------------------------------------------------------void CircleMill()---
  cncModeVal = 1;
  circleReady = false;
  matrix.clear();
  matrix.drawBitmap(0, 0, millCircle, 8, 8, LED_ON);
  matrix.writeDisplay();
  while (KeyPadRetrieve() != 11 || circleReady == false){
    EncoderPosition();
    switch(KeyPadRetrieve()){
      case 1:
        keypadValRequired = true;
        KeypadEnterValue();
        circleDiameter = keypadEnteredValue;
        keypadValRequired = false;
        circleDiaSet = true;
        KeyHeldWait();
        break;
      case 2:
        if (internalCircle == true){
          internalCircle = false;
        }else{
          internalCircle = true;
        }
        intSet = true;
        KeyHeldWait();
        break;
      case 3:
        keypadValRequired = true;
        KeypadEnterValue();
        circleDepth = keypadEnteredValue;
        keypadValRequired = false;
        depSet = true;
        KeyHeldWait();
        break;
      case 4:
        keypadValRequired = true;
        KeypadEnterValue();
        circleXcoord = keypadEnteredValue;
        depSet = true;
        keypadValRequired = false;
        cirXset = true;
        KeyHeldWait();
        break;
      case 5:
        keypadValRequired = true;
        KeypadEnterValue();
        circleYcoord = keypadEnteredValue;
        depSet = true;
        keypadValRequired = false;
        cirYset = true;
        KeyHeldWait();
        break;
      case 6:
        if (climbMill == true){
          climbMill = false;
        }else{
          climbMill = true;
        }
        climbSet = true;
        KeyHeldWait();
        break;
      case 7:
        keypadValRequired = true;
        KeypadEnterValue();
        toolDiameter = keypadEnteredValue;
        keypadValRequired = false;
        toolDiaSet = true;
        KeyHeldWait();
        break;
      case 8:
        keypadValRequired = true;
        KeypadEnterValue();
        feedSpeed = keypadEnteredValue;
        keypadValRequired = false;
        speedSet = true;
        KeyHeldWait();
        break;
      case 9:
        circleDiaSet = true;
        intSet = true;
        depSet = true;
        cirXset = true;
        cirYset = true;
        climbSet = true;
        toolDiaSet = true;
        speedSet = true;        
        break;
      EncoderPosition();
    }
  }
  startLEDMode = 1;
  pauseLEDMode = 5;
  stopLEDMode = 5;
  for(int cycles = 0; circleDepth > cycles; cycles++){
    if(internalCircle == true){
      G00(circleXcoord+(circleDiameter/2)-(toolDiameter/2),circleYcoord,0);
      if(climbMill == true){
        G03(circleXcoord+(circleDiameter/2)-(toolDiameter/2),circleYcoord,0,circleXcoord,circleYcoord,0,feedSpeed);
      }else{
        G02(circleXcoord+(circleDiameter/2)-(toolDiameter/2),circleYcoord,0,circleXcoord,circleYcoord,0,feedSpeed);
      }
    }else{
      G00(circleXcoord+(circleDiameter/2)+(toolDiameter/2),circleYcoord,0);
      if(climbMill == true){
        G02(circleXcoord+(circleDiameter/2)+(toolDiameter/2),circleYcoord,0,circleXcoord,circleYcoord,0,feedSpeed);
      }else{
        G03(circleXcoord+(circleDiameter/2)+(toolDiameter/2),circleYcoord,0,circleXcoord,circleYcoord,0,feedSpeed);
      }
    }
  }
  circleDiaSet = false;
  intSet = false;
  depSet = false;
  cirXset = false;
  cirYset = false;
  climbSet = false;
  toolDiaSet = false;
  speedSet = false;
  cncModeVal = 0;
  startLEDMode = 4;
  pauseLEDMode = 0;
  stopLEDMode = 5;
}

//-----------------------------------------------------------------------------------------------------------------------void LineMill()---
void LineMill (long xLMSpeed, double xLMDestination_mm, long yLMSpeed, double yLMDestination_mm, long zLMSpeed, long zLMDestination_mm){ // Mill or move from one point to another with interpollation (ALL Axes reach target at same time)  double loopTime = 0.01d; // Adjust to get the feed speed (mm/s) correct
  double loopTime = 0.02d; // Adjust to get the feed speed (mm/s) correct
  long xLMDestination = 0;
  long yLMDestination = 0;
  long zLMDestination = 0;
    long loopCount = 0;
    double progress = 0.0d; // Changed to a double from long
    double xDelta_m1 = 0.0d;
    double yDelta_m0 = 0.0d;
   // long zDelta_m2 = 0;  
    double xRatio = 0.0d;
    double yRatio = 0.0d;
   // double zRatio = 0.0f:
    double xStep = 0.0d;
    double yStep = 0.0d;
   // long zStep = 0;
    double xLinPos_m1 = 0.0d; // Changed to a double from long
    double yLinPos_m0 = 0.0d; // Changed to a double from long
   // long zLinPos_m2 = 0;
    bool firstLineLoop = true;
    bool xDeltaNeg = false;
    bool yDeltaNeg = false;
    bool xDeltaPrimary = true;
    bool xDeltaZero = false;
    bool yDeltaZero = false;
    long LMSpeedPassThrough = 0;
    xLMDestination = xLMDestination_mm * 16384;
    yLMDestination = yLMDestination_mm * 16384;
    xLMDestination = xLMDestination - xDisplayOffset;
    yLMDestination = yLMDestination - yDisplayOffset;
    xLinPos_m1 = xTab_m1;
    yLinPos_m0 = yTab_m0;
    
    if(xLMSpeed < 32768){
      LMSpeedPassThrough = 32768;
    }else{
      LMSpeedPassThrough = xLMSpeed;
    }
  
    xDelta_m1 = (double)xLMDestination - (double)xTab_m1;
    yDelta_m0 = (double)yLMDestination - (double)yTab_m0;
    
    if(xDelta_m1 == 0){ // removes the chance of divide by Zero issue
      xDeltaZero = true;
    }
    if(yDelta_m0 == 0){ // removes the chance of divide by Zero issue
      yDeltaZero = true;
    }
  
    if(xDelta_m1 < 0){
      xDelta_m1 = -xDelta_m1;
      xDeltaNeg = true;
    }
    if(yDelta_m0 < 0){
      yDelta_m0 = -yDelta_m0;
      yDeltaNeg = true;
    }
   
    xRatio = xDelta_m1 / yDelta_m0;
    yRatio = yDelta_m0 / xDelta_m1;
  
    if(xDelta_m1 > yDelta_m0){
      xDeltaPrimary = true;
      yLMSpeed = xLMSpeed * yRatio;
      xStep = (double)xLMSpeed * loopTime;
      yStep = (double)xLMSpeed * loopTime * yRatio;
    }else{
      xDeltaPrimary = false;
      yLMSpeed = xLMSpeed;
      xLMSpeed = yLMSpeed * xRatio;
      xStep = (double)yLMSpeed * loopTime * xRatio;
      yStep = (double)yLMSpeed * loopTime;
    }

  if((xLMSpeed >= 32768 && yLMSpeed >= 32768)||(xLMSpeed == 0 && yLMSpeed >= 32768)||(xLMSpeed >= 32768 && yLMSpeed == 0)){
    //Serial.println("----SPEED OF X & Y GREATER THAN 32768----");
    //Serial.print("xLMSpeed = ");
    //Serial.print(xLMSpeed);
    //Serial.print(", yLMSpeed = ");
    //Serial.print(yLMSpeed);
    //Serial.print(", zLMSpeed = ");
    //Serial.println(zLMSpeed);
    GlobalMove(xLMSpeed,xLMDestination,yLMSpeed,yLMDestination,0,zLMDestination,true);
    EncoderPosition();
    GCodePause();
  }else{
    //Serial.println("----SPEED OF X & Y LESS THAN 32768----");
    //Serial.print("xLMSpeed = ");
    //Serial.print(xLMSpeed);
    //Serial.print(", yLMSpeed = ");
    //Serial.print(yLMSpeed);
    //Serial.print(", zLMSpeed = ");
    //Serial.println(zLMSpeed);
  
    if(xDeltaNeg == true){ // re-apply the neagative
      if (xTab_m1 > xLMDestination){
        xDelta_m1 = -xDelta_m1;
        xStep = -xStep;
      }
    }
    if(yDeltaNeg == true){ // re-apply the neagative
      if (yTab_m0 > yLMDestination){
        yDelta_m0 = -yDelta_m0;
        yStep = -yStep;
      }
    }
  
    if (xDeltaPrimary == true){
      progress = xTab_m1;    
      //for(xLinPos_m1; xLinPos_m1 != progress + xDelta_m1;){
        if (xLMDestination - xStep <= xLinPos_m1 && xLMDestination + xStep >= xLinPos_m1 || xLMDestination + xStep <= xLinPos_m1 && xLMDestination - xStep >= xLinPos_m1){
          //Serial.print("XPri - X STEP SKIP, Step = ");
          //Serial.println(xStep);
          xLinPos_m1 = xLMDestination;
        }else{
          xLinPos_m1 = xLinPos_m1 + xStep;
          //Serial.print("X Pri - X STEP, Step = ");
          //Serial.println(xStep);
        }
        if (yLMDestination - yStep <= yLinPos_m0 && yLMDestination + yStep >= yLinPos_m0 || yLMDestination + yStep <= yLinPos_m0 && yLMDestination - yStep >= yLinPos_m0){
          //Serial.print("XPri - Y STEP SKIP, Step = ");
          //Serial.println(yStep);
          yLinPos_m0 = yLMDestination;
        }else{
          yLinPos_m0 = yLinPos_m0 + yStep;
          //Serial.print("XPri - Y STEP, Step = ");
          //Serial.println(yStep);
        }
        //  zLinPos_m2 = zLinPos_m2 - zStep;
        GlobalMove(LMSpeedPassThrough,xLinPos_m1,LMSpeedPassThrough,yLinPos_m0,0,0,firstLineLoop);
        firstLineLoop = false;
        EncoderPosition();
        GCodePause();
      //}
    }
    if (xDeltaPrimary == false){
      progress = yTab_m0;
      //for(yLinPos_m0; yLinPos_m0 != progress + yDelta_m0;){
        if (xLMDestination - xStep <= xLinPos_m1 && xLMDestination + xStep >= xLinPos_m1 || xLMDestination + xStep <= xLinPos_m1 && xLMDestination - xStep >= xLinPos_m1){
          //Serial.print("YPri - X STEP SKIP, Step = ");
          //Serial.println(xStep);
          xLinPos_m1 = xLMDestination;
        }else{
          //Serial.print("YPri - X STEP, Step = ");
          //Serial.println(xStep);
          xLinPos_m1 = xLinPos_m1 + xStep;
        }
        if (yLMDestination - yStep <= yLinPos_m0 && yLMDestination + yStep >= yLinPos_m0 || yLMDestination + yStep <= yLinPos_m0 && yLMDestination - yStep >= yLinPos_m0){
          //Serial.print("YPri - Y STEP SKIP, Step = ");
          //Serial.println(yStep);
          yLinPos_m0 = yLMDestination;
        }else{
          //Serial.print("YPri - Y STEP, Step = ");
          //Serial.println(yStep);
          yLinPos_m0 = yLinPos_m0 + yStep;
        }
        //  zLinPos_m2 = zLinPos_m2 - zStep;
        GlobalMove(LMSpeedPassThrough,xLinPos_m1,LMSpeedPassThrough,yLinPos_m0,0,0,firstLineLoop);
        firstLineLoop = false;
        EncoderPosition();
        GCodePause();
      //}
    }
  }
}

//-------------------------------------------------------------------------------------------------------------------void GlobalMovemm()---
// Global Move of all Axes in mm (Converter)
void GlobalMovemm (long xSpeedPassThrough, double xDestinationmm, long ySpeedPassThrough, double yDestinationmm, long zSpeedPassThrough, double zDestinationmm, bool firstLoopPassThrough){
  long xDestinationEC = 0;
  long yDestinationEC = 0;
  long zDestinationEC = 0;
  xDestinationEC = xDestinationmm * 16384;
  yDestinationEC = yDestinationmm * 16384;
  zDestinationEC = zDestinationmm * 16384;
  xDestinationEC = xDestinationEC - xDisplayOffset;
  yDestinationEC = yDestinationEC - yDisplayOffset;
//  zDestinationEC = zDestinationEC - zDisplayOffset;
  GlobalMove (xSpeedPassThrough, xDestinationEC, ySpeedPassThrough, yDestinationEC, zSpeedPassThrough, zDestinationEC, firstLoopPassThrough); 
}

//---------------------------------------------------------------------------------------------------------------------void GlobalMove()---
// Global Move of all Axes
void GlobalMove (long xSpeed, long xTab_m1_New, long ySpeed, long yTab_m0_New, long zSpeed, long zTab_m1_New, bool firstLoop){

// Pre-Move Backlash control
  if (xTab_m1_New > xTab_m1){
    if (xBacklashIn == false){
      odrive_serial << "w axis" << 1 << ".controller.config.vel_limit " << backLashSpeed << '\n';
      xPos_m1 = xPos_m1 + xBacklash;
      xCNC_m1 = xPos_m1;
      odrive.SetPosition(1, xCNC_m1);
      while(xPos_m1 - xEnc_m1 < -163.84 || xPos_m1 - xEnc_m1 > 163.84){
        EncoderPosition();
        GCodePause();
      }
      odrive_serial << "w axis" << 1 << ".controller.config.vel_limit " << xSpeed << '\n';
    }
    Serial.println("Sys - xBacklash = true");
    xBacklashIn = true;
    xPos_m1 = xTab_m1_New + xBacklash;
    xTab_m1 = xTab_m1_New;
  }
  if (xTab_m1_New < xTab_m1){
    if (xBacklashIn == true){
      odrive_serial << "w axis" << 1 << ".controller.config.vel_limit " << backLashSpeed << '\n';
      xPos_m1 = xPos_m1 - xBacklash;
      xCNC_m1 = xPos_m1;
      odrive.SetPosition(1, xCNC_m1);
      while(xPos_m1 - xEnc_m1 < -163.84 || xPos_m1 - xEnc_m1 > 163.84){    // Wait for the motor to almost finish moving (within 0.01mm)
        EncoderPosition();
        GCodePause();
      } 
      odrive_serial << "w axis" << 1 << ".controller.config.vel_limit " << xSpeed << '\n';
    }
    Serial.println("Sys - xBacklash = false");
    xBacklashIn = false;
    xPos_m1 = xTab_m1_New;
    xTab_m1 = xTab_m1_New;
  }
  if (yTab_m0_New > yTab_m0){
    if (yBacklashIn == false){
      odrive_serial << "w axis" << 0 << ".controller.config.vel_limit " << backLashSpeed << '\n';
      yPos_m0 = yPos_m0 + yBacklash;
      yCNC_m0 = yPos_m0;
      odrive.SetPosition(0, yCNC_m0);
      while(yPos_m0 - yEnc_m0 < -163.84 || yPos_m0 - yEnc_m0 > 163.84){
        EncoderPosition();
        GCodePause();
      }
      odrive_serial << "w axis" << 0 << ".controller.config.vel_limit " << ySpeed << '\n';
    }
    Serial.println("Sys - yBacklash = true");
    yBacklashIn = true;
    yPos_m0 = yTab_m0_New + yBacklash;
    yTab_m0 = yTab_m0_New;
  }
  if (yTab_m0_New < yTab_m0){
    if (yBacklashIn == true){
      odrive_serial << "w axis" << 0 << ".controller.config.vel_limit " << backLashSpeed << '\n';
      yPos_m0 = yPos_m0 - yBacklash;
      yCNC_m0 = yPos_m0;
      odrive.SetPosition(0, yCNC_m0);
      while(yPos_m0 - yEnc_m0 < -163.84 || yPos_m0 - yEnc_m0 > 163.84){    // Wait for the motor to almost finish moving (within 0.01mm)
        EncoderPosition();
        GCodePause();
      } 
      odrive_serial << "w axis" << 0 << ".controller.config.vel_limit " << ySpeed << '\n';
    }
    Serial.println("Sys - yBacklash = false");
    yBacklashIn = false;
    yPos_m0 = yTab_m0_New;
    yTab_m0 = yTab_m0_New;
  }

// Only X-Axis is moving and less than 120mm/m
  if (xSpeed < 32768 && xSpeed != 0 && ySpeed == 0 && zSpeed == 0){   // Positive direction Fine control of speed if below 2mm/s (slower than the ODrive min speed)
    Serial.print("Sys - GM - Only X moving < 120mm/m, xEnc_m1 = ");
    Serial.print(xEnc_m1);
    Serial.print(", xPos_m1 = ");
    Serial.println(xPos_m1);
    if (xfirstLoop == true){
      odrive_serial << "w axis" << 1 << ".controller.config.vel_limit " << 32768 << '\n';
    }
    globalMoveLoopTime = millis();
    while(xPos_m1 - xEnc_m1 < -1638.4 || xPos_m1 - xEnc_m1 > 1638.4){
      EncoderPosition();
      GCodePause();
      if (xBacklashIn == true){
        if(xStepIncrement * 2 > xPos_m1 - xEnc_m1){ // alter multiplier to adjust for end accuracy
          xCNC_m1 = xPos_m1;
        }else{
          timeElapsed = millis() - globalMoveLoopTime;
          globalMoveLoopTime = millis();
          xStepIncrement = xSpeed * timeElapsed / 1000;
          xCNC_m1 = xCNC_m1 + xStepIncrement;
        }  
      }else{
         if(xStepIncrement * -2 < xPos_m1 - xEnc_m1){ // alter multiplier to adjust for end accuracy
           xCNC_m1 = xPos_m1;
        }else{
          timeElapsed = millis() - globalMoveLoopTime;
          globalMoveLoopTime = millis();
          xStepIncrement = xSpeed * timeElapsed / 1000;
          xCNC_m1 = xCNC_m1 - xStepIncrement;
        }
      }
      odrive.SetPosition(1, xCNC_m1);
    }
  }

// Only Y-Axis is moving and less than 120mm/m
  if (xSpeed == 0 && ySpeed < 32768 && ySpeed != 0 && zSpeed == 0){   // Positive direction Fine control of speed if below 2mm/s (slower than the ODrive min speed)
    Serial.print("Sys - GM - Only Y moving < 120mm/m, yEnc_m0 = ");
    Serial.print(yEnc_m0);
    Serial.print(", yPos_m0 = ");
    Serial.println(yPos_m0);
    if (yfirstLoop == true){
      odrive_serial << "w axis" << 0 << ".controller.config.vel_limit " << 32768 << '\n';
    }
    globalMoveLoopTime = millis();
    while(yPos_m0 - yEnc_m0 < -1638.4 || yPos_m0 - yEnc_m0 > 1638.4){
      //Serial.print("yEnc_m0 = ");
      //Serial.print(yEnc_m0);
      EncoderPosition();
      GCodePause();
      if (yBacklashIn == true){
        if(yStepIncrement * 2 > yPos_m0 - yEnc_m0){ // alter multiplier to adjust for end accuracy
          yCNC_m0 = yPos_m0;
        }else{
          timeElapsed = millis() - globalMoveLoopTime;
          globalMoveLoopTime = millis();
          yStepIncrement = ySpeed * timeElapsed / 1000;
          yCNC_m0 = yCNC_m0 + yStepIncrement;
        }  
      }else{
         if(yStepIncrement * -2 < yPos_m0 - yEnc_m0){ // alter multiplier to adjust for end accuracy
           yCNC_m0 = yPos_m0;
        }else{
          timeElapsed = millis() - globalMoveLoopTime;
          globalMoveLoopTime = millis();
          yStepIncrement = ySpeed * timeElapsed / 1000;
          yCNC_m0 = yCNC_m0 - yStepIncrement;
        }   
      }
      odrive.SetPosition(0, yCNC_m0);
    }
  }

// Only X-Axis is moving and greater than or equal to 120mm/m
  if (xSpeed > 32767 && ySpeed == 0 && zSpeed == 0){
    Serial.print("Sys - GM - Only X moving >= 120mm/m, xEnc_m1 = ");
    Serial.print(xEnc_m1);
    Serial.print(", xPos_m1 = ");
    Serial.println(xPos_m1);
    if (xfirstLoop == true){
      odrive_serial << "w axis" << 1 << ".controller.config.vel_limit " << xSpeed << '\n';
    }
    globalMoveLoopTime = millis();
      xCNC_m1 = xPos_m1;
      odrive.SetPosition(1, xCNC_m1);
    while(xPos_m1 - xEnc_m1 < -1638.4 || xPos_m1 - xEnc_m1 > 1638.4){
      //Serial.print(", xEnc_m1 = ");
      //Serial.println(xEnc_m1);
      EncoderPosition();
      GCodePause();
    } 
  }

// Only Y-Axis is moving and greater than or equal to 120mm/m
  if (xSpeed == 0 && ySpeed > 32767 && zSpeed == 0){
    Serial.print("Sys - GM - Only Y moving >= 120mm/m, yEnc_m0 = ");
    Serial.print(yEnc_m0);
    Serial.print(", yPos_m0 = ");
    Serial.println(yPos_m0);
    if (yfirstLoop == true){
      odrive_serial << "w axis" << 0 << ".controller.config.vel_limit " << ySpeed << '\n';
    }
    globalMoveLoopTime = millis();
      yCNC_m0 = yPos_m0;
      odrive.SetPosition(0, yCNC_m0);
    while(yPos_m0 - yEnc_m0 < -1638.4 || yPos_m0 - yEnc_m0 > 1638.4){
      //Serial.print("yEnc_m0 = ");
      //Serial.print(yEnc_m0);
      EncoderPosition();
      GCodePause();
    } 
  }

// X-Axis and Y-Axis moving and both are greater than or equal to 120mm/m, can be different speeds
  if (xSpeed > 32767 && ySpeed > 32767 && zSpeed == 0){
      Serial.print("Sys - GM - X & Y moving >= 120mm/m, xEnc_m1 = ");
      Serial.print(xEnc_m1);
      Serial.print(", xPos_m1 = ");
      Serial.println(xPos_m1);
      Serial.print(", yEnc_m0 = ");
      Serial.print(yEnc_m0);
      Serial.print(", yPos_m0 = ");
      Serial.println(yPos_m0);
    if (firstLoop == true){
      odrive_serial << "w axis" << 1 << ".controller.config.vel_limit " << xSpeed << '\n';
      odrive_serial << "w axis" << 0 << ".controller.config.vel_limit " << ySpeed << '\n';
    }
    globalMoveLoopTime = millis();
      xCNC_m1 = xPos_m1;
      yCNC_m0 = yPos_m0;
      odrive.SetPosition(1, xCNC_m1);
      odrive.SetPosition(0, yCNC_m0);
    while(xPos_m1 - xEnc_m1 < -1638.4 || xPos_m1 - xEnc_m1 > 1638.4 || yPos_m0 - yEnc_m0 < -1638.4 || yPos_m0 - yEnc_m0 > 1638.4){
      //Serial.print("yEnc_m0 = ");
      //Serial.print(yEnc_m0);
      //Serial.print(", xEnc_m1 = ");
      //Serial.println(xEnc_m1);
      EncoderPosition();
      GCodePause();
    }
  }
}

void KeyHeldWait(void){ //--------------------------------------------------------------------------------------------void KeyHeldWait()---
  while(KeyPadRetrieve() < 12){
    EncoderPosition(); // Gets ODrive encoder positions (X,Y,Z and Spindle speed) and updates display
    delay(10);
  }
}

// TRACK KEYPAD PRESSES
int KeyPadRetrieve(void){ //---------------------------------------------------------------------------------------void KeyPadRetrieve()---
  int Y1;
  int Y2;
  int Y3;
  int Y4;
  keyVal = 12;
  digitalWrite(keyPadX1, LOW);
  digitalWrite(keyPadX2, HIGH);
  digitalWrite(keyPadX3, HIGH);
  Y1 = digitalRead(keyPadY1);
  Y2 = digitalRead(keyPadY2);
  Y3 = digitalRead(keyPadY3);
  Y4 = digitalRead(keyPadY4);
  if (Y1 == LOW){
    keyVal = 1;
    return keyVal;
  }else{
    if (Y2 == LOW){
      keyVal = 4;
      return keyVal;
    }else{
      if (Y3 == LOW){
        keyVal = 7;
        return keyVal;
      }else{
        if (Y4 == LOW){
          keyVal = 10;
          return keyVal;
        }
      }
    }
  }
  digitalWrite(keyPadX1, HIGH);
  digitalWrite(keyPadX2, LOW);
  digitalWrite(keyPadX3, HIGH);
  Y1 = digitalRead(keyPadY1);
  Y2 = digitalRead(keyPadY2);
  Y3 = digitalRead(keyPadY3);
  Y4 = digitalRead(keyPadY4);
  if (Y1 == LOW){
    keyVal = 2;
    return keyVal;
  }else{
    if (Y2 == LOW){
      keyVal = 5;
      return keyVal;
    }else{
      if (Y3 == LOW){
        keyVal = 8;
        return keyVal;
      }else{
        if (Y4 == LOW){
          keyVal = 0;
          return keyVal;
        }
      }
    }
  }
  digitalWrite(keyPadX1, HIGH);
  digitalWrite(keyPadX2, HIGH);
  digitalWrite(keyPadX3, LOW);
  Y1 = digitalRead(keyPadY1);
  Y2 = digitalRead(keyPadY2);
  Y3 = digitalRead(keyPadY3);
  Y4 = digitalRead(keyPadY4);
  if (Y1 == LOW){
    keyVal = 3;
    return keyVal;
  }else{
    if (Y2 == LOW){
      keyVal = 6;
      return keyVal;
    }else{
      if (Y3 == LOW){
        keyVal = 9;
        return keyVal;
      }else{
        if (Y4 == LOW){
          keyVal = 11;
          return keyVal;
        }
      }
    }
  }
  return keyVal;
}

void KeypadEnterValue (){ //-------------------------------------------------------------------------------------void KeypadEnterValue()---
  String keyPadString = " ";
  byte dec_count = 0;
  byte sign_count = 0;
  while(KeyPadRetrieve() < 12){
    EncoderPosition();
  }
  while(KeyPadRetrieve() != 11){ 
    EncoderPosition();
    switch (KeyPadRetrieve()){
      case 0:
        keyPadString += (char)48;
        KeyHeldWait();
      break;
      case 1:
        keyPadString += (char)49;
        KeyHeldWait();
      break;
      case 2:
        keyPadString += (char)50;
        KeyHeldWait();
      break;
      case 3:
        keyPadString += (char)51;
        KeyHeldWait();
      break;
      case 4:
        keyPadString += (char)52;
        KeyHeldWait();
      break;
      case 5:
        keyPadString += (char)53;
        KeyHeldWait();
      break;
      case 6:
        keyPadString += (char)54;
        KeyHeldWait();
      break;
      case 7:
        keyPadString += (char)55;
        KeyHeldWait();
      break;
      case 8:
        keyPadString += (char)56;
        KeyHeldWait();
      break;
      case 9:
        keyPadString += (char)57;
        KeyHeldWait();
      break;
      case 10:
        if(dec_count < 1){
          keyPadString += (char)46;
          dec_count++;
        }
        KeyHeldWait();
      break; // Delete value
      case -3:
        keyPadString = " ";
        dec_count = 0;
        sign_count = 0;
        KeyHeldWait();
      break;
      case -4: // - Character
        if(sign_count < 1){
          keyPadString += (char)45;
          sign_count++;
        }
        KeyHeldWait();
      break;
      case -10: // + Character
        if(sign_count < 1){
          keyPadString += (char)43;
          sign_count++;
        }
        KeyHeldWait();
      break;
      default:
      break;
    }
    keypadEnteredValue = keyPadString.toFloat();
  }
  KeyHeldWait();
}

void EnteredValueToDisplay(int hor, int ver){ //------------------------------------------------------------void EnteredValueToDisplay()---
  GD.cmd_text(hor,ver, 16, OPT_CENTERY, "Enter Value : ");
  GD.cmd_number(hor+150,ver, 16, OPT_CENTERY, keypadEnteredValue);
}

void FullCalibration(int odrivenum, int motornum){ //----------------------------------------------------------------------------void FullCalibration()---
  Serial.println("FullCalibration");
  unsigned int calibTime = millis();
  int requested_state;
  
  cncStatVal = 1;
  switch(motornum){
    case 0:
      if(odrivenum == 0){
        axisYstate = 3;
        oDvStatVal = 3;
      }else{
        axisSstate = 5;
        oDvStatVal = 5;
      }
      break;
    case 1:
      if(odrivenum == 0){
        axisXstate = 2;
        oDvStatVal = 2;
      }else{
        axisZstate = 4;
        oDvStatVal = 4;
      }
      break;
  }
  requested_state = ODriveArduino::AXIS_STATE_FULL_CALIBRATION_SEQUENCE;
  if(odrivenum == 1){
    odrive1.run_state(motornum, requested_state, false);
  }else{
    odrive.run_state(motornum, requested_state, false);
  }
  pauseLEDMode = 2;
  while(calibTime + 14000 > millis()){
    EncoderPosition();
  }
  
  requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
  if(odrivenum == 1){
    odrive1.run_state(motornum, requested_state, false);
  }else{
    odrive.run_state(motornum, requested_state, false);
  }
  
  pauseLEDMode = 0;
  switch (motornum){
    case 0:
      if(odrivenum == 0){
        odrive.SetPosition(0, yPos_m0);
        EncoderPosition();
        oDrive0m0CLC = true;
      }else{
        odrive1.SetVelocity(0, 0);
        EncoderPosition();
        oDrive1m0CLC = true;
        startLEDMode = 4;
        pauseLEDMode = 0;
        stopLEDMode = 0;
        spindleMode = 3;
      }
      break;
    case 1:
      if(odrivenum == 0){
        xTab_m1 = 0;
        odrive.SetPosition(1, xPos_m1);
        EncoderPosition();
        oDrive0m1CLC = true;
      }else{
        zTab_m1 = 0;
        odrive1.SetPosition(1, zPos_m1);
        EncoderPosition();
        oDrive1m1CLC = true;
      }
      break;
    default:
      break;
  }
}

void SimultaneousCalibration(void){ //---------------------------------------------------------------------void SimultaneousCalibration()---
  Serial.println("SimultaneousCalibration");
  unsigned int calibTime = millis();
  int requested_state;
  
  cncStatVal = 1;
  oDvStatVal = 6;
  axisXstate = 6;
  axisYstate = 6;
  axisZstate = 6;
  axisSstate = 6;
  startLEDMode = 0;
  pauseLEDMode = 0;
  stopLEDMode = 0;
  
  requested_state = ODriveArduino::AXIS_STATE_FULL_CALIBRATION_SEQUENCE;
  odrive1.run_state(0, requested_state, false);
  odrive.run_state(0, requested_state, false);
  //odrive1.run_state(1, requested_state, false); // Not yet installed
  odrive.run_state(1, requested_state, false);

  pauseLEDMode = 6;
  
  while(calibTime + 14000 > millis()){
    EncoderPosition();
  }
  
  requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
  odrive1.run_state(0, requested_state, false);
  odrive.run_state(0, requested_state, false);
  //odrive1.run_state(1, requested_state, false); // Not yet installed
  odrive.run_state(1, requested_state, false);

  pauseLEDMode = 0;
  startLEDMode = 4;

  yTab_m0 = 0;
  odrive.SetPosition(0, yPos_m0);
  EncoderPosition();
  oDrive0m0CLC = true;

  odrive1.SetVelocity(0, 0);
  EncoderPosition();
  oDrive1m0CLC = true;
  spindleMode = 3;

  xTab_m1 = 0;
  odrive.SetPosition(1, xPos_m1);
  EncoderPosition();
  oDrive0m1CLC = true;

  //zTab_m1 = 0;
  //odrive1.SetPosition(1, zPos_m1);
  //EncoderPosition();
  oDrive1m1CLC = true;
  oDvStatVal = 7;
}

void EncoderPosition(void){ //------------------------------------------------------------------------------------void EncoderPosition()---
    odrive_serial << "r axis" << 1 << ".encoder.pos_estimate " << '\n';
    xEnc_m1 = odrive.readFloat();
    if (xBacklashIn == true){
      xDisplayAbs = xEnc_m1 - xBacklash;
      xDisplay = xDisplayAbs + xDisplayOffset;
      x_mmDisplayAbs = (double)xDisplayAbs / 16384;
      x_mmDisplay = (double)xDisplay / 16384;
    }else{
      xDisplayAbs = xEnc_m1;
      xDisplay = xDisplayAbs + xDisplayOffset;
      x_mmDisplayAbs = (double)xDisplayAbs / 16384;
      x_mmDisplay = (double)xDisplay / 16384;
    }
    odrive_serial << "r axis" << 0 << ".encoder.pos_estimate " << '\n';
    yEnc_m0 = odrive.readFloat();
    if (yBacklashIn == true){
      yDisplayAbs = yEnc_m0 - yBacklash;
      yDisplay = yDisplayAbs + yDisplayOffset;
      y_mmDisplayAbs = (double)yDisplayAbs / 16384;
      y_mmDisplay = (double)yDisplay / 16384;
    }else{
      yDisplayAbs = yEnc_m0;
      yDisplay = yDisplayAbs + yDisplayOffset;
      y_mmDisplayAbs = (double)yDisplayAbs / 16384;
      y_mmDisplay = (double)yDisplay / 16384;
    }
    odrive_serial3 << "r axis" << 0 << ".encoder.pos_estimate " << '\n';
    sEnc_m0 = odrive1.readFloat();
    sTimeNow = millis();
    if (sEnc_m0 != sEnc_m0_New){
      sEnc_m0_New = sEnc_m0;
      sEncCount = (sEnc_m0_New - sEnc_m0_Old) * 60000;
      sEnc_m0_Old = sEnc_m0_New;
      sTimeElapsed = sTimeNow - sTime;
      spindleRPM = (sEncCount/sTimeElapsed)/spindleCPR;
    }else{
      spindleRPM = 0;
    }
    sTime = sTimeNow;
    spinMotBmpRot = spinMotBmpRot + spindleRPM / 10;
    if(spinMotBmpRot > 2100000000){
      spinMotBmpRot = 0;
    }
    loopTime = micros() - loopTime; // loopTime now holds time taken to move through the loop once
    if(drawMainScreenDelay + screenRefreshDelay < millis()){ // Only draws the main screen every 50 milliseconds (20Hz refresh) to speed up Axes update and movement control
      DrawMainScreen(); // Draws all elements on the main scren with updated values
      drawMainScreenDelay = millis();
    }
    if(loopTimeDispDelay + loopTimeDisplayRefresh < millis()){ // Measures how fast each loop is and updates the value every 0.2 Seconds
      loopTimeDisplay = loopTime; // Put loopTime into loopTimeDisplay read to be written to the display within the Teensy Status window 
      loopTimeDispDelay = millis();
    }
    loopTime = micros(); // Resets Loop Time counter to now!
}

void AllStop(void){ //----------------------------------------------------------------------------------------------------void AllStop()---
  int requested_state;
  oDrive0m0CLC = false;
  oDrive0m1CLC = false;
  requested_state = ODriveArduino::AXIS_STATE_IDLE;
  odrive.run_state(0, requested_state, false);
  odrive.run_state(1, requested_state, false);
  odrive1.run_state(0, requested_state, false);
  odrive1.run_state(1, requested_state, false);
  cncStatVal = 0;
  axisXstate = 1;
  axisYstate = 1;
  axisZstate = 1;
  axisSstate = 1;
  oDvStatVal = 1;
}

void GCodeLoadtoRAM (){
  GD.ClearColorRGB(0x000020);
  GD.Clear();
  if(SD.begin(chipSelect)){ // Start the SD Card and display on screen if Ok
    GD.cmd_text(10,10, 16, OPT_CENTERY, "SD Card Started ok");
    sDcardInserted = true;
  }else{
    GD.cmd_text(10,10, 16, OPT_CENTERY, "SD Card Not Found");
    sDcardInserted = false;
  }
  GD.swap(); // Draw the Display List to the display
  delay(1500);
  int fileNumber = 0;
  String fileNumString = "";
  gcodeToRAMDisplay = true;
  while(gcodeToRAMDisplay == true){
    horPos = 10;
    verPos = 100;
    GD.ClearColorRGB(0x000020);
    GD.Clear();
    GD.VertexFormat(0); // points are scaled to screen pixels
    DevRotate(665,10);
    rootDir = SD.open("/");
    fileCount = 0;
    displayContents(rootDir, 0);
    fileCount = 0;
    rootDir = SD.open("/");
    fileCopied = false;
    if(KeyPadRetrieve() < 12){
      if(KeyPadRetrieve() == 11){
        gcodeToRAMDisplay = false;
        break;
      }else{
        fileNumString += KeyPadRetrieve();
      }
      while(KeyPadRetrieve() < 12){
      }
      fileNumber = fileNumString.toInt();
    }
    GD.ColorRGB(0xffffff);
    GD.cmd_text(340,30, 18, OPT_CENTER, "SD Card Contents");
    GD.cmd_text(340,50, 18, OPT_CENTER, "Copy GCode File to Teensy 4.1 RAM");
    GD.cmd_text(340,70, 18, OPT_CENTER, "Maximum File Size = 300000 Bytes (300kB)");
    GD.ColorRGB(0xaaffff);
    GD.cmd_text(480,530, 16, OPT_RIGHTX, "Enter GCode File Number to load to RAM:");
    GD.cmd_number(484,534, 16, OPT_CENTERY, fileNumber);
    GD.swap();
  }
  ChooseFile(rootDir, fileNumber, 0);
  while(KeyPadRetrieve() < 12){
  }
  GD.ClearColorRGB(0x000020);
  GD.Clear();
  GD.swap();
}

void displayContents(File dir, int indent){
  horPos = 170;
  while(verPos < 450){
    File temp = dir.openNextFile();
    if(!temp){
      break;
    }
    fileCount++;
    for(int i = 0; i < indent; i++){
      if (temp.isDirectory()) {
        if(indent == 0){
          horPos = 160;
        }else{
          horPos = (indent * 20) + 170;
        }
      }
    }
    if(temp.isDirectory()){
      GD.ColorRGB(0xbbbbff);
      GD.cmd_text(horPos,verPos, 16, OPT_CENTERY, temp.name());
      verPos = verPos + 10;
      fileCount--;
      displayContents(temp, indent + 1);
    }else{
      if(indent == 0){
        horPos = 170;
      }else{
        horPos = (indent * 20) + 170;
      }
      GD.ColorRGB(0xaaffff);
      GD.cmd_number(150,verPos-4, 16, OPT_RIGHTX, fileCount);
      GD.cmd_text(158,verPos-4, 16, OPT_RIGHTX, ":");
      GD.ColorRGB(0xffffff);
      GD.cmd_text(horPos,verPos, 16, OPT_CENTERY, temp.name());
      GD.ColorRGB(0xffffcc);
      GD.cmd_number(470,verPos-4, 16, OPT_RIGHTX, temp.size());
      GD.cmd_text(480,verPos, 16, OPT_CENTERY, "Bytes");
      verPos = verPos + 10;
      horPos = 170;
    }
    temp.close();
  }
}

void ChooseFile(File dir, int selectFile, int indent){
  memoryFile = "";
  selectFile--;
  while(fileCopied == false){
    gcFile = dir.openNextFile();
    if(!gcFile){
      break;
    }
    if(fileCount == selectFile && fileCopied == false){ // open this file and copy to memory
      memoryFileLength = memoryFile.length();
      Serial.print("memoryFileLength Start = ");
      Serial.print(memoryFileLength);
      unsigned long tempFileSize = gcFile.size();
      if(tempFileSize > maxFileLength){
        Serial.println("memoryFileLenth too big");
      }else{
        while(memoryFileLength < tempFileSize){
          gcASCIIval = gcFile.read();
          memoryFile += char(gcASCIIval);
          memoryFileLength = memoryFile.length();
        }
        Serial.print("  memoryFileLength Copied = ");
        Serial.println(memoryFileLength);
        gcLiveCharPos = 0;
        gcCurrLineNo = 0;
        gcCurrLSP = 0;
        readCount = 0;
      }
      fileCopied = true;
      gcodeToRAMDisplay = false;
      break;
    }
    fileCount++;
    if(gcFile.isDirectory()){
      fileCount--;
      selectFile++;
      ChooseFile(gcFile, selectFile, indent + 1);
      selectFile--;
    }
    gcFile.close();
  }
}

void ManageEEPROM (void){
  SettingsObject loadVar;
  EEPROM.get(0, loadVar);
  xBacklash = loadVar.xBacklash_Set;
  yBacklash = loadVar.yBacklash_Set;
  backLashSpeed = loadVar.backlashSpeed_Set;
  //spindleRPM_Max = loadVar.spindleRPM_Max_Set;
  jogSpeed = loadVar.jogSpeed_Max_Set;
  xBacklash_mm = (xBacklash * 1000) / 16384;
  yBacklash_mm = (yBacklash * 1000) / 16384;
  Serial.println("-----EEPROM CURRENT VALUES-----");
  Serial.print("xBacklash = ");
  Serial.println(xBacklash);
  Serial.print("yBacklash = ");
  Serial.println(yBacklash);
  Serial.print("backLashSpeed = ");
  Serial.println(backLashSpeed);
  Serial.print("spindleRPM_Max = ");
  Serial.println(spindleRPM_Max);
  Serial.print("jogSpeed = ");
  Serial.println(jogSpeed);
  Serial.println();
  if (backlashAdjust == true){
    Serial.println("Backlash Adjust");
    xBacklash = 1; // Set to 1 to prevent divide by zero errors
    yBacklash = 1; // Set to 1 to prevent divide by zero errors
    cncModeVal = 4;
    axisXstate = 9;
    axisYstate = 9;
    JogMode();
    if (xBacklash != 1){
      updateEEPROM = true;
    }else{
      xBacklash = loadVar.xBacklash_Set;
    }
    if (yBacklash != 1){
      updateEEPROM = true;
    }else{
      yBacklash = loadVar.yBacklash_Set;
    }
    backlashAdjust = false;
  }
  if (updateEEPROM == true){
    backLashSpeed = 200000;
    //spindleRPM_Max = 4500;
    jogSpeed = 300000;
    SettingsObject updateVar = {xBacklash, yBacklash, backLashSpeed, spindleRPM_Max, jogSpeed};
    EEPROM.put(0, updateVar);
    SettingsObject loadVar;
    EEPROM.get(0, loadVar);
    xBacklash = loadVar.xBacklash_Set;
    yBacklash = loadVar.yBacklash_Set;
    backLashSpeed = loadVar.backlashSpeed_Set;
    spindleRPM_Max = loadVar.spindleRPM_Max_Set;
    jogSpeed = loadVar.jogSpeed_Max_Set;
    Serial.println("-----EEPROM UPDATED VALUES-----");
    Serial.print("xBacklash = ");
    Serial.println(xBacklash);
    Serial.print("yBacklash = ");
    Serial.println(yBacklash);
    Serial.print("backLashSpeed = ");
    Serial.println(backLashSpeed);
    Serial.print("spindleRPM_Max = ");
    Serial.println(spindleRPM_Max);
    Serial.print("jogSpeed = ");
    Serial.println(jogSpeed);
    Serial.println();
  }
}

/*-------------------------------------------------------------INDEX OF FUNCTIONS----------------------------------------------------------
 * 
 * Line:  317 - void setup()
 * Line:  438 - void loop()
 * Line:  620 - void DrawMainScreen()
 * Line:  651 - void Menu_MainDisplay()
 * Line:  777 - void Menu_Machine()
 * Line:  819 - void Menu_Work()
 * Line:  861 - void CircleToDisplay()
 * Line:  932 - void GCodeToDisplay()
 * Line:  989 - void AxesPanel()
 * Line: 1075 - void AxesDisplay()
 * Line: 1095 - void AxesDisplaySmall()
 * Line: 1115 - void AxesDial()
 * Line: 1134 - void AxesPanelMove()
 * Line: 1145 - void SpindleRotate()
 * Line: 1167 - void DevRotate()
 * Line: 1185 - void Dev255Rotate()
 * Line: 1202 - void TimeWindow()
 * Line: 1238 - void ClockDisplayMove()
 * Line: 1270 - void CreateBevel()
 * Line: 1279 - void LEDflash()
 * Line: 1322 - void GCodeDraw()
 * Line: 1336 - void G00()
 * Line: 1341 - void G01()
 * Line: 1347 - void G02()
 * Line: 1401 - void G03()
 * Line: 1456 - void ArcMillG02()
 * Line: 1507 - void ArcMillG03()
 * Line: 1558 - void CircleMill()
 * Line: 1681 - void LineMill()
 * Line: 1812 - void GlobalMovemm()
 * Line: 1827 - void GlobalMove()
 * Line: 1996 - void KeyHeldWait()
 * Line: 2003 - int KeyPadRetrieve()
 * Line: 2092 - void KeypadEnterValue()
 * Line: 2177 - void EnteredValueToDisplay()
 * Line: 2182 - void FullCalibration()
 * Line: 2233 - void EncoderPosition()
 * Line: 2260 - void AllStop()
 * Line: 2280 - void TempAxesVals()
 *
 */

//----------Things to address-----------

// Set Maximm speed constant to cap Global move
// Provide Jog Mode Instructions while in Jog Mode
// Take picture of and display JoyPad function in menus and Jog mode, could use graphics text rather than GD.text to save display overflow
// Look at velocity control either here or on ODrive for CNC accuracy
// GCode test menu with GCode test (runs GCode above the work)
// Bite size chunks
// Joystick controlled menu?
// Use physical microswitch endstops connected to ODrive for safety with ODrive system and connect to Arduino for program control also
// Force spindle motor to idle when doing a tool change
// Menu adjustable Jog speed?
// HDMI Output for menu system
// HDMI GCode to graphics (can this be a sprite?)
// Automatic Backlash measurement at startup? - Save results to log file for trend analysis
// Auto Tool Height Setting
// Tool Change dwell introduction

//-----------------------------------------------Notes-----------------------------------------------
// Low speed limit in Counts Per Second = 32768 (Minimum speed in counts per second for ODrive stability at 12v)
// High speed limit in Counts Per Second = 360284 (Maximum speed in counts per second for ODrive stability at 12v)
