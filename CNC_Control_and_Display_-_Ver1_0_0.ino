/*
Neil Devonshire - Dev255
YouTube Dev255
www.dev255.uk

This program is an on-going project and is still being worked on (updated 06/10/2021) and has not been optimized to run at maximum efficiency.

G-Code to CNC sketch to machine features from a main menue system
comments added to help describe the funcion of each statement (if requied)

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
 * # Move all displays slowly to reduce screen burn
 * # Change colour of Axes dial background to show Axis status
 * # Change colour of Axes button font to show Zeroed = Green, Machine Coordinates = Blue
 * # Fit battery to Teensy 4.1 and created a menu to change the time and date
 * # Use the RTC to determine Run Time and On Time
 * # Show Run Time on main display
 * # Show On Time on main display
 * # Show Estimated Time on main display, although all GCode will need to be evaluated
 * # Prevent GCode reaching the end of the GCode file and exitin the main void loop
 * # If not too time consuming, create Line drawing from the GCode file to show what is being milled
 * # Work on Emergency Stop Mode
 * # Provide a pause function
 * # Setup Warning Pop up if not calibrated
 * # Setup Warning Pop up if not Zeroed
 * # Setup Warning Pop up if tool changed and requires tool legnth setup
 * 
 * ## NOTES ON SKETCH ##
 * # Sketch has been/being written to read a GCode File creted in Fusion 360
 */

// Library header files to write to Teensy/Arduino
#include <DS1307RTC.h> // Real Time Clock (RTC) on Teensy 4.1 Library
#include <SPI.h> // Standard Serial Periferal Interface Library (Built in)
#include <GD2.h> // Gameduino 3x Dazzler Library (https://excamera.com/sphinx/gameduino2/code.html#with-the-arduino-ide)
#include <SD.h> // Standard SD Card Library (built in)
#include <ODriveArduino.h> // ODrive Motor Controller Library (https://docs.odriverobotics.com/ascii-protocol)
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_LEDBackpack.h>
#include <Math.h>

// SD Card variables
const int chipSelect = BUILTIN_SDCARD; // Chip select value for the Teensy 4.1 SD Card
bool sDcardInserted = false; // NOT USED YET ######

// Clock variables
int yr = 2021; // Temporary Year for the Teensy 4.1 Real Time Clock (RTC)
int mon = 8; // Temporary Month for the Teensy 4.1 (RTC)
int dy = 14; // Temporary Day for the Teensy 4.1 (RTC)
int hr = 8; // Temporary Hour for the Teensy 4.1 (RTC)
int mnt = 59; // Temporary Minute for the Teensy 4.1 (RTC)
int sec = 0; // Temporary Second for the Teensy 4.1 (RTC)
int rot = 0; // Rotate value for the Branding Bitmap (Dev255)

// Keypad Pin setup and variables
const int keyPadY1 = 24; // Keypad matrix Y1 connected to Teensy 4.1 pin 24
const int keyPadY2 = 25; // Keypad matrix Y2 connected to Teensy 4.1 pin 25
const int keyPadY3 = 26; // Keypad matrix Y3 connected to Teensy 4.1 pin 26
const int keyPadY4 = 27; // Keypad matrix Y4 connected to Teensy 4.1 pin 27
const int keyPadX1 = 28; // Keypad matrix X1 connected to Teensy 4.1 pin 28
const int keyPadX2 = 29; // Keypad matrix X2 connected to Teensy 4.1 pin 29                                                                                                                                                                                                                                                                     
const int keyPadX3 = 30; // Keypad matrix X3 connected to Teensy 4.1 pin 30
int keyVal = 12; // Current value of the key pressed, [0] to [9] for numbers, 10 for [*], 11 for [#] and 12 for 'no key pressed'
double keypadEnteredValue = 0.0d; // This is the user entered number temporary storage aera, including decimal point.

//Menu variables
int menuSelect = 0; // Current menu to display on main screen, 0 = Main window with GCode, 1 = Machine Menu, 2 = Work Menu
int menu00Pos = 20; // Starting position of Menu 0, 20 is on screen and drawn
int menu01Pos = 760; // Starting position of Menu 1, > 720 is off screen and not drawn
int menu02Pos = 740; // Starting position of Menu 2, > 720 is off screen and not drawn
bool menu000 = true; // Control logic for Menu 0 (main window)
bool menu001 = false; // Control logic for Menu 1
bool menu002 = false; // Control logic for Menu 2

// Window positions
int axDispHor = 0; // Horizontal position of the Axes display panel - linked to and moves with X-Axis 
int axDispVer = 0; // Vertical position of the Axes display panel - linked to and moves with Y-Axis
double clDispHor = 45.0d; // Clock window Horizontal position - moves around to reduce screen burn
double clDispVer = 1235.0d; // Clock window Vertical position - moves around to reduce screen burn
bool clDispVerDown = true; // Used to control Vertical direction of Clock window
bool clDispHorRight = true; // Used to control Horizontal direction of Clock window

// Timing variables
int loopTimeDisplay = 0; // Stores time taken to run through 1 Main Void loop in microseconds
int loopTime = millis(); // Used to calculate loopTimeDisplay above
int loopTimeFunction = millis(); // Used to calculate the time taken to press and hold keypad keys (for fast scroll functions)

// Strings for Status Displays and their contolling integers
char *cncModeStrings[] = {"G-CODE MANUAL Z","MANUAL CIRCLE","MANUAL LINE","MANUAL RECTANGLE","AXIS JOG"};
char *cncStatStrings[] = {"Calibration Required","Calibrating","Idle","Running G-Code","Paused","Tool Change","EMERGENCY STOP"}; // If cncStatStrings [2], then "Idle" is displayed on screen
char *dazStatStrings[] = {"720 x 1280 @ 60Hz","Error"};                                                                          // If dazStatStrings [0], then "720 x 1280 @ 60Hz" is displayed on screen
char *tnyStatStrings[] = {"Loop microsec =","SD Card Full","Holding ODrive Reset"};                                              // If tnyStatStrings [1], then "SD Card Full" is displayed on screen
char *oDvStatStrings[] = {"All axis Idle","Part axis Idle","Calibrating X Axis","Calibrating Y Axis","Calibrating Z Axis","Calibrating Spindle","Closed Loop Control","Error"};                         // If oDvStatStrings [3], then "Closed Loop Control" is displayed on screen
char *nextToolStrings[] = {"None","Pen","40mm Shell","10mm End","8mm End"};
int cncModeVal = 0; // Used in conjunction with cncModeSting above to control the string that is displayed next to the Program Status Window
int cncStatVal = 0; // Used in conjunction with cncStatSting above and cncStatLED below to control the string that is displayed and LED colour, i.e. cncStatStrings[cncStatVal]
int dazStatVal = 0; // Used in conjunction with dazStatSting above and dazStatLED below to control the string that is displayed and LED colour, i.e. dazStatStrings[dazStatVal]
int tnyStatVal = 0; // Used in conjunction with tnyStatSting above and tnyStatLED below to control the string that is displayed and LED colour, i.e. tnyStatStrings[tnyStatVal]
int oDvStatVal = 0; // Used in conjunction with oDvStatSting above and oDvStatLED below to control the string that is displayed and LED colour, i.e. oDvStatStrings[oDvStatVal]
int xStatVal = 0; // Used in conjunction with xyzAxesCharLED below to control the LED colour, i.e. xyzAxesCharLED[xStatVal]
int yStatVal = 0; // Used in conjunction with xyzAxesCharLED below to control the LED colour, i.e. xyzAxesCharLED[yStatVal]
int zStatVal = 0; // Used in conjunction with xyzAxesCharLED below to control the LED colour, i.e. xyzAxesCharLED[zStatVal]
int axisXstate = 0; // Used in conjunction with oDvStatSting above and oDvStatLED below to control the string that is displayed and LED colour, i.e. oDvStatStrings[axisXstate]
int axisYstate = 0; // Used in conjunction with oDvStatSting above and oDvStatLED below to control the string that is displayed and LED colour, i.e. oDvStatStrings[axisYstate]
int axisZstate = 0; // Used in conjunction with oDvStatSting above and oDvStatLED below to control the string that is displayed and LED colour, i.e. oDvStatStrings[axisZstate]

// LED Colours and Control
char *cncStatLED[] = {0xff8000,0xffff00,0x008000,0x00ff00,0x00ff00,0xff8000,0xff0000}; // LED colour for CNC status
char *dazStatLED[] = {0x00ff00,0xff0000}; // LED colour for Gameduino 3x Dazzler status
char *tnyStatLED[] = {0x00ff00,0xff0000,0xff0000}; // LED colour for Teensy 4.1 status
char *oDvStatLED[] = {0xff8000,0xffff00,0xffff00,0xffff00,0xffff00,0xffff00,0x00ff00,0xff0000}; // LED colour for ODrive motor contoller status
char *xyzAxesCharLED[] = {0x00ffff,0xff00ff,0xff0000}; // LED colour for X,Y & Z button status
int cncLEDSize = 75; // Size of LED controlled to show (75) or not show (0) LED colour to mimic flashing
int dazLEDSize = 75; // Size of LED controlled to show (75) or not show (0) LED colour to mimic flashing
int tnyLEDSize = 75; // Size of LED controlled to show (75) or not show (0) LED colour to mimic flashing
int oDvLEDSize = 75; // Size of LED controlled to show (75) or not show (0) LED colour to mimic flashing
int xDialLEDSize = 528; // Size of LED controlled to show (528) or not show (0) LED colour to mimic flashing
int yDialLEDSize = 528; // Size of LED controlled to show (528) or not show (0) LED colour to mimic flashing
int zDialLEDSize = 528; // Size of LED controlled to show (528) or not show (0) LED colour to mimic flashing
int loopLEDflash = 0; // Used to count milliseconds to flash LED once per second
bool ledOn = false; // Logic true if LED on

// GCode Variables and boolean flags
int gcTextVer = 0; // GCode Vertical start position, value changes with main window as it moves around 
int gcTextHor = 0; // GCode Horizontal start position, value changes with main window as it moves around
int gcTextLine = 0; // Used to control maximum number of lines to display on screen to prevent 'display list overflow' errors on Dazzler as only 2048 instructions per frame allowed
int gcCharCount = 0; // Used to control maximum number of characters to display on screen to prevent 'display list overflow' errors on Dazzler as only 2048 instructions per frame allowed
int gcCurrLineNo = 0; // Current GCode Line number chosen, stepped by user to look at each line. This will be used to step through GCode on CNC when sketches merged
int gcLiveCharPos = 0; // Used to track GCode characters. This will be used to determine GCode Mode (G00,G01,G02,etc, X,Y,Z values and all other control for the CNC machine to use once sketches merged
int gcCurrLSP = 0; // Current Line Starting Point (LSP) that writes to the display from this line forward
int gcASCIIval = 0; // Current ASCII value read from the GCode file before being processed by this sketch
int gCodeMode = 0; // Reflects the current GCode for GCode Run mode, so G00 = 0, G01 = 1, etc
int spindleSpeed = 0; // in RPM
bool nextTool = true;
int nextToolval = 0;
bool spindleOn = true;
int readCount = 0;
double preX = 0.0d;
double preY = 0.0d;
double preZ = 0.0d;
double x = 0.0d;
double y = 0.0d;
double z = 0.0d;
double a = 0.0d;
double b = 0.0d;
double c = 0.0d;
double h = 0.0d;
double i = 0.0d;
double j = 0.0d;
double k = 0.0d;
double r = 0.0d;
double f = 0.0d;
int p = 0;
double q = 0.0d;
double s = 0.0d;
double v = 0.0d;
bool absArcCenMode = false;
bool absPosMode = false;
int planeSelect = 17;

// Axis and Spindle
double ph1 = 0.0d; // Tempoaray value incremented to provide cosine value of X Axis display. This will be linked to the CNC X-Axis once sketches merged
double ph2 = 0.0d; // Tempoaray value incremented to provide sine value of Y Axis display. This will be linked to the CNC Y-Axis once sketches merged
double ph3 = 0.0d; // Tempoaray value incremented to provide sine value of Z Axis display. This will be linked to the CNC Z-Axis once sketches merged
double ph4 = 0.0d; // Tempoaray value incremented to provide cosine value of Spindle Speed display. This will be linked to the CNC spindle motor speed once sketches merged
double spindleRPM = 0.0d; // Spindle RPM value. This will be linked to actual spindle speed once CNC sketch merged
double spinMotBmpRot = 0.0d; // Motor Bitmap rotation value, value changes angle of Bitmap to show spindle motor spinning at different RPM's

int loopTimeDispDelay = millis();
int drawMainScreenDelay = millis();

// VARIOUS VARIABLES FOR EACH AXIS

// X-Axis variables
long xPos_m1 = 0;                    // Commanded position of X-Axis Motor in encoder counts
long xEnc_m1 = 0;                    // Current Estimated encoder counts of X-Axis rotary encoder
long xTab_m1 = 0;                    // Current table position in encoder counts
long xTab_m1_New = 0;                // New table position before X-Axis move in encoder counts
long xDisplayOffset = 0;             // Difference between acual Table position (xTab_m1) and displayed position for X-Axis in encoder counts
long xDisplay = 0;                   // Displayed position of X-Axis in encoder counts, offset from xTab_m1 when X axis zeroed
long xDisplayAbs = 0;                // Displayed Absolute (machine) position of X-Axis in encoder counts, never offset from xTab_m1
double x_mmDisplay = 0.0d;              // Displayed position of X-Axis in mm, offset from xTab_m1 when X axis zeroed
double x_mmDisplayAbs = 0.0d;           // Displayed Absolute (machine) position of X-Axis in mm, never offset from xTab_m1

// Y-Axis variables
long yPos_m0 = 0;                    // Commanded position of Y-Axis Motor in encoder counts
long yEnc_m0 = 0;                    // Current Estimated encoder counts of Y-Axis rotary encoder
long yTab_m0 = 0;                    // Current table position in encoder counts
long yTab_m0_New = 0;                // New table position before Y-Axis move in encoder counts
long yDisplayOffset = 0;             // Difference between actual Table position (yTab_m0) and displayed position for Y-Axis in encoder counts
long yDisplay = 0;                   // Displayed position of Y-Axis in encoder counts, offset from yTab_m0 when Y axis zeroed
long yDisplayAbs = 0;                // Displayed Absolute (machine) position of Y-Axis in encoder counts, never offset from yTab_m0
double y_mmDisplay = 0.0d;              // Displayed position of Y-Axis in mm, offset from yTab_m0 when Y axis zeroed
double y_mmDisplayAbs = 0.0d;           // Displayed Absolute (machine) position of Y-Axis in mm, never offset from yTab_m0

// Z-Axis variables
long zPos_m0 = 0;                    // Commanded position of Z-Axis Motor in encoder counts
long zEnc_m0 = 0;                    // Current Estimated encoder counts of Z-Axis rotary encoder
long zTab_m0 = 0;                    // Current head position in encoder counts
long zTab_m0_New = 0;                // New head position before Z-Axis move in encoder counts
long zDisplayOffset = 0;             // Difference between actual head position (zTab_m0) and displayed position for Z-Axis in encoder counts
long zDisplay = 0;                   // Displayed position of Z-Axis in encoder counts, offset from zTab_m0 when Z axis zeroed
long zDisplayAbs = 0;                // Displayed Absolute (machine) position of Z-Axis in encoder counts, never offset from zTab_m0
double z_mmDisplay = 0.0d;              // Displayed position of z-Axis in mm, offset from zTab_m0 when Z axis zeroed
double z_mmDisplayAbs = 0.0d;           // Displayed Absolute (machine) position of Z-Axis in mm, never offset from zTab_m0

// Backlash variables
const long backLashSpeed = 250000;      // Constant - Speed for taking up backlash (X & Y axes affected at present), no backlash control on Z axis

// X-Axis Backlash variables
const long xBacklash = 6269;          // Number of encoder counts to remove backlash, tune on table with .pos_gain from ODriveTools.
bool xBacklashIn = false;             // Set to true when X-Axis moving in plus (count increases) direction.

// Y-Axis Backlash variables
const long yBacklash = 2694;          // Number of encoder counts to remove backlash, tune on table with .pos_gain from ODriveTools.
bool yBacklashIn = false;             // Set to true when Y-Axis moving in plus (count increases) direction.

// Program Status Flags
volatile bool oDrive0m0CLC = false; // Is ODrive 0, motor 0 (Y-Axis) in Closed Loop Control? Volatile because used in Emergency Stop ISR (interrupt)
volatile bool oDrive0m1CLC = false; // Is ODrive 0, motor 1 (X-Axis) in Closed Loop Control? Volatile because used in Emergency Stop ISR (interrupt)
volatile bool eStopPressed = false; // Set to true only when the Emergency Stop has been pressed, resets to false once callibration is carried out.

unsigned long dev255LoopTime1 = 0;
unsigned long dev255LoopTime2 = 0;
byte dev255LoopCount = 2;
double feedSpeed = 0.0d;

//Global Move Function values, moved here to speed up global move
unsigned long globalMoveLoopTime = 0;
unsigned long timeElapsed = 0;
long xStepIncrement = 0;
long xCNC_m1 = 0;
long yStepIncrement = 0;
long yCNC_m0 = 0;

// Circle function values and flags
bool internalCircle = true;
double circleDiameter = 0.0d;
double circleDepth = 0.0d;
double toolDiameter = 0.0d;
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

double mmJogIncr = 0.1d;
long jogIncr = 1638;
long yJogPos_m0 = 0;
long xJogPos_m1 = 0;
double xCirPos_mm = 0.0d;
double yCirPos_mm = 0.0d;
long jogSpeed = 150000;
bool xfirstLoop = true;
bool yfirstLoop = true;
long xyzSpeed = 0;
int count = 1;
bool keypadValRequired = false;
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
bool gCodePause = false;
int gCodePauseVal = 0;
bool machinePause = false;
int gcLinesToDisplay = 19;
String gcRunningString = "";
char gcRunningCharArray[50];
bool cncEnd = false;
String memoryFile = ""; // Huge String to contain the GCode file to speed up writing to display

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

// Printing with stream operator - ODrive requirement for ASCII Protocol
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, double arg) { obj.print(arg, 4); return obj; }

ODriveArduino odrive(odrive_serial); // ODrive object

File gcFile; // Current GCode file object - linked later to "TempFile.TXT"
File settingsFile; // Current GCode file object - linked later to "Settings.TXT"

void setup(){ //------------------------------------------------------------------------------------------------------------void setup()---
  
  matrix.begin(0x70);  // Pass in the Adafruit 8x8 Matrix Address
  matrix.setBrightness(0);

  odrive_serial.begin(115200); // Serial Port 1 to ODrive = 115200 baud
  
  Serial.begin(115200); // Serial speed to PC from Teensy 4.1
  Serial.println("Serial Started @ 115200 bps"); // Written to confirm Serial is working

  // Gameuino Reset - Pin 16 linked to Gameduino 3x Dazzler RESET pin to reset the Dazzler after powering on  
  pinMode(16, OUTPUT);
  digitalWrite(16, LOW);
  delay(500);
  digitalWrite(16, HIGH);
  delay(1000);

  // Set ODrive limits
  odrive_serial << "w axis" << 0 << ".motor.config.current_lim " << 40.0f << '\n';  // Y-Axis Motor
  odrive_serial << "w axis" << 0 << ".controller.config.vel_limit " << 36028.0f << '\n';  // Y-Axis Motor
  odrive_serial << "w axis" << 1 << ".motor.config.current_lim " << 40.0f << '\n';  // X-Axis Motor
  odrive_serial << "w axis" << 1 << ".controller.config.vel_limit " << 36028.0f << '\n';  // X-Axis Motor

  // Dispaly Dev255 logo on Adafruit Matrix
  /*
  matrix.setRotation(0);
  for (int8_t x=2; x>=-35; x--) {
    matrix.clear();
    matrix.setCursor(x,0);
    matrix.print("Dev");
    matrix.print("255");
    matrix.writeDisplay();
    delay(100);
  }
  */
  matrix.clear();
  matrix.setRotation(0);
  matrix.drawBitmap(0, 0, dev255, 8, 8, LED_ON);
  matrix.writeDisplay();

  dev255LoopTime1 = millis();
  dev255LoopTime2 = millis();
  
  GD.begin(); // Start Dazzler
  GD.cmd_setrotate(2); // Rotate display to Portrait
  GD.ClearColorRGB(0x000000); // Colour to Clear all pixels to
  GD.Clear(); // Wipe the display with the Clear colour
  GD.VertexFormat(0); // points are scaled to screen pixels

  if(SD.begin(chipSelect)){ // Start the SD Card and display on screen if Ok
    GD.cmd_text(10,10, 16, OPT_CENTERY, "SD Card Started ok");
    sDcardInserted = true;
  }else{
    GD.cmd_text(10,10, 16, OPT_CENTERY, "SD Card Not Found");
    sDcardInserted = false;
  }
  if(SD.exists("tempfile.txt")){
    gcFile = SD.open("tempFile.txt"); // Open the GCode file, this is a *.TAP or *.nc or *.gcode file with only the extension changed to *.TXT, no other changes done
    GD.cmd_text(10,20, 16, OPT_CENTERY, "tempfile.txt exists");
    Serial.println("tempfile.txt exists");
    int memoryFileLength = memoryFile.length();
    int tempFileSize = gcFile.size();
    GD.cmd_text(10,30, 16, OPT_CENTERY, "tempfile.txt size = ");
    Serial.print("tempfile.txt size = ");
    GD.cmd_number(200,30, 16, OPT_CENTERY, tempFileSize);
    Serial.println(tempFileSize);
    GD.cmd_text(10,40, 16, OPT_CENTERY, "memoryFile length = ");
    Serial.print("memoryFile length = ");
    GD.cmd_number(200,40, 16, OPT_CENTERY, memoryFileLength);
    Serial.println(memoryFileLength);
    while(memoryFileLength < tempFileSize){
      gcASCIIval = gcFile.read();
      memoryFile += char(gcASCIIval);
      memoryFileLength = memoryFile.length();
      Serial.println(memoryFileLength);
    }
    GD.cmd_text(10,50, 16, OPT_CENTERY, "New memoryFile length = ");
    Serial.print("New memoryFile length = ");
    GD.cmd_number(200,50, 16, OPT_CENTERY, memoryFileLength);
    Serial.println(memoryFileLength);
  }else{
    GD.cmd_text(10,20, 16, OPT_CENTERY, "tempfile.txt does not exist");
  }
  
  GD.swap(); // Draw the Display List to the display
  gcFile.close();
delay (1000);
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
  GD.load("AluFrm2.png"); // Brusshed Aluminium Frame Image - Load to Dazzler memory - Ceter removed and corners rounded on Windows Paint 3D (with transparrent background to allow silloette to function on Axes display panel)

  GD.BitmapHandle(7);
  GD.cmd_loadimage(-7, 0);
  GD.load("BallScrw.png"); // Ball Screw X Image - Load to Dazzler memory

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

  setTime(hr,mnt,sec,dy,mon,yr); // Temporarily set as there is no battery currently connected to the Teensy 4.1, this will be a Menu function once a battery is fitted 

  // Set the KeyPad Y matrix pins to Input (with pull up resistors enabled) and X matrix pins to Output mode
  pinMode(keyPadY1, INPUT);
  pinMode(keyPadY1, INPUT_PULLUP);
  pinMode(keyPadY2, INPUT);
  pinMode(keyPadY2, INPUT_PULLUP);
  pinMode(keyPadY3, INPUT);
  pinMode(keyPadY3, INPUT_PULLUP);
  pinMode(keyPadY4, INPUT);
  pinMode(keyPadY4, INPUT_PULLUP);
  pinMode(keyPadX1, OUTPUT);
  pinMode(keyPadX2, OUTPUT);
  pinMode(keyPadX3, OUTPUT); 
}

void loop(){ //--------------------------------------------------------------------------------------------------------------void loop()---
  
  //gcFile = SD.open("tempFile.txt"); // Open the GCode file, this is a *.TAP or *.nc file with only the extension changed to *.TXT, no other changes done
  if(cncModeVal == 0){
    Dev255Rotate();
  }
  EncoderPosition(); // Gets ODrive encoder positions (X,Y,Z and Spindle speed) and updates display

  // Status LED and descriptions updated here
  if(oDrive0m0CLC == true && oDrive0m1CLC == true){
    cncStatVal = 2;
    oDvStatVal = 6;
    axisXstate = 6;
    axisYstate = 6;
    axisZstate = 6;
  }else{
    if(oDrive0m0CLC == false && oDrive0m1CLC == true){
      cncStatVal = 0;
      oDvStatVal = 1;
      axisXstate = 6;
      axisYstate = 0;
      axisZstate = 6;    
    }else{
      if(oDrive0m0CLC == true && oDrive0m1CLC == false){
        cncStatVal = 0;
        oDvStatVal = 1;
        axisXstate = 0;
        axisYstate = 6;
        axisZstate = 6;
      }else{
        cncStatVal = 0;
        oDvStatVal = 0;
        axisXstate = 0;
        axisYstate = 0;
        axisZstate = 6;
      }
    }
  }
   
  if(eStopPressed == true){
    menuSelect = 0;
    matrix.clear();
    matrix.setRotation(2);
    matrix.drawBitmap(0, 0, emergencyStop, 8, 8, LED_ON);
    matrix.writeDisplay();
  }
  
  // Pressing keypad 8 increases gcCurrLineNo by 1, forcing the GCode line to step forwards and read from the next return value (ASCII 13)
  // Holding keypad no 8 for 0.5 second steps gcCurrLineNo once per screen refresh (fast scrolling) until there are no more characters to read
  if(KeyPadRetrieve() == 8 && cncModeVal == 0){
    loopTimeFunction = millis();
    gcCurrLineNo++;
    while(KeyPadRetrieve() < 12){
      EncoderPosition(); // Gets ODrive encoder positions (X,Y,Z and Spindle speed) and updates display      if(millis() > loopTimeFunction + 1000){
      if(millis() > loopTimeFunction + 500){
        while(KeyPadRetrieve() == 8){
          if(gcCurrLSP < gcCurrLineNo){
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
  if  (KeyPadRetrieve() == 2 && cncModeVal == 0){
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

  // Calibrate ODrive Motor 1
  if (KeyPadRetrieve() == 0 && menu000 == true && cncModeVal == 0){
    FullCalibration(0);
    KeyHeldWait();
  }
  // Calibrate ODrive Motor 1
  if (KeyPadRetrieve() == 1 && menu000 == true && cncModeVal == 0){
    FullCalibration(1);
    KeyHeldWait();
  }

  // Calibrate ODrive Motor 1
  if (KeyPadRetrieve() == 9 && menu000 == true && cncModeVal == 0){
    AllStop();
    KeyHeldWait();
  }

  // Determines which menu to display on screen from the menu selected with [#] or [*] keys
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

  if(KeyPadRetrieve() == 2 && menu002 == true){
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
  if (KeyPadRetrieve() == 9 && menu002 == true){
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
  
  if (KeyPadRetrieve() == 5 && menu000 == true){
    GCodeRun();
  }
}

void DrawMainScreen(void){ //--------------------------------------------------------------------------------------void DrawMainScreen()---
  GD.ClearColorRGB(0x000020);
  GD.Clear();
  GD.VertexFormat(0); // points are scaled to screen pixels
  AxesPanelMove(); // float value to move the Axes Display in Horizontal and Vertical plane, 0 = No Move.
  AxesPanel(axDispHor,axDispVer); // horizontal min = 10, max = 320, vertical min = 10, max = 100 ---- set axDispHor 154, axDispVer 70 for center
  DevRotate(665,10);
  ClockDisplayMove (0.0166); // float value to move the Clock Display in Horizontal and Vertical plane, 0 = No Move.
  TimeWindow(clDispHor,clDispVer); // horizontal min = 13, max = 580, vertical min = 1150, max = 1230 ---- set axDispHor 576, axDispVer 1226 for center
  
  if(menuSelect == 0 || menu00Pos >-700){
    Menu_MainDisplay(menu00Pos,400);
    if (menuSelect == 0 && menu00Pos == 20 && cncModeVal == 0){
      GCodeToDisplay(menu00Pos+132,622);
    }
    if (menuSelect == 0 && menu00Pos == 20 && cncModeVal == 1){
      CircleToDisplay(menu00Pos+132,622);
      if(keypadValRequired == true){
        EnteredValueToDisplay(150,150);       
      }
    }
  }
  if(menuSelect == 1 || menu01Pos <720 && menu01Pos >-700){
    Menu_Machine(menu01Pos,400);
  }
  if(menuSelect == 2 || menu02Pos <740 && menu02Pos >-700){
    Menu_Work(menu02Pos,400);
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
  if (oDvStatVal == 0 || oDvStatVal == 1 || oDvStatVal == 2 || oDvStatVal == 3 || oDvStatVal == 4 || oDvStatVal == 5 || oDvStatVal == 7){
    if(ledOn == false){
      oDvLEDSize = 0;
    }else{
      oDvLEDSize = 75;
    }
  }
  if (oDvStatVal == 6){
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
  GD.ColorRGB(0x606080);
  GD.cmd_text(hor+10,ver+50,28,OPT_CENTERY,"1. Calibrate ODrive X & Y");
  GD.cmd_text(hor+10,ver+70,28,OPT_CENTERY,"2. Setup Soft End Stops");
  GD.cmd_text(hor+10,ver+90,28,OPT_CENTERY,"3. Set Max Spidle Speed");
  GD.cmd_text(hor+10,ver+110,28,OPT_CENTERY,"4. Measure X, Y backlash");
  GD.cmd_text(hor+260,ver+50,28,OPT_CENTERY,"5. Option 5");
  GD.cmd_text(hor+260,ver+70,28,OPT_CENTERY,"6. Option 6");
  GD.cmd_text(hor+260,ver+90,28,OPT_CENTERY,"7. Option 7");
  GD.cmd_text(hor+260,ver+110,28,OPT_CENTERY,"8. Option 8");
  GD.cmd_text(hor+480,ver+50,28,OPT_CENTERY,"9. Option 9");
  GD.cmd_text(hor+480,ver+70,28,OPT_CENTERY,"10. Option 10");
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
  GD.ColorRGB(0x606080);
  GD.cmd_text(hor+10,ver+50,28,OPT_CENTERY,"1. Zero Axes X, Y & Z");
  GD.cmd_text(hor+10,ver+70,28,OPT_CENTERY,"2. Cut Basic Circle");
  GD.cmd_text(hor+10,ver+90,28,OPT_CENTERY,"3. Cut Basic Rectangle");
  GD.cmd_text(hor+10,ver+110,28,OPT_CENTERY,"4. Cut Basic Line Shape");
  GD.cmd_text(hor+260,ver+50,28,OPT_CENTERY,"5. Tool Change");
  GD.cmd_text(hor+260,ver+70,28,OPT_CENTERY,"6. Hole Drilling");
  GD.cmd_text(hor+260,ver+90,28,OPT_CENTERY,"7. Hole Tapping");
  GD.cmd_text(hor+260,ver+110,28,OPT_CENTERY,"8. Dry Run");
  GD.cmd_text(hor+480,ver+50,28,OPT_CENTERY,"9. Jog Mode");
  GD.cmd_text(hor+480,ver+70,28,OPT_CENTERY,"10. Tool Height Measure");
}

//WRITE THE CIRCLE PROGRAM ON TO PROGRAM STATUS DISPLAY
void CircleToDisplay(int hor, int ver){ //------------------------------------------------------------------------void CircleToDisplay()---
  GD.ColorRGB(0xffffff);
  GD.cmd_text(hor+210,ver, 16, OPT_CENTERY, "MILL A CIRCLE");
  GD.cmd_text(hor,ver+20, 16, OPT_CENTERY, "Setup circle parameters");
  GD.cmd_text(hor+20,ver+40, 16, OPT_CENTERY, "1. Diameter mm = ");
  GD.cmd_number(hor+240,ver+40, 16, OPT_CENTERY, circleDiameter);  
  GD.cmd_text(hor+20,ver+50, 16, OPT_CENTERY, "2. Internal/External = ");
  if(internalCircle == true){
    GD.cmd_text(hor+240,ver+50, 16, OPT_CENTERY, "Internal");
  }else{
    GD.cmd_text(hor+240,ver+50, 16, OPT_CENTERY, "External");
  }
  GD.cmd_text(hor+20,ver+60, 16, OPT_CENTERY, "3. Depth mm = ");
  GD.cmd_number(hor+240,ver+60, 16, OPT_CENTERY, circleDepth);
  GD.cmd_text(hor+20,ver+70, 16, OPT_CENTERY, "4. Center X mm = ");
  GD.cmd_number(hor+240,ver+70, 16, OPT_CENTERY, circleXcoord);
  GD.cmd_text(hor+20,ver+80, 16, OPT_CENTERY, "5. Center Y mm = ");
  GD.cmd_number(hor+240,ver+80, 16, OPT_CENTERY, circleYcoord);
  GD.cmd_text(hor+20,ver+90, 16, OPT_CENTERY, "6. Mill Direction = ");
  if(climbMill == true){
    GD.cmd_text(hor+240,ver+90, 16, OPT_CENTERY, "Climb");
  }else{
    GD.cmd_text(hor+240,ver+90, 16, OPT_CENTERY, "Conventional");
  }
  GD.cmd_text(hor+20,ver+100, 16, OPT_CENTERY, "7. Tool Diameter mm = ");
  GD.cmd_number(hor+240,ver+100, 16, OPT_CENTERY, toolDiameter);
  GD.cmd_text(hor+20,ver+110, 16, OPT_CENTERY, "8. Feed Speed mm/min = ");
  GD.cmd_number(hor+240,ver+110, 16, OPT_CENTERY, feedSpeed);
  GD.cmd_text(hor+20,ver+120, 16, OPT_CENTERY, "9. Use Existing Values");
  if(keypadValRequired == true){
    EnteredValueToDisplay(hor,ver+140);
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
    GD.cmd_text(hor,ver+130, 16, OPT_CENTERY, "! PREVIOUS VALUES - CHANGE IF REQUIRED OR [9] TO ACCEPT");    
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

if(cncEnd == true){
  gcLiveCharPos = 0;
  gcCurrLineNo = 0;
  gcCurrLSP = 0;
  readCount = 0;
}
          
  if(displayZAxisSet == true){
    GD.cmd_text(hor+30,ver-120,16, OPT_CENTERY, "SET Z AXIS TO: ");
    GD.cmd_number(hor+155,ver-124,16, OPT_RIGHTX | OPT_SIGNED, int(z));
    GD.cmd_text(hor+157,ver-120,16,OPT_CENTERY,".");
    GD.cmd_number(hor+167,ver-124,16,4,int(10000 * abs(z)));
    GD.cmd_text(hor+205,ver-120, 16, OPT_CENTERY, "THEN PRESS [#] TO RESUME");
  }
  if(cncStatVal == 3){
    gcRunningString.toCharArray(gcRunningCharArray,50);
    //Serial.print("String = ");
    //Serial.print(gcRunningString);
    //Serial.print("  Char Array = ");
    //Serial.println(gcRunningCharArray);
    GD.cmd_text(hor,ver-21, 16, OPT_CENTERY, gcRunningCharArray);
    GD.cmd_text(hor,ver-90, 18, OPT_CENTERY, "TO PAUSE, HOLD [0]");
  }
  if(cncStatVal == 4){
    GD.cmd_text(hor,ver-90, 18, OPT_CENTERY, "MACHINE PAUSED, PRESS [1] TO RESUME, OR [*] TO QUIT");
  }
   
  gcTextHor = hor;
  gcTextVer = ver;
  gcCharCount = 0;
  gcTextLine = 0;
  GD.BitmapSource(0x2000e8);// Memory location 0x2000e8 aligns to the ASCII Table in memory for use in TXT8X8, only when buttons are drawn on the screen though
  GD.BitmapSize(NEAREST, BORDER, BORDER, 8, 8);
  GD.BitmapLayout(TEXT8X8,1,1);
  GD.Begin(BITMAPS);

  //gcFile.seek(gcLiveCharPos);
  gcASCIIval = memoryFile.charAt(gcLiveCharPos);
  //gcLiveCharPos ++;
  if(gcCurrLineNo != gcCurrLSP){ // Move displayed GCode by one line if gcCurrentLineNo has been changed by user or GCode Run
    if(gcCurrLineNo < gcCurrLSP){ // Move displayed GCode back by one line
      gcLiveCharPos = gcLiveCharPos - 2; // Go back 2 characters past the return (ASCII 13)
      do{
        //gcFile.seek(gcLiveCharPos);
        //gcASCIIval = gcFile.read();
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
        //gcASCIIval = gcFile.read();
        gcASCIIval = memoryFile.charAt(gcLiveCharPos);
        gcLiveCharPos ++;
        //gcLiveCharPos++;
      }while(gcASCIIval != 13);
      gcCurrLSP = gcCurrLineNo;
    }
  }
  
  //gcFile.seek(gcLiveCharPos);
  gcStartCharPos = gcLiveCharPos;
  while(gcCharCount < 200 && gcTextLine < gcLinesToDisplay){ // Maximum 350 amount of characters to prevent "display list error" and maximum lines, whichever comes first
    //gcASCIIval = gcFile.read();
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
  for(int currentChar = 0; featureLength > currentChar; currentChar++){
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
  GD.cmd_rotate(DEGREES(spinPos/33.33));
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
void CreateBevel(int tlPosX, int tlPosY, int brPosX, int brPosY, char Str1[], char Str2[]){ //------------------------void CreateBevel()---
  GD.ColorRGB(Str1);
  GD.Vertex2f((brPosX-5),(tlPosY+2));
  GD.Vertex2f(brPosX,(brPosY-2));
  GD.ColorRGB(Str2);
  GD.Vertex2f((tlPosX+2),(tlPosY+2));
  GD.Vertex2f((brPosX-2),(brPosY-2));
}

void LEDflash(int hor, int ver){ //--------------------------------------------------------------------------------------void LEDflash()---
  GD.Begin(POINTS);
  if(loopLEDflash + 500 < millis()){
    loopLEDflash = millis();
    if(ledOn == true){
      ledOn = false;
    }else{
      ledOn = true;
    }
  }
  if (axisXstate == 0 || axisXstate == 1 || axisXstate == 2 || axisXstate == 3 || axisXstate == 4 || axisXstate == 5 || axisXstate == 7){
    if(ledOn == false){
      xDialLEDSize = 0;
    }else{
      xDialLEDSize = 528;
    }
  }
  if (axisXstate == 6){
    xDialLEDSize = 528;
  }

  if (axisYstate == 0 || axisYstate == 1 || axisYstate == 2 || axisYstate == 3 || axisYstate == 4 || axisYstate == 5 || axisYstate == 7){
    if(ledOn == false){
      yDialLEDSize = 0;
    }else{
      yDialLEDSize = 528;
    }
  }
  if (axisYstate == 6){
    yDialLEDSize = 528;
  }
  if (axisZstate == 0 || axisZstate == 1 || axisZstate == 2 || axisZstate == 3 || axisZstate == 4 || axisZstate == 5 || axisZstate == 7){
    if(ledOn == false){
      zDialLEDSize = 0;
    }else{
      zDialLEDSize = 528;
    }
  }
  if (axisZstate == 6){
    zDialLEDSize = 528;
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
  xyzSelect = 0;
  bool case0FirstTime = true;
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
    switch (xyzSelect){
      default:
        EncoderPosition();
      break;
      case 0:                        // X,Y & Z-Axes Selected
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
  readCount = gcLiveCharPos;
  gCodeMode = 0;
  cncEnd = false;
  
  gcRunCharPos = gcLiveCharPos;
  //gcFile.seek(gcRunCharPos);
  gcASCIIval = memoryFile.charAt(gcRunCharPos);
  cncStatVal = 3;
  gcRunningString = "";
  while(KeyPadRetrieve() !=10 && cncEnd == false){
    if(KeyPadRetrieve() == 0){ // PAUSE GCODE
      cncStatVal = 4;
      gcLinesToDisplay = 19;
      while(KeyPadRetrieve() != 1){
        EncoderPosition();
        if(KeyPadRetrieve() == 10){// Exit G-Code run
          KeyHeldWait();
          cncEnd = true;
          gCodeMode = 100; // To stop any execution of GCode
          break;
        }
      }
      cncStatVal = 3;
      KeyHeldWait();
    }
    gcLinesToDisplay = 19;
    //gcFile.seek(readCount);
    //gcASCIIval = gcFile.read();
    gcASCIIval = memoryFile.charAt(readCount);
    readCount++;
   //Serial.print("First ASCII Value = ");
   //Serial.println(gcASCIIval);
    if(cncEnd == true){
      gcASCIIval = 100; // To prevent the following switch carrying our GCode operations
    }
    switch(gcASCIIval){
      case 10: // 'LF' line feed, used with 'CR'(13) carriage return (both denote a new line similar to '/n')
       //Serial.println("¬");
      break;
      case 13: // 'CR' carriage return, used with 'LF' above
       //Serial.print("<");
        gcCurrLineNo++;
        EncoderPosition();
        preX = x;
        preY = y;
        preZ = z;
      break;
      case 32: // 'space' used to seperate commands and values
       //Serial.print("_");
      break;
      case 37: // '%' used at the end of the full GCode program
       //Serial.println(char(gcASCIIval));
        cncEnd = true;
      break;
      case 40: // '(' used at the beginning of a GCode section description, put the contents of braces into CurrentFeature string for display
       //Serial.print(char(gcASCIIval));
        combineChars = "";
        do{
          //gcASCIIval = gcFile.read();
          gcASCIIval = memoryFile.charAt(readCount);
          readCount++;
         //Serial.print(char(gcASCIIval));
          combineChars += (char)gcASCIIval;
          subReadCount++;
        }while(gcASCIIval != 41);
        combineChars.remove(subReadCount-1,1);
        currentFeature = combineChars;
        subReadCount = 0;
      break;
      case 65: // 'A' Denotes 4th Axis movement followed by move end value
       //Serial.print(char(gcASCIIval));
        a = GCodeValueBuild();
       //Serial.print("[");
       //Serial.print(a,4);
       //Serial.print("]");
        //gcASCIIval = gcFile.read();
        gcASCIIval = memoryFile.charAt(readCount);
        readCount++;
        if(gcASCIIval == 13){ // Execute move only if it's the last value on that line
          ExecuteGCodeMove();
        }
        readCount--;
      break;
      case 66: // 'B' Denotes 5th Axis movement followed by move end value
       //Serial.print(char(gcASCIIval));
        b = GCodeValueBuild();
       //Serial.print("[");
       //Serial.print(b,4);
       //Serial.print("]");
        //gcASCIIval = gcFile.read();
        gcASCIIval = memoryFile.charAt(readCount);
        readCount++;
        if(gcASCIIval == 13){ // Execute move only if it's the last value on that line
          ExecuteGCodeMove();
        }
        readCount--;
      break;
      case 67: // 'C' Denotes 6th Axis movement followed by move end value
       //Serial.print(char(gcASCIIval));
        c = GCodeValueBuild();
       //Serial.print("[");
       //Serial.print(c,4);
       //Serial.print("]");
        //gcASCIIval = gcFile.read();
        gcASCIIval = memoryFile.charAt(readCount);
        readCount++;
        if(gcASCIIval == 13){ // Execute move only if it's the last value on that line
          ExecuteGCodeMove();
        }
        readCount--;
      break;
      case 68: // 'D' Tool diameter offset used
       //Serial.print(char(gcASCIIval));
      break;
      case 70: // 'F' Feed rate
       //Serial.print(char(gcASCIIval));
        f = GCodeValueBuild();
       //Serial.print("[");
       //Serial.print(f,4);
       //Serial.print("]");
        //gcASCIIval = gcFile.read();
        gcASCIIval = memoryFile.charAt(readCount);
        readCount++;
        if(gcASCIIval == 13){ // Execute move only if it's the last value on that line
          ExecuteGCodeMove();
        }
        readCount--;
      break;
      case 71: // 'G' Preparatory function, G followed by a numerical code, specifies machining modes and functions
       //Serial.print(char(gcASCIIval));
        //gcASCIIval = gcFile.read();
        gcASCIIval = memoryFile.charAt(readCount);
       //Serial.print(char(gcASCIIval));
        readCount++;
        switch(gcASCIIval){
          case 48:
            //gcASCIIval = gcFile.read();
            gcASCIIval = memoryFile.charAt(readCount);
            if(gcASCIIval != 32){
             //Serial.print(char(gcASCIIval));
            }
            readCount++;
            switch(gcASCIIval){
              case 48:// G00 - Rapid Move
                gCodeMode = 0;
              break;
              case 49:// G01 - Linear Feed Move
                gCodeMode = 1;
              break;
              case 50:// G02 - Clockwise Arc Feed Move
                gCodeMode = 2;
              break;
              case 51:// G03 - Counter Clockwise Arc Feed Move
                gCodeMode = 3;
              break;
              case 52:// G04 - Dwell
                gCodePause = true;
              break;
              case 57:// G09 - Exact stop
              break;
              default:// G0 - Rapid Move G00, most GCode miss the preceding 0
                gCodeMode = 0;
                readCount--;
              break;
            }
          break;
          case 49:
            //gcASCIIval = gcFile.read();
            gcASCIIval = memoryFile.charAt(readCount);
            if(gcASCIIval != 32){
             //Serial.print(char(gcASCIIval));
            }
            readCount++;
            switch(gcASCIIval){
              case 48:// G10 - Fixture and Tool Offset Setting
              break;
              case 50:// G12 - Clockwise Circle
                gCodeMode = 12;
              break;
              case 51:// G13 - Counter Clockwise Circle
                gCodeMode = 13;
              break;
              case 53:// G15 - Polar Coordinate Cancel
              break;
              case 54:// G16 - Polar Coordinate
              break;
              case 55:// G17 - XY Plane Select
                planeSelect = 17;
              break;
              case 56:// G18 - ZX Plane Select
                planeSelect = 18;
              break;
              case 57:// G19 - YZ Plane Select
                planeSelect = 19;
              break;
              default:// G1 - Linear Feed Move G01, most GCode miss the preceding 0
                gCodeMode = 1;
                readCount--;
              break;
            }
          break;
          case 50:
            //gcASCIIval = gcFile.read();
            gcASCIIval = memoryFile.charAt(readCount);
            if(gcASCIIval != 32){
             //Serial.print(char(gcASCIIval));
            }
            readCount++;
            switch(gcASCIIval){
              case 48:// G20 - Inch Unit Measurement
              break;
              case 49:// G21 - Millimeter Unit Measurement
              break;
              case 56:// G28 - Zero Return
                gCodeMode = 28;
              break;
              default:// G2 - Clockwise Arc Feed Move G02, most GCode miss the preceding 0
                gCodeMode = 2;
                readCount--;
              break;
            }
          break;
          case 51:
            //gcASCIIval = gcFile.read();
            gcASCIIval = memoryFile.charAt(readCount);
            if(gcASCIIval != 32){
             //Serial.print(char(gcASCIIval));
            }
            readCount++;
            switch(gcASCIIval){
              case 48:// G30 - 2nd, 3rd, 4th Zero Return
                gCodeMode = 30;
              break;
              case 49:// G31 - Probe Function
                gCodeMode = 31;
              break;
              case 50:// G32 - Threading
                gCodeMode = 32;
              break;
              default:// G3 - Counter Clockwise Arc Feed Move G03, most GCode miss the preceding 0
                gCodeMode = 3;
                readCount--;
              break;
            }
          break;
          case 52:
            //gcASCIIval = gcFile.read();
            gcASCIIval = memoryFile.charAt(readCount);
            if(gcASCIIval != 32){
             //Serial.print(char(gcASCIIval));
            }
            readCount++;
            switch(gcASCIIval){
              case 48:// G40 - Cutter Compensation Cancel
              break;
              case 49:// G41 - Cutter Compensation Left
              break;
              case 50:// G42 - Cutter Compensation Right
              break;
              case 51:// G43 - Tool Length Offset + Enable
              break;
              case 52:// G44 - Tool Length Offset - Enable
              break;
              case 57:// G49 - Tool Length Offset Cancel
              break;
              default:// G4 - Dwell G04, most GCode miss the preceding 0
                gCodePause = true;
              break;
            }
          break;
          case 53:
            //gcASCIIval = gcFile.read();
            gcASCIIval = memoryFile.charAt(readCount);
            if(gcASCIIval != 32){
             //Serial.print(char(gcASCIIval));
            }
            readCount++;
            switch(gcASCIIval){
              case 48:// G50 - Cancel Scaling
              break;
              case 49:// G51 - Scale Axes
              break;
              case 50:// G52 - Local Coordinate System Shift
              break;
              case 51:// G53 - Machine Coordinate System
              break;
              case 52:// G54 - Fixture Offset 1
                //gcASCIIval = gcFile.read();
                gcASCIIval = memoryFile.charAt(readCount);
               //Serial.print(char(gcASCIIval));
                readCount++;
                if(gcASCIIval == 46){
                  //gcASCIIval = gcFile.read();
                  gcASCIIval = memoryFile.charAt(readCount);
                 //Serial.print(char(gcASCIIval));
                  readCount++;
                  switch(gcASCIIval){
                    case 49:// G54.1 - Additional Fixture Offsets
                    break;
                  }
                }else{
                  readCount--;
                }
              break;
              case 53:// G55 - Fixture Offset 2
              break;
              case 54:// G56 - Fixture Offset 3
              break;
              case 55:// G57 - Fixture Offset 4
              break;
              case 56:// G58 - Fixture Offset 5
              break;
              case 57:// G59 - Fixture Offset 6
              break;
            }
          break;
          case 54:
            //gcASCIIval = gcFile.read();
            gcASCIIval = memoryFile.charAt(readCount);
            if(gcASCIIval != 32){
             //Serial.print(char(gcASCIIval));
            }
            readCount++;
            switch(gcASCIIval){
              case 48:// G60 - Unidirectional Approach
              break;
              case 49:// G61 - Exact Stop Mode
              break;
              case 52:// G64 - Cutting Mode (Constant Velocity)
              break;
              case 53:// G65 - Macro Call
              break;
              case 54:// G66 - Macro Modal Call
              break;
              case 55:// G67 - Macro Modal Call Cancel
              break;
              case 56:// G68 - Coordinate System Rotation
              break;
              case 57:// G69 - Coordinate System Rotation Cancel
              break;
            }
          break;
          case 55:
            //gcASCIIval = gcFile.read();
            gcASCIIval = memoryFile.charAt(readCount);
            if(gcASCIIval != 32){
             //Serial.print(char(gcASCIIval));
            }
            readCount++;
            switch(gcASCIIval){
              case 51:// G73 - High Speed Peck Drilling
              break;
              case 52:// G74 - LH Tapping
              break;
              case 54:// G76 - Fine Boring
              break;
            }
          break;
          case 56:
            //gcASCIIval = gcFile.read();
            gcASCIIval = memoryFile.charAt(readCount);
            if(gcASCIIval != 32){
             //Serial.print(char(gcASCIIval));
            }
            readCount++;
            switch(gcASCIIval){
              case 48:// G80 - Canned Cycle Cancel
              break;
              case 49:// G81 - Hole Drilling
              break;
              case 50:// G82 - Spot Face
              break;
              case 51:// G83 - Deep Hole Peck Drilling
              break;
              case 52:// G84 - RH Tapping
                //gcASCIIval = gcFile.read();
                gcASCIIval = memoryFile.charAt(readCount);
               //Serial.print(char(gcASCIIval));
                readCount++;
                if(gcASCIIval == 46){
                  //gcASCIIval = gcFile.read();
                  gcASCIIval = memoryFile.charAt(readCount);
                 //Serial.print(char(gcASCIIval));
                  readCount++;
                  switch(gcASCIIval){
                    case 50:// G84.2 - Additional Fixture Offsets
                    break;
                    case 51:// G84.3 - Additional Fixture Offsets
                    break;
                  }
                }else{
                  readCount--;
                }
              break;
              case 53:// G85 - Boring, Retract at Feed, Spindle On
              break;
              case 54:// G86 - Boring, Retract at Rapid, Spindle Off
              break;
              case 55:// G87 - Back Boring
              break;
              case 56:// G88 - Boring, Manual Retract
              break;
              case 57:// G89 - Boring, Dwell, Retract at Feed, Spindle On
              break;
            }
          break;
          case 57:
            //gcASCIIval = gcFile.read();
            gcASCIIval = memoryFile.charAt(readCount);
            if(gcASCIIval != 32){
             //Serial.print(char(gcASCIIval));
            }
            readCount++;
            switch(gcASCIIval){
              case 48:// G90 - Absolute Position Mode
                //gcASCIIval = gcFile.read();
                gcASCIIval = memoryFile.charAt(readCount);
                readCount++;
                if(gcASCIIval == 46){
                 //Serial.print(char(gcASCIIval));
                  //gcASCIIval = gcFile.read();
                  gcASCIIval = memoryFile.charAt(readCount);
                 //Serial.print(char(gcASCIIval));
                  readCount++;
                  switch(gcASCIIval){
                    case 49:// G90.1 - Arc Center Absolute Mode
                      absArcCenMode = true;
                    break;
                  }
                }else{
                  readCount--;
                  //gcFile.seek(readCount);
                  gcASCIIval = memoryFile.charAt(readCount);
                  absPosMode = true;
                  // Code here to switch to Absolute Position Mode
                }
              break;
              case 49:// G91 - Incremental Position Mode
                //gcASCIIval = gcFile.read();
                gcASCIIval = memoryFile.charAt(readCount);
                readCount++;
                if(gcASCIIval == 46){
                 //Serial.print(char(gcASCIIval));
                  //gcASCIIval = gcFile.read();
                  gcASCIIval = memoryFile.charAt(readCount);
                 //Serial.print(char(gcASCIIval));
                  readCount++;
                  switch(gcASCIIval){
                    case 49:// G91.1 - Arc Center Incremental Mode
                      absArcCenMode = false;
                    break;
                  }
                }else{
                  readCount--;
                  //gcFile.seek(readCount);
                  gcASCIIval = memoryFile.charAt(readCount);
                  absPosMode = false;
                  // Code here to switch to Incremental Position Mode
                }
              break;
              case 50:// G92 - Local Coordinate System Setting
                //gcASCIIval = gcFile.read();
                gcASCIIval = memoryFile.charAt(readCount);
               //Serial.print(char(gcASCIIval));
                readCount++;
                if(gcASCIIval == 46){
                  //gcASCIIval = gcFile.read();
                  gcASCIIval = memoryFile.charAt(readCount);
                 //Serial.print(char(gcASCIIval));
                  readCount++;
                  switch(gcASCIIval){
                    case 49:// G92.1 - Local Coordinate System Cancel
                    break;
                  }
                }else{
                  readCount--;
                }
              break;
              case 51:// G93 - Inverse Time Feed
              break;
              case 52:// G94 - Feed per Minute
              break;
              case 53:// G95 - Feed per Revolution
              break;
              case 54:// G96 - Constant Surface Speed
              break;
              case 55:// G97 - Constant Speed
              break;
              case 56:// G98 - Initial Point Return
              break;
              case 57:// G99 - R Point Return
              break;
              default:// G9 - Exact Stop G09, most GCode miss the preceding 0
              break;
            }
          break;
        }
      break;
      case 72: // 'H' Tool height offset
       //Serial.print(char(gcASCIIval));
        h = GCodeValueBuild();
       //Serial.print("[");
       //Serial.print(h,4);
       //Serial.print("]");
        //gcASCIIval = gcFile.read();
        gcASCIIval = memoryFile.charAt(readCount);
        readCount++;
        if(gcASCIIval == 13){ // Execute move only if it's the last value on that line
          ExecuteGCodeMove();
        }
        readCount--;
      break;
      case 73: // 'I' Used in Arc mode (G02, G03) as center of circle on X axis
       //Serial.print(char(gcASCIIval));
        i = GCodeValueBuild();
       //Serial.print("[");
       //Serial.print(i,4);
       //Serial.print("]");
        //gcASCIIval = gcFile.read();
        gcASCIIval = memoryFile.charAt(readCount);
        readCount++;
        if(gcASCIIval == 13){ // Execute move only if it's the last value on that line
          ExecuteGCodeMove();
        }
        readCount--;
      break;
      case 74: // 'J' Used in Arc mode (G02, G03) as center of circle on Y axis
       //Serial.print(char(gcASCIIval));
        j = GCodeValueBuild();
       //Serial.print("[");
       //Serial.print(j,4);
       //Serial.print("]");
        //gcASCIIval = gcFile.read();
        gcASCIIval = memoryFile.charAt(readCount);
        readCount++;
        if(gcASCIIval == 13){ // Execute move only if it's the last value on that line
          ExecuteGCodeMove();
        }
        readCount--;
      break;
      case 75: // 'K' Used in Arc mode (G02, G03) as center of circle on Z axis
       //Serial.print(char(gcASCIIval));
        k = GCodeValueBuild();
       //Serial.print("[");
       //Serial.print(k,4);
       //Serial.print("]");
        //gcASCIIval = gcFile.read();
        gcASCIIval = memoryFile.charAt(readCount);
        readCount++;
        if(gcASCIIval == 13){ // Execute move only if it's the last value on that line
          ExecuteGCodeMove();
        }
        readCount--;
      break;
      case 77: // 'M' Miscellaneous functions, M followed by a numerical code - Coolant, Tool Change, M30 end of program
       //Serial.print(char(gcASCIIval));
        //gcASCIIval = gcFile.read();
        gcASCIIval = memoryFile.charAt(readCount);
       //Serial.print(char(gcASCIIval));
        readCount++;
        switch(gcASCIIval){
          case 48:
          break;
          case 49:
          break;
          case 50:
          break;
          case 51:
                //gcASCIIval = gcFile.read();
                gcASCIIval = memoryFile.charAt(readCount);
               //Serial.print(char(gcASCIIval));
                readCount++;
                if(gcASCIIval == 48){
                  cncEnd = true;
                  spindleOn = false;
                  gcCurrLSP = 0;
                  gcCurrLineNo = 0;
                  readCount = 0;
                  gcLiveCharPos = readCount;
                }else{
                  readCount--;
                  spindleOn = true;
                }
          break;
          case 52:
          break;
          case 53:
          break;
          case 54:
            spindleOn = false;
            nextTool = false;
          break;
          case 55:
          break;
          case 56:
          break;
          case 57:
          break;
        }
      break;
      case 78: // 'N' Sequence numbers to goto and run if required and used
       //Serial.print(char(gcASCIIval));
      break;
      case 80: // 'P' P followed by a numerical value denotes Dwell (G04) time in seconds
       //Serial.print(char(gcASCIIval));
        p = GCodeValueBuild();
        gCodePauseVal = p * 1000;
      break;
      case 82: // 'R' Radius of a circle if using this feture in G02/G03
       //Serial.print(char(gcASCIIval));
        r = GCodeValueBuild();
       //Serial.print("[");
       //Serial.print(r,4);
       //Serial.print("]");
        //gcASCIIval = gcFile.read();
        gcASCIIval = memoryFile.charAt(readCount);
        readCount++;
        if(gcASCIIval == 13){ // Execute move only if it's the last value on that line
          ExecuteGCodeMove();
        }
        readCount--;
      break;
      case 83: // 'S' Spindle speed followed by numerical value in RPM
       //Serial.print(char(gcASCIIval));
        s = GCodeValueBuild();
       //Serial.print("[");
       //Serial.print(s,4);
       //Serial.print("]");
        //gcASCIIval = gcFile.read();
        gcASCIIval = memoryFile.charAt(readCount);
        readCount++;
        if(gcASCIIval == 13){ // Execute move only if it's the last value on that line
          ExecuteGCodeMove();
        }
        readCount--;
      break;
      case 84: // 'T' Tool call, followed by next tool number, to prepare tool ready for change
       //Serial.print(char(gcASCIIval));
        //gcASCIIval = gcFile.read();
        gcASCIIval = memoryFile.charAt(readCount);
       //Serial.print(char(gcASCIIval));
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
       //Serial.print(char(gcASCIIval));
        v = GCodeValueBuild();
       //Serial.print("[");
       //Serial.print(v,4);
       //Serial.print("]");
        if(absPosMode == false){
          x = x + v;
        }else{
          x = v;
        }
        //gcASCIIval = gcFile.read();
        gcASCIIval = memoryFile.charAt(readCount);
        readCount++;
        if(gcASCIIval == 13){ // Execute move only if it's the last value on that line
          ExecuteGCodeMove();
        }
        readCount--;
      break;
      case 89: // 'Y' Denotes 2nd Axis movement followed by move end value
       //Serial.print(char(gcASCIIval));
        v = GCodeValueBuild();
       //Serial.print("[");
       //Serial.print(v,4);
       //Serial.print("]");
        if(absPosMode == false){
          y = y + v;
        }else{
          y = v;
        }
        //gcASCIIval = gcFile.read();
        gcASCIIval = memoryFile.charAt(readCount);
        readCount++;
        if(gcASCIIval == 13){ // Execute move only if it's the last value on that line
          ExecuteGCodeMove();
        }
        readCount--;
      break;
      case 90: // 'Z' Denotes 3rd Axis movement followed by move end value
        zManualMove = true;
       //Serial.print(char(gcASCIIval));
        z = GCodeValueBuild();
       //Serial.print("[");
       //Serial.print(z,4);
       //Serial.print("]");
        //gcASCIIval = gcFile.read();
        gcASCIIval = memoryFile.charAt(readCount);
        readCount++;
        if(gcASCIIval == 13){ // Execute move only if it's the last value on that line
          ExecuteGCodeMove();
        }
        readCount--;
      break;
      default:
       //Serial.print("#"); // any other character
      break;
    }
    gcLiveCharPos = readCount;
  }
}

void ExecuteGCodeMove(void){
  unsigned long pauseTime = 0;
  if(zManualMove == true){
    while(KeyPadRetrieve() != 11){
      EncoderPosition();
      displayZAxisSet = true;
      if(KeyPadRetrieve() == 1){
        cncEnd = true;
        gCodeMode = 100; // Stops the Execution of GCode move
      }
      if(cncEnd == true && KeyPadRetrieve() == 10){
        cncEnd = false;
      }
    }
    KeyHeldWait();
    displayZAxisSet = false;
    zManualMove = false;
  }
  pauseTime = millis();
  if(gCodePause == true){
    while(gCodePauseVal + pauseTime > millis()){
      EncoderPosition();
    }
    gCodePause = false;
  }
  switch(gCodeMode){
    case 0:
      G00(x,y,z);
    break;
    case 1:
      G01(x,y,z,f/3); // Look at using feed (f) for feed speed, currently in mm/s
    break;
    case 2:
      if(absArcCenMode == false){
        i = i + preX;
        j = j + preY;
        k = k + preZ;
      }
      //Serial.print(" [absArcCenMode = ");
      //Serial.print(absArcCenMode);
      //Serial.print("  preX = ");
      //Serial.print(preX);
      //Serial.print("  preY = ");
      //Serial.print(preY);
      //Serial.print("  preZ = ");
      //Serial.print(preZ);
      //Serial.print("  i = ");
      //Serial.print(i);
      //Serial.print("  j = ");
      //Serial.print(j);
      //Serial.print("  k = ");
      //Serial.print(k);
      //Serial.print(" G02] ");
      G02(x,y,z,i,j,k,f/3); // Look at using feed (f) for feed speed, currently in mm/s
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
      //Serial.print(" [absArcCenMode = ");
      //Serial.print(absArcCenMode);
      //Serial.print("  preX = ");
      //Serial.print(preX);
      //Serial.print("  preY = ");
      //Serial.print(preY);
      //Serial.print("  preZ = ");
      //Serial.print(preZ);
      //Serial.print("  i = ");
      //Serial.print(i);
      //Serial.print("  j = ");
      //Serial.print(j);
      //Serial.print("  k = ");
      //Serial.print(k);
      //Serial.print(" G03] ");
      G03(x,y,z,i,j,k,f/3); // Look at using feed (f) for feed speed, currently in mm/s
      i = 0;
      j = 0;
      k = 0;
    break;
  }
}
  
double GCodeValueBuild (void){
  String valueString = "";
  double value = 0.0d;
  //gcASCIIval = gcFile.read();
  gcASCIIval = memoryFile.charAt(readCount);
  readCount++;
  while(gcASCIIval != 32 && gcASCIIval != 13){
    if(gcASCIIval >= 48 && gcASCIIval <= 57 || gcASCIIval == 43 || gcASCIIval == 45 || gcASCIIval == 46){
      valueString += char(gcASCIIval);
    }
    //gcASCIIval = gcFile.read();
    gcASCIIval = memoryFile.charAt(readCount);
    readCount++;
  }
  readCount--;
  //gcFile.seek(readCount);
  value = valueString.toFloat();
  return value;
}

//===\/ G CODE COMMANDS \/=========================================================================================\/ G CODE COMMANDS \/===

void G00 (double xDestination_mm, double yDestination_mm, double zDestination_mm){
 //Serial.print("---G00---");
  GlobalMovemm (250000, xDestination_mm, 250000, yDestination_mm, 0, zDestination_mm, true);
}

void G01 (double xDestination_mm, double yDestination_mm, double zDestination_mm, double feedSpeed_mms){
 //Serial.print("---G01---");
  xyzSpeed = feedSpeed_mms * 273; // -16384-
  LineMill (xyzSpeed, xDestination_mm, xyzSpeed, yDestination_mm, xyzSpeed, zDestination_mm);
}

void G02 (double xDestination_mm, double yDestination_mm, double zDestination_mm, double iCenter_mm, double jCenter_mm, double kCenter_mm, double feedSpeed_mms){
 //Serial.print("---G02---");
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
      zDiffStart = (double) zTab_m0 + (double) zDisplayOffset;
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
      endRadians = atan2 (zDiffEnd,xDiffEnd); // swap arguments if undesired answer #######
      zDiffEnd = sq(zDiffEnd);
      xDiffEnd = sq(xDiffEnd);
      radiusSq = zDiffEnd + xDiffEnd;
      radiusEnd = sqrt(radiusSq);
    break;
    case 19: // YZ Plane Selected
      // Calculates the Current Position in mm
      yDiffStart = (double) yTab_m0 + (double) yDisplayOffset;
      zDiffStart = (double) zTab_m0 + (double) zDisplayOffset;
      yDiffStart = yDiffStart / 16384;
      zDiffStart = zDiffStart / 16384;
      yDiffStart = yDiffStart - jCenter_mm;
      zDiffStart = zDiffStart - kCenter_mm;
      startRadians = atan2 (zDiffStart,yDiffStart); // swap arguments if undesired answer #######
      
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

  gfeedSpeed = feedSpeed_mms * 273; // -16384- 273 set for ar timing around a circle
  
  // Checks the Arc movement is in line with the last move (G01, then G02)
  if (radiusStart < radiusEnd + 0.0001 || radiusStart > radiusEnd - 0.0001){
    //Serial.print("RADIUS MATCH");
    radius = radiusStart;
    ArcMillG02 (xEndOfArc_mm, yEndOfArc_mm, zEndOfArc_mm, iCenter_mm, jCenter_mm, kCenter_mm, startRadians, endRadians, radius, gfeedSpeed);
  }else{
    //Serial.println("ERROR: Start and End Radia do not match");
  }
}

void G03 (double xDestination_mm, double yDestination_mm, double zDestination_mm, double iCenter_mm, double jCenter_mm, double kCenter_mm, double feedSpeed_mms){
 //Serial.print("---G03---");
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
      zDiffStart = (double) zTab_m0 + (double) zDisplayOffset;
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
      endRadians = atan2 (zDiffEnd,xDiffEnd); // swap arguments if undesired answer #######
      zDiffEnd = sq(zDiffEnd);
      xDiffEnd = sq(xDiffEnd);
      radiusSq = zDiffEnd + xDiffEnd;
      radiusEnd = sqrt(radiusSq);
    break;
    case 19: // YZ Plane Selected
      // Calculates the Current Position in mm
      yDiffStart = (double) yTab_m0 + (double) yDisplayOffset;
      zDiffStart = (double) zTab_m0 + (double) zDisplayOffset;
      yDiffStart = yDiffStart / 16384;
      zDiffStart = zDiffStart / 16384;
      yDiffStart = yDiffStart - jCenter_mm;
      zDiffStart = zDiffStart - kCenter_mm;
      startRadians = atan2 (zDiffStart,yDiffStart); // swap arguments if undesired answer #######
      
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
  
  gfeedSpeed = feedSpeed_mms * 273; // -16384- 273 set for timing around a circle
  
  if (radiusStart < radiusEnd + 0.0001 || radiusStart > radiusEnd - 0.0001){
    //Serial.print("RADIUS MATCH");
    radius = radiusStart;
    ArcMillG03 (xEndOfArc_mm, yEndOfArc_mm, iCenter_mm, jCenter_mm, startRadians, endRadians, radius, gfeedSpeed);
  }else{
    //Serial.println("ERROR: Start and End Radia do not match");
  }
}

//===/\ G CODE COMMANDS /\=========================================================================================/\ G CODE COMMANDS /\===

void ArcMillG02 (double xEndOfArc_mm, double yEndOfArc_mm, double zEndOfArc_mm, double iCentre_mm, double jCentre_mm, double kCentre_mm, double startRadians, double endRadians, double radius, long amfeedSpeed){
  double sin_n = 0.0d;
  double cos_n = 0.0d;
  double radStep = 0.0d;
  long xCirPos_m1 = 0;
  long yCirPos_m0 = 0;
  long zCirPos_m0 = 0;
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
  zCirPos_m0 = kCentre_mm * 16384;
  xCirPos_m1 = xCirPos_m1 - xDisplayOffset;
  yCirPos_m0 = yCirPos_m0 - yDisplayOffset;
  zCirPos_m0 = zCirPos_m0 - zDisplayOffset;

  radStep = -PI * amfeedSpeed * 0.00000032 / radius; // 0.00000022 is the factor for radian steps to set speed in mm/s (tweak to adjust)
  
  if (amfeedSpeed < 32768){
    amfeedSpeed = 32768;
  }

  while (phase > endRadians) {
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
        GlobalMove(amfeedSpeed,xCirPos_m1,amfeedSpeed,yCirPos_m0,0,zCirPos_m0,firstCircleLoop);
        firstCircleLoop = false;
        EncoderPosition();
        xCirPos_m1 = xCirPos_m1 - cos_cps;
        yCirPos_m0 = yCirPos_m0 - sin_cps;
      break;
      case 18:
        zCirPos_m0 = zCirPos_m0 + cos_cps;
        xCirPos_m1 = xCirPos_m1 + sin_cps;
        GlobalMove(amfeedSpeed,xCirPos_m1,amfeedSpeed,yCirPos_m0,0,zCirPos_m0,firstCircleLoop);
        firstCircleLoop = false;
        EncoderPosition();
        zCirPos_m0 = zCirPos_m0 - cos_cps;
        xCirPos_m1 = xCirPos_m1 - sin_cps;
      break;
      case 19:
        yCirPos_m0 = yCirPos_m0 + cos_cps;
        zCirPos_m0 = zCirPos_m0 + sin_cps;
        GlobalMove(amfeedSpeed,xCirPos_m1,amfeedSpeed,yCirPos_m0,0,zCirPos_m0,firstCircleLoop);
        firstCircleLoop = false;
        EncoderPosition();
        yCirPos_m0 = yCirPos_m0 - cos_cps;
        zCirPos_m0 = zCirPos_m0 - sin_cps;
      break;
    }
  }
}

void ArcMillG03 (double xEndOfArc_mm, double yEndOfArc_mm, double iCentre_mm, double jCentre_mm, double startRadians, double endRadians, double radius, long amfeedSpeed){
  double sin_n = 0.0d;
  double cos_n = 0.0d;
  double radStep = 0.0d;
  long xCirPos_m1 = 0;
  long yCirPos_m0 = 0;
  long zCirPos_m0 = 0;
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
  xCirPos_m1 = xCirPos_m1 - xDisplayOffset;
  yCirPos_m0 = yCirPos_m0 - yDisplayOffset;

  radStep = PI * amfeedSpeed * 0.00000032 / radius; // 0.00000022 is the factor for radian steps, used to set speed in mm/s (tweak to adjust)
  
  if (amfeedSpeed < 32768){
    amfeedSpeed = 32768;
  }

  while (phase < endRadians) {
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
        GlobalMove(amfeedSpeed,xCirPos_m1,amfeedSpeed,yCirPos_m0,0,zCirPos_m0,firstCircleLoop);
        firstCircleLoop = false;
        EncoderPosition();
        xCirPos_m1 = xCirPos_m1 - cos_cps;
        yCirPos_m0 = yCirPos_m0 - sin_cps;
      break;
      case 18:
        zCirPos_m0 = zCirPos_m0 + cos_cps;
        xCirPos_m1 = xCirPos_m1 + sin_cps;
        GlobalMove(amfeedSpeed,xCirPos_m1,amfeedSpeed,yCirPos_m0,0,zCirPos_m0,firstCircleLoop);
        firstCircleLoop = false;
        EncoderPosition();
        zCirPos_m0 = zCirPos_m0 - cos_cps;
        xCirPos_m1 = xCirPos_m1 - sin_cps;
      break;
      case 19:
        yCirPos_m0 = yCirPos_m0 + cos_cps;
        zCirPos_m0 = zCirPos_m0 + sin_cps;
        GlobalMove(amfeedSpeed,xCirPos_m1,amfeedSpeed,yCirPos_m0,0,zCirPos_m0,firstCircleLoop);
        firstCircleLoop = false;
        EncoderPosition();
        yCirPos_m0 = yCirPos_m0 - cos_cps;
        zCirPos_m0 = zCirPos_m0 - sin_cps;
      break;
    }
  }
}

void CircleMill(void){ //----------------------------------------------------------------------------------------------void CircleMill()---
  bool circleDisplay = false;
  bool case0FirstTime = true;
  long xCirPos_m1 = 0;
  long yCirPos_m0 = 0;

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

  //G00(circleXcoord,circleYcoord,0);
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
}

//-----------------------------------------------------------------------------------------------------------------------void LineMill()---
void LineMill (long xLMSpeed, double xLMDestination_mm, long yLMSpeedNU, double yLMDestination_mm, long zLMSpeed, long zLMDestination){ // Mill or move from one point to another with interpollation (ALL Axes reach target at same time)
  double loopTime = 0.01d; // Adjust to get the feed speed (mm/s) correct
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
  long yLMSpeed = 0;
 // long zLinPos_m2 = 0;
  bool firstLineLoop = true;
  bool xDeltaNeg = false;
  bool yDeltaNeg = false;
  bool xDeltaPrimary = true;
  bool xDeltaZero = false;
  bool yDeltaZero = false;
  long LMSpeedPassThrough = 0;
  long xLMDestination = 0;
  long yLMDestination = 0;
  //delay (100);
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
      
    for(xLinPos_m1; xLinPos_m1 != progress + xDelta_m1;){
      if (xLMDestination - xStep <= xLinPos_m1 && xLMDestination + xStep >= xLinPos_m1 || xLMDestination + xStep <= xLinPos_m1 && xLMDestination - xStep >= xLinPos_m1){
        xLinPos_m1 = xLMDestination;
      }else{
        xLinPos_m1 = xLinPos_m1 + xStep;
      }
      if (yLMDestination - yStep <= yLinPos_m0 && yLMDestination + yStep >= yLinPos_m0 || yLMDestination + yStep <= yLinPos_m0 && yLMDestination - yStep >= yLinPos_m0){
        yLinPos_m0 = yLMDestination;
      }else{
        yLinPos_m0 = yLinPos_m0 + yStep;
      }
      //  zLinPos_m2 = zLinPos_m2 - zStep;
      GlobalMove(LMSpeedPassThrough,xLinPos_m1,LMSpeedPassThrough,yLinPos_m0,0,0,firstLineLoop);
      firstLineLoop = false;
      EncoderPosition();
    }
  }
  if (xDeltaPrimary == false){
    progress = yTab_m0;
          
    for(yLinPos_m0; yLinPos_m0 != progress + yDelta_m0;){
      if (xLMDestination - xStep <= xLinPos_m1 && xLMDestination + xStep >= xLinPos_m1 || xLMDestination + xStep <= xLinPos_m1 && xLMDestination - xStep >= xLinPos_m1){
        xLinPos_m1 = xLMDestination;
      }else{
        xLinPos_m1 = xLinPos_m1 + xStep;
      }
      if (yLMDestination - yStep <= yLinPos_m0 && yLMDestination + yStep >= yLinPos_m0 || yLMDestination + yStep <= yLinPos_m0 && yLMDestination - yStep >= yLinPos_m0){
        yLinPos_m0 = yLMDestination;
      }else{
        yLinPos_m0 = yLinPos_m0 + yStep;
      }
      //  zLinPos_m2 = zLinPos_m2 - zStep;
      GlobalMove(LMSpeedPassThrough,xLinPos_m1,LMSpeedPassThrough,yLinPos_m0,0,0,firstLineLoop);
      firstLineLoop = false;
      EncoderPosition();
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
void GlobalMove (long xSpeed, long xDestination, long ySpeed, long yDestination, long zSpeed, long zDestination, bool firstLoop){
// Pre-Move Backlash control
  xTab_m1_New = xDestination;
  yTab_m0_New = yDestination;
  
  xCNC_m1 = xPos_m1;
  if (xTab_m1_New > xTab_m1){
    if (xBacklashIn == false){
      odrive_serial << "w axis" << 1 << ".controller.config.vel_limit " << backLashSpeed << '\n';
      xPos_m1 = xPos_m1 + xBacklash;
      odrive.SetPosition(1, xPos_m1);
      while(xPos_m1 - xEnc_m1 < -163.84 || xPos_m1 - xEnc_m1 > 163.84 && eStopPressed == false){
        EncoderPosition();
      }
      odrive_serial << "w axis" << 1 << ".controller.config.vel_limit " << xSpeed << '\n';
    }
    xBacklashIn = true;
    xPos_m1 = xTab_m1_New + xBacklash;
    xTab_m1 = xTab_m1_New;
  }
  if (xTab_m1_New < xTab_m1){
    if (xBacklashIn == true){
      odrive_serial << "w axis" << 1 << ".controller.config.vel_limit " << backLashSpeed << '\n';
      xPos_m1 = xPos_m1 - xBacklash;
      odrive.SetPosition(1, xPos_m1);
      while(xPos_m1 - xEnc_m1 < -163.84 || xPos_m1 - xEnc_m1 > 163.84 && eStopPressed == false){    // Wait for the motor to almost finish moving (within 0.01mm)
        EncoderPosition();
      } 
      odrive_serial << "w axis" << 1 << ".controller.config.vel_limit " << xSpeed << '\n';
    }
    xBacklashIn = false;
    xPos_m1 = xTab_m1_New;
    xTab_m1 = xTab_m1_New;
  }
  yCNC_m0 = yPos_m0;
  if (yTab_m0_New > yTab_m0){
    if (yBacklashIn == false){
      odrive_serial << "w axis" << 0 << ".controller.config.vel_limit " << backLashSpeed << '\n';
      yPos_m0 = yPos_m0 + yBacklash;
      odrive.SetPosition(0, yPos_m0);
      while(yPos_m0 - yEnc_m0 < -163.84 || yPos_m0 - yEnc_m0 > 163.84 && eStopPressed == false){
        EncoderPosition();
      }
      odrive_serial << "w axis" << 0 << ".controller.config.vel_limit " << ySpeed << '\n';
    }
    yBacklashIn = true;
    yPos_m0 = yTab_m0_New + yBacklash;
    yTab_m0 = yTab_m0_New;
  }
  if (yTab_m0_New < yTab_m0){
    if (yBacklashIn == true){
      odrive_serial << "w axis" << 0 << ".controller.config.vel_limit " << backLashSpeed << '\n';
      yPos_m0 = yPos_m0 - yBacklash;
      odrive.SetPosition(0, yPos_m0);
      while(yPos_m0 - yEnc_m0 < -163.84 || yPos_m0 - yEnc_m0 > 163.84 && eStopPressed == false){    // Wait for the motor to almost finish moving (within 0.01mm)
        EncoderPosition();
      } 
      odrive_serial << "w axis" << 0 << ".controller.config.vel_limit " << ySpeed << '\n';
    }
    yBacklashIn = false;
    yPos_m0 = yTab_m0_New;
    yTab_m0 = yTab_m0_New;
  }

// Only X-Axis is moving and less than 2mm/s
  if (xSpeed < 32768 && xSpeed != 0 && ySpeed == 0 && zSpeed == 0){   // Positive direction Fine control of speed if below 2mm/s (slower than the ODrive min speed)
    if (xfirstLoop == true){
      odrive_serial << "w axis" << 1 << ".controller.config.vel_limit " << 32768 << '\n';
    }
    globalMoveLoopTime = millis();
    while(xPos_m1 - xEnc_m1 < -1638.4 || xPos_m1 - xEnc_m1 > 1638.4){
      EncoderPosition();
      if (xBacklashIn == true){
        if(xStepIncrement * 2 > xPos_m1 - xEnc_m1){ // alter multiplier to adjust for end accuracy
          xCNC_m1 = xPos_m1;
        }else{
          timeElapsed = millis() - globalMoveLoopTime;
          globalMoveLoopTime = millis();
          xStepIncrement = xSpeed * timeElapsed / 1000 + 1;
          xCNC_m1 = xCNC_m1 + xStepIncrement;
        }  
      }else{
         if(xStepIncrement * -2 < xPos_m1 - xEnc_m1){ // alter multiplier to adjust for end accuracy
           xCNC_m1 = xPos_m1;
        }else{
          timeElapsed = millis() - globalMoveLoopTime;
          globalMoveLoopTime = millis();
          xStepIncrement = xSpeed * timeElapsed / 1000 + 1;
          xCNC_m1 = xCNC_m1 - xStepIncrement;
        }   
      }
      odrive.SetPosition(1, xCNC_m1);
    }
  }

// Only Y-Axis is moving and less than 2mm/s
  if (xSpeed == 0 && ySpeed < 32768 && ySpeed != 0 && zSpeed == 0){   // Positive direction Fine control of speed if below 2mm/s (slower than the ODrive min speed)
    if (yfirstLoop == true){
      odrive_serial << "w axis" << 0 << ".controller.config.vel_limit " << 32768 << '\n';
    }
    globalMoveLoopTime = millis();
    while(yPos_m0 - yEnc_m0 < -1638.4 || yPos_m0 - yEnc_m0 > 1638.4){
      EncoderPosition();
      if (yBacklashIn == true){
        if(yStepIncrement * 2 > yPos_m0 - yEnc_m0){ // alter multiplier to adjust for end accuracy
          yCNC_m0 = yPos_m0;
        }else{
          timeElapsed = millis() - globalMoveLoopTime;
          globalMoveLoopTime = millis();
          yStepIncrement = ySpeed * timeElapsed / 1000 + 1;
          yCNC_m0 = yCNC_m0 + yStepIncrement;
        }  
      }else{
         if(yStepIncrement * -2 < yPos_m0 - yEnc_m0){ // alter multiplier to adjust for end accuracy
           yCNC_m0 = yPos_m0;
        }else{
          timeElapsed = millis() - globalMoveLoopTime;
          globalMoveLoopTime = millis();
          yStepIncrement = ySpeed * timeElapsed / 1000 + 1;
          yCNC_m0 = yCNC_m0 - yStepIncrement;
        }   
      }
      odrive.SetPosition(0, yCNC_m0);
    }
  }

// Only X-Axis is moving and greater than or equal to 2mm/s
  if (xSpeed > 32767 && ySpeed == 0 && zSpeed == 0){
    if (xfirstLoop == true){
      odrive_serial << "w axis" << 1 << ".controller.config.vel_limit " << xSpeed << '\n';
    }
    globalMoveLoopTime = millis();
      xCNC_m1 = xPos_m1;
      odrive.SetPosition(1, xCNC_m1);
    while(xPos_m1 - xEnc_m1 < -1638.4 || xPos_m1 - xEnc_m1 > 1638.4){
      EncoderPosition();
    } 
  }

// Only Y-Axis is moving and greater than or equal to 2mm/s
  if (xSpeed == 0 && ySpeed > 32767 && zSpeed == 0){
    if (yfirstLoop == true){
      odrive_serial << "w axis" << 0 << ".controller.config.vel_limit " << ySpeed << '\n';
    }
    globalMoveLoopTime = millis();
      yCNC_m0 = yPos_m0;
      odrive.SetPosition(0, yCNC_m0);
    while(yPos_m0 - yEnc_m0 < -1638.4 || yPos_m0 - yEnc_m0 > 1638.4){
      EncoderPosition();
    } 
  }

// X-Axis and Y-Axis moving and both are greater than or equal to 2mm/s, can be different speeds
  if (xSpeed > 32767 && ySpeed > 32767 && zSpeed == 0){
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
      EncoderPosition();
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

void FullCalibration(int motornum){ //----------------------------------------------------------------------------void FullCalibration()---
  int calibTime = millis();
  int requested_state;
  
  cncStatVal = 1;
  switch(motornum){
    case 0:
      axisYstate = 3;
      oDvStatVal = 3;
      break;
    case 1:
      axisXstate = 2;
      oDvStatVal = 2;
      break;
    case 2:
      axisZstate = 4;
      oDvStatVal = 4;
      break;
  }
  requested_state = ODriveArduino::AXIS_STATE_FULL_CALIBRATION_SEQUENCE;
  odrive.run_state(motornum, requested_state, false);

  while(calibTime + 14000 > millis()){
    EncoderPosition();
  }
  
  requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
  odrive.run_state(motornum, requested_state, false);

  switch (motornum){
    case 0:
      yTab_m0_New = yTab_m0 - yTab_m0 - yTab_m0;
      odrive.SetPosition(0, yPos_m0);
      EncoderPosition();
      oDrive0m0CLC = true;
      eStopPressed = false;
      break;
    case 1:
      xTab_m1 = 0;
      xTab_m1_New = xEnc_m1;
      odrive.SetPosition(1, xPos_m1);
      EncoderPosition();
      oDrive0m1CLC = true;
      eStopPressed = false;
      break;
    default:
      break;
  }
}

void EncoderPosition(void){ //------------------------------------------------------------------------------------void EncoderPosition()---
  odrive_serial << "r axis" << 1 << ".encoder.pos_estimate " << '\n';
  xEnc_m1 = odrive.readFloat();
  //delay (3);
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
  //delay (2);
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
  TempAxesVals(); // Generates X, Y and Z Psuedo values
  loopTime = micros() - loopTime; // loopTime now holds time taken to move through the loop once
  if(drawMainScreenDelay + 25 < millis()){
    DrawMainScreen(); // Draws all elements on the main scren with updated values (1 Layer)
    drawMainScreenDelay = millis();
  }
  if(loopTimeDispDelay + 200 < millis()){
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
  requested_state = ODriveArduino::AXIS_STATE_IDLE;
  odrive.run_state(1, requested_state, false);
  requested_state = ODriveArduino::AXIS_STATE_IDLE;
  odrive.run_state(2, requested_state, false);
  requested_state = ODriveArduino::AXIS_STATE_IDLE;
  odrive.run_state(3, requested_state, false);
  cncStatVal = 0;
  axisXstate = 0;
  axisYstate = 0;
  axisZstate = 0;
  oDvStatVal = 0;
}

// TEMPORARY AXIS VALUES
void TempAxesVals(void){ //------------------------------------------------------------------------------------------void TempAxesVals()---
  ph1 = ph1 + 0.005f;
  ph2 = ph2 + 0.01f;
  ph3 = ph3 + 0.001f;
  ph4 = ph4 + 0.001f;
  //x_mmDisplay = 150.0f * cos(ph1);
  //y_mmDisplay = 75.0f * sin(ph2);
  //z_mmDisplay = 300.0f * sin(ph3);
  if(spindleOn == true){
    spindleRPM = 2000.0f * cos(ph4);
  }else{
    spindleRPM = 0;
  }
  spinMotBmpRot = spinMotBmpRot + spindleRPM;
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

//----------Version Major_Minor_Patch changes-----Ver1_0_0----------

//----------Things to address-----------

// Look at velocity control either here or on ODrive for CNC accuracy
// GCode test menu with GCode test (runs GCode above the work)
// Bite size chunks
// SD Card reader
// Joystick controlled menu?
// HDMI Display using Gameduino 3x Dazzler on SPI port
// Reverse recorded Y-Axis travel
// Use physical microswitch endstops connected to ODrive for safety with ODrive system and connect to Arduino for program control also
// After Emergency Stop or during, force the ODrive into idle mode for all motors.
// Force spindle motor to idle when doing a tool change
// Menu adjustable Jog speed?
// HDMI Output for menu system
// HDMI GCode to graphics (can this be a sprite?)
// SD Card support of GCode files
// 8x8 Display moved to CNC machine
// Emergency Stop tied to ODrive idle and Interrupt/input
// Automatic Backlash measurement at startup? - Save results to log file for trend analysis
// Auto Tool Height Setting
// Tool Change dwell introduction
// 

//-----------------------------------------------Notes-----------------------------------------------
// Low speed limit in Counts Per Second = 32768 (Minimum speed in counts per second for ODrive stability at 12v)
// High speed limit in Counts Per Second = 360284 (Maximum speed in counts per second for ODrive stability at 12v)
