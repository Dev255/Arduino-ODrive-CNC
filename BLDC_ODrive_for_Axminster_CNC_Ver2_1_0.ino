/*
Neil Devonshire - Dev255
YouTube Dev255
www.dev255.co.uk

G-Code to CNC sketch to machine features from a main menue system
comments added to help describe the funcion of each statement (if requied)

    Copyright (C) <2020>  <Neil Devonshire>

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

//----------Version Major_Minor_Patch changes-----Ver2_1_0----------

// Remove all joypad functions as this was removed.
// X-Axis added to Circle cosine control
// Added tool, circle, zero and speed parameters within circle milling function
// Ability to clear entry while entering values (had to reset the device to stop machine moving to wrong input)

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

//---------------------------------------------Functions---------------------------------------------
//
// KeyPadRetrieve(void); -------------- Returns the key press value from the Key Pad. i.e. 1=1, 2=2, 3=3, etc, *=10 & #=11. Keypad 2 returns 1=-1, 2=-2, etc.
// FullCalibration(void); ------------- Calibrates each motor in turn, X,Y, then Z
// AllStop (void); -------------------- Sets all motors idle (AXIS_STATE_IDLE), thereby stopping all movement
// EncoderPosition (float yEnc_m0); --- 

//-----------------------------------------------Notes-----------------------------------------------
// Low speed limit in Counts Per Second = 32768 (Minimum speed in counts per second for ODrive stability at 12v)
// High speed limit in Counts Per Second = 360284 (Maximum speed in counts per second for ODrive stability at 12v)

#include <LiquidCrystal.h>
#include <ODriveArduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_LEDBackpack.h>
#include <Math.h>

HardwareSerial & odrive_serial = Serial1;

// Emergency Stop Interuprt
const byte interruptESPin = 2;
volatile byte stateES = LOW;

// Printing with stream operator
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }

// ODrive object
ODriveArduino odrive(odrive_serial);

// VARIOUS VARIABLES FOR EACH AXIS

// X-Axis variables
long xPos_m1 = 0;                    // Commanded position of X-Axis Motor in encoder counts
long xEnc_m1 = 0;                    // Current Estimated encoder counts of X-Axis rotary encoder
long xTab_m1 = 0;                    // Current table position in encoder counts
long xTab_m1_New = 0;                // New table position before X-Axis move in encoder counts
long xDisplayOffset = 0;             // Difference between acual Table position (xTab_m1) and displayed position for X-Axis in encoder counts
long xDisplay = 0;                   // Displayed position of X-Axis in encoder counts, offset from xTab_m1 when X axis zeroed
float x_mmDisplay = 0;               // Displayed position of X-Axis in mm, offset from xTab_m1 when X axis zeroed

// Y-Axis variables
long yPos_m0 = 0;                    // Commanded position of Y-Axis Motor in encoder counts
long yEnc_m0 = 0;                    // Current Estimated encoder counts of Y-Axis rotary encoder
long yTab_m0 = 0;                    // Current table position in encoder counts
long yTab_m0_New = 0;                // New table position before Y-Axis move in encoder counts
long yDisplayOffset = 0;             // Difference between actual Table position (yTab_m0) and displayed position for Y-Axis in encoder counts
long yDisplay = 0;                   // Displayed position of Y-Axis in encoder counts, offset from yTab_m0 when Y axis zeroed
float y_mmDisplay = 0;               // Displayed position of Y-Axis in mm, offset from yTab_m0 when Y axis zeroed

// Z-Axis variables
long zPos_m0 = 0;                    // Commanded position of Z-Axis Motor in encoder counts
long zEnc_m0 = 0;                    // Current Estimated encoder counts of Z-Axis rotary encoder
long zToolTip_m0 = 0;                // Current Tool Tip position in encoder counts
long zToolTip_m0_New = 0;            // New Tool Tip position before Z-Axis move in encoder counts
long zDisplay = 0;                   // Displayed position of Z-Axis in encoder counts, offset from zTab_m0 when Z axis zeroed
float z_mmDisplay = 0;               // Displayed position of z-Axis in mm, offset from zTab_m0 when Z axis zeroed

// Backlash variables
const long backLashSpeed = 250000;      // Constant - Speed for taking up backlash (X & Y axes affected at present), no backlash control on Z axis

// X-Axis Backlash variables
const long xBacklash = 6269;          // Number of encoder counts to remove backlash, tune on table with .pos_gain from ODriveTools.
bool xBacklashIn = false;             // Set to true when X-Axis moving in plus (count increases) direction.

// Y-Axis Backlash variables
const long yBacklash = 2694;          // Number of encoder counts to remove backlash, tune on table with .pos_gain from ODriveTools.
bool yBacklashIn = false;             // Set to true when Y-Axis moving in plus (count increases) direction.

// lcd1 Pin setup
const int lcd1Backlight = 13; //lcd1 LED output, also SMLED on Arduino (this will be PWM to control brightness - see setup)
const int rs_1 = 22;
const int rw_1 = 24;
const int e_1 = 26;
const int db0_1 = 28;
const int db1_1 = 30;
const int db2_1 = 32;
const int db3_1 = 34;
const int db4_1 = 36;
const int db5_1 = 38;
const int db6_1 = 40;
const int db7_1 = 42;
LiquidCrystal lcd1(rs_1, rw_1, e_1, db0_1, db1_1, db2_1, db3_1, db4_1, db5_1, db6_1, db7_1);

// lcd2 Pin setup
const int lcd2Backlight = 12; //lcd2 LED output (this will be PWM to control brightness - see setup)
const int rs_2 = 23;
const int rw_2 = 25;
const int e_2 = 27;
const int db0_2 = 29;
const int db1_2 = 31;
const int db2_2 = 33;
const int db3_2 = 35;
const int db4_2 = 37;
const int db5_2 = 39;
const int db6_2 = 41;
const int db7_2 = 43;
LiquidCrystal lcd2(rs_2, rw_2, e_2, db0_2, db1_2, db2_2, db3_2, db4_2, db5_2, db6_2, db7_2);

// Program Status Flags
volatile bool oDrive0m0CLC = false; // Is ODrive 0, motor 0 (Y-Axis) in Closed Loop Control? Volatile because used in Emergency Stop ISR (interrupt)
volatile bool oDrive0m1CLC = false; // Is ODrive 0, motor 1 (X-Axis) in Closed Loop Control? Volatile because used in Emergency Stop ISR (interrupt)
volatile bool eStopPressed = false; // Set to true only when the Emergency Stop has been pressed, resets to false once callibration is carried out.

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

// Keypad Pin setup and Variables
const int keyPadY1 = 44;
const int keyPadY2 = 46;
const int keyPadY3 = 48;
const int keyPadY4 = 50;
const int keyPadX1 = 52;
const int keyPadX2 = 45;                                                                                                                                                                                                                                                                                                                        
const int keyPadX3 = 47;
const int keyPad2Y1 = A15;
const int keyPad2Y2 = 49;
const int keyPad2Y3 = 51;
const int keyPad2Y4 = 53;

// Keypad entered values
float keypadEnteredValue = 0.0f;

// ============================================================ EDITED ABOVE THIS DOUBLE LINE ==================================================================

//Menu select
int menuSelect = 0;

unsigned long lcdRefreshTime = 0;
unsigned long lcdRefreshTimeCir = 0;
unsigned long dev255LoopTime1 = 0;
unsigned long dev255LoopTime2 = 0;
byte dev255LoopCount = 2;
float feedSpeed = 0.0f;

//Global Move Function values, moved here to speed up global move
unsigned long globalMoveLoopTime = 0;
unsigned long timeElapsed = 0;
long xStepIncrement = 0;
long xCNC_m1 = 0;
long yStepIncrement = 0;
long yCNC_m0 = 0;

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

float mmJogIncr = 1.0f;
long jogIncr = 1638;
long yJogPos_m0 = 0;
long xJogPos_m1 = 0;
float xCirPos_mm = 0.0;
float yCirPos_mm = 0.0;
long jogSpeed = 150000;
bool xfirstLoop = true;
bool yfirstLoop = true;
long xyzSpeed = 0;
bool lCD2_UpdatePause = false;
int count = 1;

//_____________________________________________________________VOID SETUP()_________________________________________________________

void setup() {
  // Seup the Emergency Stop Interrupt
  pinMode(interruptESPin, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(interruptESPin), ISR_EStop, LOW);
  
  // Set the lcd1 pins mode
  pinMode(lcd1Backlight, OUTPUT); //Set pin 13 to an Output (this is a PWM output)
  pinMode(rs_1, OUTPUT);
  pinMode(rw_1, OUTPUT);
  pinMode(e_1, OUTPUT);
  pinMode(db0_1, OUTPUT);
  pinMode(db1_1, OUTPUT);
  pinMode(db2_1, OUTPUT);
  pinMode(db3_1, OUTPUT);
  pinMode(db4_1, OUTPUT);
  pinMode(db5_1, OUTPUT);
  pinMode(db6_1, OUTPUT);
  pinMode(db7_1, OUTPUT);
  
  // Set the lcd2 pins mode
  pinMode(lcd2Backlight, OUTPUT); //Set pin 12 to an Output (this is a PWM output)
  pinMode(rs_2, OUTPUT);
  pinMode(rw_2, OUTPUT);
  pinMode(e_2, OUTPUT);
  pinMode(db0_2, OUTPUT);
  pinMode(db1_2, OUTPUT);
  pinMode(db2_2, OUTPUT);
  pinMode(db3_2, OUTPUT);
  pinMode(db4_2, OUTPUT);
  pinMode(db5_2, OUTPUT);
  pinMode(db6_2, OUTPUT);
  pinMode(db7_2, OUTPUT);

  // Set the KeyPad pins mode
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
  pinMode(keyPad2Y1, INPUT);          // Analogue Pin, hence no on-board Pull-Up resistor. 22k ohm fitted externally
  pinMode(keyPad2Y2, INPUT);
  pinMode(keyPad2Y2, INPUT_PULLUP);
  pinMode(keyPad2Y3, INPUT);
  pinMode(keyPad2Y3, INPUT_PULLUP);
  pinMode(keyPad2Y4, INPUT);
  pinMode(keyPad2Y4, INPUT_PULLUP);

  // Adafruit 8x8 Matrix Address
  matrix.begin(0x70);  // pass in the address
  matrix.setBrightness(0);
  
  // Serial Port 1 to ODrive = 115200 baud
  odrive_serial.begin(115200);

  // Serial to PC
  Serial.begin(115200);
  while (!Serial) ; // wait for Arduino Serial Monitor to open
  
  odrive_serial << "w axis" << 0 << ".motor.config.current_lim " << 40.0f << '\n';  // Y-Axis Motor
  odrive_serial << "w axis" << 0 << ".controller.config.vel_limit " << 36028.0f << '\n';  // Y-Axis Motor
  odrive_serial << "w axis" << 1 << ".motor.config.current_lim " << 40.0f << '\n';  // X-Axis Motor
  odrive_serial << "w axis" << 1 << ".controller.config.vel_limit " << 36028.0f << '\n';  // X-Axis Motor
  lcd1.begin(20, 4);
  analogWrite(lcd1Backlight, 255);
  lcd2.begin(20, 4);
  analogWrite(lcd2Backlight, 255);

  lcd1.setCursor(0, 0);
  lcd1.print("    Hello, World    ");
  delay(1000);
  lcd1.setCursor(0, 0);
  lcd1.print("Ver 2.0.2     Dev255");
  lcd1.setCursor(0, 1);
  lcd1.print("   Latest Changes   ");
  lcd1.setCursor(0, 2);
  lcd1.print("G-Code movmnts added");
  lcd1.setCursor(0, 3);
  lcd1.print("G00, G01, G02, G03  ");
  for (int lcd1BkltDim = 0; lcd1BkltDim <255; lcd1BkltDim++){  
    analogWrite(lcd1Backlight, lcd1BkltDim);
    delay (5);
  }
  lcd2.setCursor(0, 0);
  lcd2.print("    Hello, World    ");
  delay(1000);
  lcd2.setCursor(0, 0);
  lcd2.print("Add clr on num entry");
  lcd2.setCursor(0, 1);
  lcd2.print("Add set in crcle mll");
  lcd2.setCursor(0, 2);
  lcd2.print("                    ");
  lcd2.setCursor(0, 3);
  lcd2.print("                    ");
  for (int lcd2BkltDim = 0; lcd2BkltDim <255; lcd2BkltDim++){  
    analogWrite(lcd2Backlight, lcd2BkltDim);
    delay (5);
  }

  matrix.setRotation(2);
  for (int8_t x=2; x>=-35; x--) {
    matrix.clear();
    matrix.setCursor(x,0);
    matrix.print("Dev");
    matrix.print("255");
    matrix.writeDisplay();
    delay(100);
  }
  matrix.clear();
  matrix.setRotation(2);
  matrix.drawBitmap(0, 0, dev255, 8, 8, LED_ON);
  matrix.writeDisplay();

  lcdRefreshTime = millis();
  dev255LoopTime1 = millis();
  dev255LoopTime2 = millis();
}

//_____________________________________________________________VOID LOOP()__________________________________________________________

void loop(void) {

  if(eStopPressed == true){
    menuSelect = 0;
    matrix.clear();
    matrix.setRotation(2);
    matrix.drawBitmap(0, 0, emergencyStop, 8, 8, LED_ON);
    matrix.writeDisplay();
  }
  
  Dev255Rotate();
  EncoderPosition();
  if (lCD2_UpdatePause == false){
    LCD2_UpdateDelay(1);
  }

//______________________________MENU 0__________________________________

  if (menuSelect == 0 && oDrive0m0CLC == false || menuSelect == 0 && oDrive0m1CLC == false){
    lcd1.setCursor(0, 0);
    lcd1.print("    ODrives idle    ");
    lcd1.setCursor(0, 1);
    lcd1.print("0 to Calibrate Y-Axs");
    lcd1.setCursor(0, 2);
    lcd1.print("1 to Calibrate X-Axs");
    if(eStopPressed == true){
      lcd1.setCursor(0, 2);
      lcd1.print("Calibration Required");
      lcd1.setCursor(0, 3);
      lcd1.print("After EStop, Press 0");     
    }else{
      lcd1.setCursor(0, 3);
      lcd1.print("Press 5 for Menu    ");      
    }
  }
    
  if (menuSelect == 0 && KeyPadRetrieve() == 0) {
    while (KeyPadRetrieve() <12){
    }
    FullCalibration(0);
    if(oDrive0m0CLC == false || oDrive0m1CLC == false){
      menuSelect = 0;
    }else{
      menuSelect = 1;
    }
  }
  
  if (menuSelect == 0 && KeyPadRetrieve() == 1) {
    while (KeyPadRetrieve() <12){
    }
    FullCalibration(1);
    if(oDrive0m0CLC == false || oDrive0m1CLC == false){
      menuSelect = 0;
    }else{
      menuSelect = 1;
    }
  }

  if (menuSelect == 0 && KeyPadRetrieve() == 5){
    while (KeyPadRetrieve() <12){
    }
    menuSelect = 1;
  } 
  
//______________________________MENU 1__________________________________

  if (menuSelect == 1) {
    lcd1.setCursor(0, 0);
    lcd1.print("1:Circle   2:Line   ");
    lcd1.setCursor(0, 1);
    lcd1.print("3:Feature  4:Parts  ");
    lcd1.setCursor(0, 2);
    lcd1.print("5:         6:       ");
    if(oDrive0m0CLC == false || oDrive0m1CLC == false){
      lcd1.setCursor(0, 3);
      lcd1.print("7:         *:CLC Cal");
    }else{
      lcd1.setCursor(0, 3);
      lcd1.print("7:         8:CLC OFF");
    }
  }

  if (menuSelect == 1 && KeyPadRetrieve() == 10 && oDrive0m0CLC == false || menuSelect == 1 && KeyPadRetrieve() == 10 && oDrive0m1CLC == false){
    while (KeyPadRetrieve() <12){
    }
    menuSelect = 0;
  } 
    
  if (menuSelect == 1 && KeyPadRetrieve() == 1) {
    while (KeyPadRetrieve() <12){
    }
    CircleMill();
    while (KeyPadRetrieve() <12){
    }
  }
  if (menuSelect == 1 && KeyPadRetrieve() == 2) {
    while (KeyPadRetrieve() < 12){
    }
  }
  if (menuSelect == 1 && KeyPadRetrieve() == 3) {
    while (KeyPadRetrieve() < 12){
    }
    menuSelect = 2;
  }
  if (menuSelect == 1 && KeyPadRetrieve() == 4) {
    while (KeyPadRetrieve() < 12){
    }
    menuSelect = 3;
  }
  if (menuSelect == 1 && KeyPadRetrieve() == 8 && oDrive0m0CLC == true) {
    while (KeyPadRetrieve() < 12){
    }
    AllStop();
  }
  
//______________________________MENU 2__________________________________
  
  if (menuSelect == 2) {
    lcd1.setCursor(0, 0);
    lcd1.print("                    ");
    lcd1.setCursor(0, 1);
    lcd1.print("                    ");
    lcd1.setCursor(0, 2);
    lcd1.print("                    ");
    lcd1.setCursor(0, 3);
    lcd2.print("              *:Back");
  }
  
  if (menuSelect == 2 && KeyPadRetrieve() == 10) {
    while (KeyPadRetrieve() < 12){
    }
    menuSelect = 1;
  }

//______________________________MENU 3__________________________________
  
  if (menuSelect == 3) {
    lcd1.setCursor(0, 0);
    lcd1.print("1:Motor Heatsink    ");
    lcd1.setCursor(0, 1);
    lcd1.print("2:Z-AxisDrv Housing ");
    lcd1.setCursor(0, 2);
    lcd1.print("3:SpindleDrv Housing");
    lcd1.setCursor(0, 3);
    lcd1.print("4:That Pixel Bloke  ");
    lcd2.setCursor(0, 0);
    lcd2.print("5:                  ");
    lcd2.setCursor(0, 1);
    lcd2.print("6:                  ");
    lcd2.setCursor(0, 2);
    lcd2.print("7:                  ");
    lcd2.setCursor(0, 3);
    lcd2.print("8:            *:Back");
  }
  
  if (menuSelect == 3 && KeyPadRetrieve() == 10) {
    while (KeyPadRetrieve() < 12){
    }
    menuSelect = 1;
  }
    
  if (menuSelect == 3 && KeyPadRetrieve() == 1) {
    while (KeyPadRetrieve() < 12){
    }
    menuSelect = 4;
  }  
  if (menuSelect == 3 && KeyPadRetrieve() == 2) {
    while (KeyPadRetrieve() < 12){
    }
    menuSelect = 5;
  }  
  if (menuSelect == 3 && KeyPadRetrieve() == 3) {
    while (KeyPadRetrieve() < 12){
    }
    menuSelect = 6;
  } 
  if (menuSelect == 3 && KeyPadRetrieve() == 4) {
    while (KeyPadRetrieve() < 12){
    }
    menuSelect = 5;
  }

//______________________________MENU 4__________________________________
  
  if (menuSelect == 4) {
    lCD2_UpdatePause = true;
    lcd1.setCursor(0, 0);
    lcd1.print("1:Face off Sides    ");
    lcd1.setCursor(0, 1);
    lcd1.print("2:52mm Blind Hole   ");
    lcd1.setCursor(0, 2);
    lcd1.print("3:                  ");
    lcd1.setCursor(0, 3);
    lcd1.print("4:                  ");
    lcd2.setCursor(0, 0);
    lcd2.print("5:                  ");
    lcd2.setCursor(0, 1);
    lcd2.print("6:                  ");
    lcd2.setCursor(0, 2);
    lcd2.print("7:MOVE PART TO CHECK");
    lcd2.setCursor(0, 3);
    lcd2.print("8:GOTO DATUM  *:Back");
  }

  if (menuSelect == 4 && KeyPadRetrieve() == 1) { // Face off sides with 40mm Shell Mill
    while (KeyPadRetrieve() < 12){
    }
    LCD2_UpdateDelay(1);
    G00 (0,0,0);
    G01 (0,120,0,1.5);
    G00 (30,120,0);
    G01 (30,0,0,1.5);
    G00 (60,0,0);
    G01 (60,120,0,1.5);
    G00 (0,0,0);
  }
  
  if (menuSelect == 4 && KeyPadRetrieve() == 2) { // Cut a 52mm Blind hole of 56mm depth using 10mm 1.5r cutter
    count = 100;
    while (KeyPadRetrieve() < 12){
    }
    LCD2_UpdateDelay(1);
    lcd1.setCursor(0, 0);
    lcd1.print(" DRILL 11mm CENTRE  ");
    lcd1.setCursor(0, 1);
    lcd1.print(" HOLE TO 60mm FIRST ");
    lcd1.setCursor(0, 2);
    lcd1.print(" Press # to Move to ");
    lcd1.setCursor(0, 3);
    lcd1.print("       Centre       ");
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    G00 (37.5,37.5,0);
    lcd1.setCursor(0, 0);
    lcd1.print("  FIT 10mm END MILL ");
    lcd1.setCursor(0, 1);
    lcd1.print("& Mill to 56mm Depth");
    lcd1.setCursor(0, 2);
    lcd1.print("   Press # to run   ");
    lcd1.setCursor(0, 3);
    lcd1.print("   Level =          ");
    lcd1.setCursor(12, 3);
    lcd1.print(count);
    lcd1.print(" ");
      while (KeyPadRetrieve() < 12){
      }
      while (KeyPadRetrieve() != 11){
      }
    while (count > 0){
      lcd1.setCursor(0, 0);
      lcd1.print("  Press * to Exit   ");
      lcd1.setCursor(0, 1);
      lcd1.print(" Before Next Level  ");
      while (KeyPadRetrieve() < 12){
      }
      while (KeyPadRetrieve() != 11){
        if (KeyPadRetrieve () == 10){
          count = 0;
        }
      }
      if (count > 0){
        count --;
        G01 (30.5,37.5,0,3);
        G02 (30.5,37.5,0,37.5,37.5,0,3);
        G01 (23.5,37.5,0,3);
        G02 (23.5,37.5,0,37.5,37.5,0,3);
        G01 (16.5,37.5,0,3);
        G02 (16.5,37.5,0,37.5,37.5,0,3);
        G01 (11.5,37.5,0,3);
        G02 (11.5,37.5,0,37.5,37.5,0,3);
        G00 (37.5,37.5,0);
        lcd1.setCursor(12, 3);
        lcd1.print(count);
        lcd1.print(" ");
      } 
    }
  }
   
  if (menuSelect == 4 && KeyPadRetrieve() == 7) { // Move part to a location away from the tool
    while (KeyPadRetrieve() < 12){
    }
    LCD2_UpdateDelay(1);
    G00 (-100,120,0);
  }  
  if (menuSelect == 4 && KeyPadRetrieve() == 8) { // Move tool back to Datum Position
    while (KeyPadRetrieve() < 12){
    }
    LCD2_UpdateDelay(1);
    G00 (0,0,0);
  } 
  if (menuSelect == 4 && KeyPadRetrieve() == 10) {
    while (KeyPadRetrieve() < 12){
    }
    menuSelect = 1;
    lCD2_UpdatePause = false;
  }

//______________________________MENU 5__________________________________
  
  if (menuSelect == 5) {
    lCD2_UpdatePause = true;
    lcd1.setCursor(0, 0);
    lcd1.print("1:Blank Flat End +DP");
    lcd1.setCursor(0, 1);
    lcd1.print("2:Face Off Top/Bottm");
    lcd1.setCursor(0, 2);
    lcd1.print("3:Main Bdy Prts Slow");
    lcd1.setCursor(0, 3);
    lcd1.print("4:Main Bdy Prts Fast");
    lcd2.setCursor(0, 0);
    lcd2.print("5:Rear Flat Mag Hldr");
    lcd2.setCursor(0, 1);
    lcd2.print("6:Holes & CountrBore");
    lcd2.setCursor(0, 2);
    lcd2.print("7:Pixel Feature Top ");
    lcd2.setCursor(0, 3);
    lcd2.print("8:NEXT MENU   *:BACK");
  }

/*
  if (menuSelect == 5 && KeyPadRetrieve() == 1) { // Blank Flat End + Datum Point
    while (KeyPadRetrieve() < 12){
    }
    LCD2_UpdateDelay(1);
    lcd1.setCursor(0, 0);
    lcd1.print("  FIT 6mm END MILL  ");
    lcd1.setCursor(0, 1);
    lcd1.print("                    ");
    lcd1.setCursor(0, 2);
    lcd1.print(" Press # to Move to ");
    lcd1.setCursor(0, 3);
    lcd1.print("       START        ");
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    G00 (0,0,0);
    lcd1.setCursor(0, 0);
    lcd1.print("Blank Flat End + DP ");
    lcd1.setCursor(0, 1);
    lcd1.print("                    ");
    lcd1.setCursor(0, 2);
    lcd1.print("     Press # to     ");
    lcd1.setCursor(0, 3);
    lcd1.print("        RUN         ");
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    G01 (0,85,0,3);
    G00 (0,0,0);
    G01 (13,0,0,3);
    G00 (0,0,0);
  }

  if (menuSelect == 5 && KeyPadRetrieve() == 2) { // Face Off Top and Bottom
    while (KeyPadRetrieve() < 12){
    }
    LCD2_UpdateDelay(1);
    lcd1.setCursor(0, 0);
    lcd1.print("FIT 40mm SHELL MILL ");
    lcd1.setCursor(0, 1);
    lcd1.print("                    ");
    lcd1.setCursor(0, 2);
    lcd1.print(" Press # to Move to ");
    lcd1.setCursor(0, 3);
    lcd1.print("       START        ");
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    G00 (19,-21,0);
    lcd1.setCursor(0, 0);
    lcd1.print("FACE OFF TOP/BOTTOM ");
    lcd1.setCursor(0, 1);
    lcd1.print("                    ");
    lcd1.setCursor(0, 2);
    lcd1.print("     Press # to     ");
    lcd1.setCursor(0, 3);
    lcd1.print("        RUN         ");
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    G01 (19,100,0,3);
    G00 (54,100,0);
    G01 (54,-21,0,3);
    G00 (89,-21,0);
    G01 (89,100,0,3);
    G00 (124,100,0);
    G01 (124,-21,0,3);
    G00 (159,-21,0);
    G01 (159,100,0,3);
    G00 (194,100,0);
    G01 (194,-21,0,3);
    G00 (229,-21,0);
    G01 (229,100,0,3);
    G00 (264,100,0);
    G01 (264,-21,0,3);
    G00 (277,-21,0);
    G01 (277,100,0,3);
  }

  if (menuSelect == 5 && KeyPadRetrieve() == 3) { // Main Body Parts
    count = 1;
    int millSpeed = 1;
    while (KeyPadRetrieve() < 12){
    }
    LCD2_UpdateDelay(1);
    lcd1.setCursor(0, 0);
    lcd1.print("FIT 3mm PR END MILL ");
    lcd1.setCursor(0, 1);
    lcd1.print("TOUCH OFF and ZERO Z");
    lcd1.setCursor(0, 2);
    lcd1.print(" Press # to Move to ");
    lcd1.setCursor(0, 3);
    lcd1.print("       START        ");
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    G00 (-1.5,-4,0);
    lcd1.setCursor(0, 0);
    lcd1.print("   BODY PARTS SLOW  ");
    lcd1.setCursor(0, 1);
    lcd1.print(" 22 Passes at 0.5mm ");
    lcd1.setCursor(0, 2);
    lcd1.print("    Press # to RUN  ");
    lcd1.setCursor(0, 3);
    lcd1.print("     Pass No.    *Qt");
    lcd1.setCursor(14, 3);
    lcd1.print(count);
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    while (count < 23){
      G01 (-1.5,76.5,0,millSpeed);
      G01 (1,79,0,millSpeed);
      G01 (21.5,79,0,millSpeed);
      G01 (24,76.5,0,millSpeed);
      G01 (24,13,0,millSpeed);
      G01 (26.5,10.5,0,millSpeed);
      G01 (47,10.5,0,millSpeed);
      G01 (49.5,13,0,millSpeed);
      G01 (49.5,76.5,0,millSpeed);
      G01 (52,79,0,millSpeed);
      G01 (72.5,79,0,millSpeed);
      G01 (75,76.5,0,millSpeed);
      G01 (75,13,0,millSpeed);
      G01 (77.5,10.5,0,millSpeed);
      G01 (98,10.5,0,millSpeed);
      G01 (100.5,13,0,millSpeed);
      G01 (100.5,76.5,0,millSpeed);
      G01 (103,79,0,millSpeed);
      G01 (120.5,79,0,millSpeed);
      G01 (120.5,67,0,millSpeed);
      G01 (125.5,67,0,millSpeed);
      G01 (125.5,13,0,millSpeed);
      G01 (127.5,10.5,0,millSpeed);
      G01 (148,10.5,0,millSpeed);
      G01 (150.5,13,0,millSpeed);
      G01 (150.5,67,0,millSpeed);
      G01 (155.5,67,0,millSpeed);
      G01 (155.5,79,0,millSpeed);
      G01 (173,79,0,millSpeed);
      G01 (175.5,76.5,0,millSpeed);
      G01 (175.5,13,0,millSpeed);
      G01 (178,10.5,0,millSpeed);
      G01 (198.5,10.5,0,millSpeed);
      G01 (200.5,13,0,millSpeed);
      G01 (200.5,76.5,0,millSpeed);
      G01 (203,79,0,millSpeed);
      G01 (245,79,0,millSpeed);
      G01 (247.5,76.5,0,millSpeed);
      G01 (247.5,13,0,millSpeed);
      G01 (250,10.5,0,millSpeed);
      G01 (264.5,10.5,0,millSpeed);
      G01 (264.5,1,0,millSpeed);
      G01 (267,-1.5,0,millSpeed);
      G01 (275,-1.5,0,millSpeed);
      G01 (277.5,1,0,millSpeed);
      G01 (277.5,10.5,0,millSpeed);
      G01 (292,10.5,0,millSpeed);
      G01 (294.5,13,0,millSpeed);
      G01 (294.5,76.5,0,millSpeed);
      G01 (292,79,0,millSpeed);
      G01 (250,79,0,millSpeed);
      G01 (247.5,76.5,0,millSpeed);
      G01 (247.5,13,0,5);
      G01 (245,10.5,0,millSpeed);
      G01 (230.5,10.5,0,millSpeed);
      G01 (230.5,1,0,millSpeed);
      G01 (228,-1.5,0,millSpeed);
      G01 (220,-1.5,0,millSpeed);
      G01 (217.5,1,0,millSpeed);
      G01 (217.5,10.5,0,millSpeed);
      G01 (203,10.5,0,millSpeed);
      G01 (200.5,13,0,millSpeed);
      G01 (200.5,67,0,5);
      G01 (195.5,67,0,millSpeed);
      G01 (195.5,79,0,millSpeed);
      G01 (178,79,0,millSpeed);
      G01 (175.5,76.5,0,millSpeed);
      G01 (175.5,13,0,5);
      G01 (173,10.5,0,millSpeed);
      G01 (152.5,10.5,0,millSpeed);
      G01 (150.5,13,0,millSpeed);
      G01 (150.5,67,0,5);
      G01 (150.5,76.5,0,millSpeed);
      G01 (148,79,0,millSpeed);
      G01 (130.5,79,0,millSpeed);
      G01 (130.5,67,0,millSpeed);
      G01 (125.5,67,0,millSpeed);
      G01 (125.5,13,0,5);
      G01 (123.5,10.5,0,millSpeed);
      G01 (103,10.5,0,millSpeed);
      G01 (100.5,13,0,millSpeed);
      G01 (100.5,76.5,0,5);
      G01 (98,79,0,millSpeed);
      G01 (77.5,79,0,millSpeed);
      G01 (75,76.5,0,millSpeed);
      G01 (75,13,0,5);
      G01 (72.5,10.5,0,millSpeed);
      G01 (52,10.5,0,millSpeed);
      G01 (49.5,13,0,millSpeed);
      G01 (49.5,76.5,0,5);
      G01 (47,79,0,millSpeed);
      G01 (26.5,79,0,millSpeed);
      G01 (24,76.5,0,millSpeed);
      G01 (24,13,0,5);
      G01 (21.5,10.5,0,millSpeed);
      G01 (1,10.5,0,millSpeed);
      G01 (-1.5,13,0,millSpeed);
      G01 (-1.5,-4,0,5);
      count ++;
      lcd1.setCursor(14, 3);
      lcd1.print(count);
      while (KeyPadRetrieve() < 12){
      }
      while (KeyPadRetrieve() != 11){
        if (KeyPadRetrieve() == 10){
          count = 23;
        }
      }
    }
  }

  if (menuSelect == 5 && KeyPadRetrieve() == 4) { // Main Body Parts Fast
    count = 1;
    while (KeyPadRetrieve() < 12){
    }
    LCD2_UpdateDelay(1);
    lcd1.setCursor(0, 0);
    lcd1.print("FIT 3mm PR END MILL ");
    lcd1.setCursor(0, 1);
    lcd1.print("TOUCH OFF and ZERO Z");
    lcd1.setCursor(0, 2);
    lcd1.print(" Press # to Move to ");
    lcd1.setCursor(0, 3);
    lcd1.print("       START        ");
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    G00 (-1.5,-4,0);
    lcd1.setCursor(0, 0);
    lcd1.print("   BODY PARTS FAST  ");
    lcd1.setCursor(0, 1);
    lcd1.print(" 22 Passes at 0.5mm ");
    lcd1.setCursor(0, 2);
    lcd1.print("    Press # to RUN  ");
    lcd1.setCursor(0, 3);
    lcd1.print("     Pass No.    *Qt");
    lcd1.setCursor(14, 3);
    lcd1.print(count);
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    while (count < 23){
      G01 (-1.5,76.5,0,1);
      G01 (1,79,0,1);
      G01 (21.5,79,0,1);
      G01 (24,76.5,0,1);
      G01 (24,13,0,1);
      G01 (26.5,10.5,0,1);
      G01 (47,10.5,0,1);
      G01 (49.5,13,0,1);
      G01 (49.5,76.5,0,1);
      G01 (52,79,0,1);
      G01 (72.5,79,0,1);
      G01 (75,76.5,0,1);
      G01 (75,13,0,1);
      G01 (77.5,10.5,0,1);
      G01 (98,10.5,0,1);
      G01 (100.5,13,0,1);
      G01 (100.5,76.5,0,1);
      G01 (103,79,0,1);
      G01 (120.5,79,0,1);
      G01 (120.5,67,0,1);
      G01 (125.5,67,0,1);
      G01 (125.5,13,0,1);
      G01 (127.5,10.5,0,1);
      G01 (148,10.5,0,1);
      G01 (150.5,13,0,1);
      G01 (150.5,67,0,1);
      G01 (155.5,67,0,1);
      G01 (155.5,79,0,1);
      G01 (173,79,0,1);
      G01 (175.5,76.5,0,1);
      G01 (175.5,13,0,1);
      G01 (178,10.5,0,1);
      G01 (198.5,10.5,0,1);
      G01 (200.5,13,0,1);
      G01 (200.5,76.5,0,1);
      G01 (203,79,0,1);
      G01 (245,79,0,1);
      G01 (247.5,76.5,0,1);
      G01 (247.5,13,0,1);
      G01 (250,10.5,0,1);
      G01 (264.5,10.5,0,1);
      G01 (264.5,1,0,1);
      G01 (267,-1.5,0,1);
      G01 (275,-1.5,0,1);
      G01 (277.5,1,0,1);
      G01 (277.5,10.5,0,1);
      G01 (292,10.5,0,1);
      G01 (294.5,13,0,1);
      G01 (294.5,76.5,0,1);
      G01 (292,79,0,1);
      G01 (250,79,0,1);
      G01 (247.5,76.5,0,1);
      G00 (247.5,13,0);
      G01 (245,10.5,0,1);
      G01 (230.5,10.5,0,1);
      G01 (230.5,1,0,1);
      G01 (228,-1.5,0,1);
      G01 (220,-1.5,0,1);
      G01 (217.5,1,0,1);
      G01 (217.5,10.5,0,1);
      G01 (203,10.5,0,1);
      G01 (200.5,13,0,1);
      G00 (200.5,67,0);
      G01 (195.5,67,0,1);
      G01 (195.5,79,0,1);
      G01 (178,79,0,1);
      G01 (175.5,76.5,0,1);
      G00 (175.5,13,0);
      G01 (173,10.5,0,1);
      G01 (152.5,10.5,0,1);
      G01 (150.5,13,0,1);
      G00 (150.5,67,0);
      G01 (150.5,76.5,0,1);
      G01 (148,79,0,1);
      G01 (130.5,79,0,1);
      G01 (130.5,67,0,1);
      G01 (125.5,67,0,1);
      G00 (125.5,13,0);
      G01 (123.5,10.5,0,1);
      G01 (103,10.5,0,1);
      G01 (100.5,13,0,1);
      G00 (100.5,76.5,0);
      G01 (98,79,0,1);
      G01 (77.5,79,0,1);
      G01 (75,76.5,0,1);
      G00 (75,13,0);
      G01 (72.5,10.5,0,1);
      G01 (52,10.5,0,1);
      G01 (49.5,13,0,1);
      G00 (49.5,76.5,0);
      G01 (47,79,0,1);
      G01 (26.5,79,0,1);
      G01 (24,76.5,0,1);
      G00 (24,13,0);
      G01 (21.5,10.5,0,1);
      G01 (1,10.5,0,1);
      G01 (-1.5,13,0,1);
      G00 (-1.5,-4,0);
      count ++;
      lcd1.setCursor(14, 3);
      lcd1.print(count);
      while (KeyPadRetrieve() < 12){
      }
      while (KeyPadRetrieve() != 11){
        if (KeyPadRetrieve() == 10){
          count = 23;
        }
      }
    }
  }

  if (menuSelect == 5 && KeyPadRetrieve() == 5) { // Flatten Off Rear Tab
    count = 1;
    int millSpeed = 1;
    int transitSpeed = 15;
    while (KeyPadRetrieve() < 12){
    }
    LCD2_UpdateDelay(1);
    lcd1.setCursor(0, 0);
    lcd1.print("  FIT 6mm END MILL  ");
    lcd1.setCursor(0, 1);
    lcd1.print("    & TOUCH OFF     ");
    lcd1.setCursor(0, 2);
    lcd1.print(" Press # to Move to ");
    lcd1.setCursor(0, 3);
    lcd1.print("       START        ");
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    G00 (218,-7,0);
    lcd1.setCursor(0, 0);
    lcd1.print("  FLATTEN REAR TAB  ");
    lcd1.setCursor(0, 1);
    lcd1.print("   5 Passes at 1mm  ");
    lcd1.setCursor(0, 2);
    lcd1.print("    Press # to RUN  ");
    lcd1.setCursor(0, 3);
    lcd1.print("     Pass No.    *Qt");
    lcd1.setCursor(14, 3);
    lcd1.print(count);
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    while (count < 6){
      G01 (218,9,0,0.5);
      G01 (230,9,0,0.5);
      G01 (230,4,0,0.5);
      G01 (218,4,0,0.5);
      G01 (218,-1,0,0.5);
      G01 (230,-1,0,0.5);
      G01 (230,9,0,0.5);
      G01 (230,-7,0,5);
      G01 (218,-7,0,5);
      count ++;
      lcd1.setCursor(14, 3);
      lcd1.print(count);
      while (KeyPadRetrieve() < 12){
      }
      while (KeyPadRetrieve() != 11){
        if (KeyPadRetrieve() == 10){
          count = 6;
        }
      }
    }
  }

  if (menuSelect == 5 && KeyPadRetrieve() == 6) { // Drilled Holes & CounterBore
    while (KeyPadRetrieve() < 12){
    }
    LCD2_UpdateDelay(1);
    lcd1.setCursor(0, 0);
    lcd1.print("FIT 3mm DRILL BIT & ");
    lcd1.setCursor(0, 1);
    lcd1.print("SET DEPTH ABOVE BASE");
    lcd1.setCursor(0, 2);
    lcd1.print(" Press # to Move to ");
    lcd1.setCursor(0, 3);
    lcd1.print("      HOLE 1    *:Qt");
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
      G00 (31.5,18,0);
      lcd1.setCursor(11, 3);
      lcd1.print("2");
      while (KeyPadRetrieve() < 12){
      }
      while (KeyPadRetrieve() != 11){
      }
      G00 (42,71.5,0);
      lcd1.setCursor(11, 3);
      lcd1.print("3");
      while (KeyPadRetrieve() < 12){
      }
      while (KeyPadRetrieve() != 11){
      }
      G00 (82.5,71.5,0);
      lcd1.setCursor(11, 3);
      lcd1.print("4");
      while (KeyPadRetrieve() < 12){
      }
      while (KeyPadRetrieve() != 11){
      }
      G00 (93,18,0);
      lcd1.setCursor(11, 3);
      lcd1.print("5");
      while (KeyPadRetrieve() < 12){
      }
      while (KeyPadRetrieve() != 11){
      }
      G00 (132.5,18,0);
      lcd1.setCursor(11, 3);
      lcd1.print("6");
      while (KeyPadRetrieve() < 12){
      }
      while (KeyPadRetrieve() != 11){
      }
      G00 (143,71.5,0);
      lcd1.setCursor(11, 3);
      lcd1.print("7");
      while (KeyPadRetrieve() < 12){
      }
      while (KeyPadRetrieve() != 11){
      }
      G00 (183,71.5,0);
      lcd1.setCursor(11, 3);
      lcd1.print("8");
      while (KeyPadRetrieve() < 12){
      }
      while (KeyPadRetrieve() != 11){
      }
      G00 (193.5,18,0);
      lcd1.setCursor(11, 3);
      lcd1.print("9");
      while (KeyPadRetrieve() < 12){
      }
      while (KeyPadRetrieve() != 11){
      }
      G00 (213.2,23.2,0);
      lcd1.setCursor(11, 3);
      lcd1.print("10");
      while (KeyPadRetrieve() < 12){
      }
      while (KeyPadRetrieve() != 11){
      }
      G00 (224,60.9,0);
      lcd1.setCursor(11, 3);
      lcd1.print("11");
      while (KeyPadRetrieve() < 12){
      }
      while (KeyPadRetrieve() != 11){
      }
      G00 (234.8,23.2,0);
      lcd1.setCursor(11, 3);
      lcd1.print("END");
      while (KeyPadRetrieve() < 12){
      }
      while (KeyPadRetrieve() != 11){
      }   
      lcd1.setCursor(0, 0);
      lcd1.print("FIT 6mm END MILL &  ");
      lcd1.setCursor(0, 1);
      lcd1.print("COUNTER BORE TO 4mm ");
      lcd1.setCursor(0, 2);
      lcd1.print("THEN, COUNTER SINK  ");
      lcd1.setCursor(0, 3);
      lcd1.print("     EACH HOLE      ");
      while (KeyPadRetrieve() < 12){
      }
      while (KeyPadRetrieve() != 11){
      }   
  }

  if (menuSelect == 5 && KeyPadRetrieve() == 7) { // Pixel Feature
    count = 1;
    float millSpeed = 0.5;
    float transitSpeed = 0.5;
    while (KeyPadRetrieve() < 12){
    }
    LCD2_UpdateDelay(1);
    lcd1.setCursor(0, 0);
    lcd1.print("FIT 6mm 45dg CHAMFER");
    lcd1.setCursor(0, 1);
    lcd1.print("                    ");
    lcd1.setCursor(0, 2);
    lcd1.print(" Press # to Move to ");
    lcd1.setCursor(0, 3);
    lcd1.print("       START        ");
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    G00 (-1.5,-4,0);
    lcd1.setCursor(0, 0);
    lcd1.print("  PIXEL FEATURE,TOP ");
    lcd1.setCursor(0, 1);
    lcd1.print("  4 Passes at 0.1mm ");
    lcd1.setCursor(0, 2);
    lcd1.print("    Press # to RUN  ");
    lcd1.setCursor(0, 3);
    lcd1.print("     Pass No.    *Qt");
    lcd1.setCursor(14, 3);
    lcd1.print(count);
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    while (count < 4){
      G01 (-1.5,17.88,0,transitSpeed);
      G01 (294.5,17.88,0,millSpeed);
      G01 (294.5,23.255,0,transitSpeed);
      G01 (-1.5,23.255,0,millSpeed);
      G01 (-1.5,28.63,0,transitSpeed);
      G01 (294.5,28.63,0,millSpeed);
      G01 (294.5,34.005,0,transitSpeed);
      G01 (-1.5,34.005,0,millSpeed);
      G01 (-1.5,39.38,0,transitSpeed);
      G01 (294.5,39.38,0,millSpeed);
      G01 (294.5,44.755,0,transitSpeed);
      G01 (-1.5,44.755,0,millSpeed);
      G01 (-1.5,50.13,0,transitSpeed);
      G01 (294.5,50.13,0,millSpeed);
      G01 (294.5,55.505,0,transitSpeed);
      G01 (-1.5,55.505,0,millSpeed);
      G01 (-1.5,60.88,0,transitSpeed);
      G01 (294.5,60.88,0,millSpeed);
      G01 (294.5,66.255,0,transitSpeed);
      G01 (-1.5,66.255,0,millSpeed);
      G01 (-1.5,71.63,0,transitSpeed);
      G01 (294.5,71.63,0,millSpeed);
      G01 (294.5,80,0,millSpeed);
      G00 (5.86,80,0);
      G01 (5.86,10.5,0,millSpeed);
      G01 (11.25,10.5,0,transitSpeed);
      G01 (11.25,80,0,millSpeed);
      G01 (16.63,80,0,transitSpeed);
      G01 (16.63,10.5,0,millSpeed);
      G01 (31.36,10.5,0,transitSpeed);
      G01 (31.36,80,0,millSpeed);
      G01 (36.75,80,0,transitSpeed);
      G01 (36.75,10.5,0,millSpeed);
      G01 (42.13,10.5,0,transitSpeed);
      G01 (42.13,80,0,millSpeed);
      G01 (56.86,80,0,transitSpeed);
      G01 (56.86,10.5,0,millSpeed);
      G01 (62.25,10.5,0,transitSpeed);
      G01 (62.25,80,0,millSpeed);
      G01 (67.63,80,0,transitSpeed);
      G01 (67.63,10.5,0,millSpeed);
      G01 (82.36,10.5,0,transitSpeed);
      G01 (82.36,80,0,millSpeed);
      G01 (87.75,80,0,transitSpeed);
      G01 (87.75,10.5,0,millSpeed);
      G01 (93.13,10.5,0,transitSpeed);
      G01 (93.13,80,0,millSpeed);
      G01 (107.36,80,0,transitSpeed);
      G01 (107.36,10.5,0,millSpeed);
      G01 (113.25,10.5,0,transitSpeed);
      G01 (113.25,80,0,millSpeed);
      G01 (118.63,80,0,transitSpeed);
      G01 (118.63,10.5,0,millSpeed);
      G01 (132.36,10.5,0,transitSpeed);
      G01 (132.36,80,0,millSpeed);
      G01 (137.75,80,0,transitSpeed);
      G01 (137.75,10.5,0,millSpeed);
      G01 (143.13,10.5,0,transitSpeed);
      G01 (143.13,80,0,millSpeed);
      G01 (157.36,80,0,transitSpeed);
      G01 (157.36,10.5,0,millSpeed);
      G01 (162.75,10.5,0,transitSpeed);
      G01 (162.75,80,0,millSpeed);
      G01 (168.13,80,0,transitSpeed);
      G01 (168.13,10.5,0,millSpeed);
      G01 (182.86,10.5,0,transitSpeed);
      G01 (182.86,80,0,millSpeed);
      G01 (188.25,80,0,transitSpeed);
      G01 (188.25,10.5,0,millSpeed);
      G01 (193.63,10.5,0,transitSpeed);
      G01 (193.63,80,0,millSpeed);
      G01 (207.88,80,0,transitSpeed);
      G01 (207.88,10.5,0,millSpeed);
      G01 (213.25,10.5,0,transitSpeed);
      G01 (213.25,80,0,millSpeed);
      G01 (218.63,80,0,transitSpeed);
      G01 (218.63,10.5,0,millSpeed);
      G01 (224,10.5,0,transitSpeed);
      G01 (224,85,0,millSpeed);
      G01 (229.38,80,0,transitSpeed);
      G01 (229.38,10.5,0,millSpeed);
      G01 (218.63,10.5,0,millSpeed);
      G01 (234.75,10.5,0,transitSpeed);
      G01 (234.75,80,0,millSpeed);
      G01 (240.13,80,0,transitSpeed);
      G01 (240.13,10.5,0,millSpeed);
      G01 (254.88,10.5,0,transitSpeed);
      G01 (254.88,80,0,millSpeed);
      G01 (260.25,80,0,transitSpeed);
      G01 (260.25,10.5,0,millSpeed);
      G01 (265.63,10.5,0,transitSpeed);
      G01 (265.63,80,0,millSpeed);
      G01 (271,80,0,transitSpeed);
      G01 (271,0,0,millSpeed);
      G01 (271,10.5,0,transitSpeed);
      G01 (276.38,10.5,0,transitSpeed);
      G01 (276.38,80,0,millSpeed);
      G01 (281.75,80,0,transitSpeed);
      G01 (281.75,10.5,0,millSpeed);
      G01 (287.13,10.5,0,transitSpeed);
      G01 (287.13,80,0,millSpeed);
      G01 (294.5,80,0,transitSpeed);
      G01 (294.5,12.5,0,transitSpeed);
      G01 (249,12.5,0,millSpeed);
      G01 (249,10.5,0,millSpeed);
      G01 (264.5,10.5,0,transitSpeed);
      G01 (264.5,7.13,0,transitSpeed);
      G01 (277.5,7.13,0,millSpeed);
      G01 (277.5,1.75,0,transitSpeed);
      G01 (264.5,1.75,0,millSpeed);
      G01 (264.5,10.5,0,transitSpeed);
      G01 (245,10.5,0,transitSpeed);
      G01 (247.5,13,0,transitSpeed);
      G01 (247.5,80,0,transitSpeed);
      G01 (1,80,0,transitSpeed);
      G01 (1,79,0,transitSpeed);
      G01 (-1.5,76.5,0,transitSpeed);
      G01 (-1.5,-4,0,transitSpeed);
      count ++;
      lcd1.setCursor(14, 3);
      lcd1.print(count);
      while (KeyPadRetrieve() < 12){
      }
      while (KeyPadRetrieve() != 11){
        if (KeyPadRetrieve() == 10){
          count = 4;
        }
      }
    }
  }
*/

  if (menuSelect == 5 && KeyPadRetrieve() == 8) {
    while (KeyPadRetrieve() < 12){
    }
    lCD2_UpdatePause = false;
    menuSelect = 6;
  }

  if (menuSelect == 5 && KeyPadRetrieve() == 10) {
    while (KeyPadRetrieve() < 12){
    }
    lCD2_UpdatePause = false;
    menuSelect = 3;
  }

  
//______________________________MENU 6__________________________________
  
  if (menuSelect == 6) {
    lCD2_UpdatePause = true;
    lcd1.setCursor(0, 0);
    lcd1.print("1:Internal Recesses ");
    lcd1.setCursor(0, 1);
    lcd1.print("2:Blind M3 holes    ");
    lcd1.setCursor(0, 2);
    lcd1.print("3:Individual FaceOff");
    lcd1.setCursor(0, 3);
    lcd1.print("4:Pixel Long Edge   ");
    lcd2.setCursor(0, 0);
    lcd2.print("5:Pixel Medium Edge ");
    lcd2.setCursor(0, 1);
    lcd2.print("6:Pixel Short Edge  ");
    lcd2.setCursor(0, 2);
    lcd2.print("7:Chamfer Remove    ");
    lcd2.setCursor(0, 3);
    lcd2.print("8:Head Fbrctn *:Back");
  }

  if (menuSelect == 6 && KeyPadRetrieve() == 1) {
    while (KeyPadRetrieve() < 12){
    }
    lCD2_UpdatePause = false;
    menuSelect = 7;
  }

    if (menuSelect == 6 && KeyPadRetrieve() == 8) {
    while (KeyPadRetrieve() < 12){
    }
    lCD2_UpdatePause = false;
    menuSelect = 8;
  }

/*
  if (menuSelect == 6 && KeyPadRetrieve() == 2) { // 2.5mm Drilled Blind Holes
    while (KeyPadRetrieve() < 12){
    }
    LCD2_UpdateDelay(1);
    lcd1.setCursor(0, 0);
    lcd1.print("FIT 2.5mm DRILL BIT ");
    lcd1.setCursor(0, 1);
    lcd1.print("   & DRILL TO 10mm  ");
    lcd1.setCursor(0, 2);
    lcd1.print(" Press # to Move to ");
    lcd1.setCursor(0, 3);
    lcd1.print("      HOLE 1    *:Qt");
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
      G00 (6,-71.5,0);
      lcd1.setCursor(11, 3);
      lcd1.print("2");
      while (KeyPadRetrieve() < 12){
      }
      while (KeyPadRetrieve() != 11){
      }
      G00 (16.5,-18,0);
      lcd1.setCursor(11, 3);
      lcd1.print("3");
      while (KeyPadRetrieve() < 12){
      }
      while (KeyPadRetrieve() != 11){
      }
      G00 (57,-18,0);
      lcd1.setCursor(11, 3);
      lcd1.print("4");
      while (KeyPadRetrieve() < 12){
      }
      while (KeyPadRetrieve() != 11){
      }
      G00 (67.5,-71.5,0);
      lcd1.setCursor(11, 3);
      lcd1.print("5");
      while (KeyPadRetrieve() < 12){
      }
      while (KeyPadRetrieve() != 11){
      }
      G00 (108,-71.5,0);
      lcd1.setCursor(11, 3);
      lcd1.print("6");
      while (KeyPadRetrieve() < 12){
      }
      while (KeyPadRetrieve() != 11){
      }
      G00 (118.5,-18,0);
      lcd1.setCursor(11, 3);
      lcd1.print("7");
      while (KeyPadRetrieve() < 12){
      }
      while (KeyPadRetrieve() != 11){
      }
      G00 (157.5,-18,0);
      lcd1.setCursor(11, 3);
      lcd1.print("8");
      while (KeyPadRetrieve() < 12){
      }
      while (KeyPadRetrieve() != 11){
      }
      G00 (168,-71.5,0);
      lcd1.setCursor(11, 3);
      lcd1.print("9");
      while (KeyPadRetrieve() < 12){
      }
      while (KeyPadRetrieve() != 11){
      }
      G00 (260.2,-23.2,0);
      lcd1.setCursor(11, 3);
      lcd1.print("10");
      while (KeyPadRetrieve() < 12){
      }
      while (KeyPadRetrieve() != 11){
      }
      G00 (271,-60.9,0);
      lcd1.setCursor(11, 3);
      lcd1.print("11");
      while (KeyPadRetrieve() < 12){
      }
      while (KeyPadRetrieve() != 11){
      }
      G00 (281.8,-23.2,0);
      lcd1.setCursor(11, 3);
      lcd1.print("END");
      while (KeyPadRetrieve() < 12){
      }
      while (KeyPadRetrieve() != 11){
      }
  }
    if (menuSelect == 6 && KeyPadRetrieve() == 3) { // Individual Face Off, remove top 0.25mm of blank 
    while (KeyPadRetrieve() < 12){
    }
    LCD2_UpdateDelay(1);
    lcd1.setCursor(0, 0);
    lcd1.print("FIT 40mm SHELL MILL ");
    lcd1.setCursor(0, 1);
    lcd1.print("                    ");
    lcd1.setCursor(0, 2);
    lcd1.print(" Press # to Move to ");
    lcd1.setCursor(0, 3);
    lcd1.print("       START        ");
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    G00 (0,-18,0);
    lcd1.setCursor(0, 0);
    lcd1.print("   FACE OFF EXCESS  ");
    lcd1.setCursor(0, 1);
    lcd1.print("                    ");
    lcd1.setCursor(0, 2);
    lcd1.print("     Press # to     ");
    lcd1.setCursor(0, 3);
    lcd1.print("        RUN         ");
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    G01 (125,-18,0,1.5);
    G00 (125,18,0);
    G01 (0,18,0,1.5);
    G00 (0,-18,0);
  }
  
  if (menuSelect == 6 && KeyPadRetrieve() == 4) { // Pixel Feature Long Edge 
    while (KeyPadRetrieve() < 12){
    }
    LCD2_UpdateDelay(1);
    lcd1.setCursor(0, 0);
    lcd1.print("FIT SMALL SPOT DRILL");
    lcd1.setCursor(0, 1);
    lcd1.print("                    ");
    lcd1.setCursor(0, 2);
    lcd1.print(" Press # to Move to ");
    lcd1.setCursor(0, 3);
    lcd1.print("       START        ");
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    G00 (-4,25,0);
    lcd1.setCursor(0, 0);
    lcd1.print("PIXEL FEAT LONG EDGE");
    lcd1.setCursor(0, 1);
    lcd1.print("                    ");
    lcd1.setCursor(0, 2);
    lcd1.print("     Press # to     ");
    lcd1.setCursor(0, 3);
    lcd1.print("        RUN         ");
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    G00 (5.88,25,0);
    G01 (5.88,-4,0,1);
    G00 (11.25,-4,0);
    G01 (11.25,25,0,1);
    G00 (16.63,25,0);
    G01 (16.63,-4,0,1);
    G00 (22,-4,0);
    G01 (22,25,0,1);
    G00 (27.38,25,0);
    G01 (27.38,-4,0,1);
    G00 (32.75,-4,0);
    G01 (32.75,25,0,1);
    G00 (38.13,25,0);
    G01 (38.13,-4,0,1);
    G00 (43.5,-4,0);
    G01 (43.5,25,0,1);
    G00 (48.88,25,0);
    G01 (48.88,-4,0,1);
    G00 (54.25,-4,0);
    G01 (54.25,25,0,1);
    G00 (59.63,25,0);
    G01 (59.63,-4,0,1);
    G00 (70,-4,0);
    G00 (70,5.88,0);
    G01 (-4,5.88,0,1);
    G00 (-4,11,0);
    G01 (70,11,0,1);
    G00 (70,16.13,0);
    G01 (-4,16.13,0,1);  
    G00 (-4,-4,0);
    G00 (-4,25,0);    
  }

  if (menuSelect == 6 && KeyPadRetrieve() == 5) { // Pixel Feature Medium Edge 
    while (KeyPadRetrieve() < 12){
    }
    LCD2_UpdateDelay(1);
    lcd1.setCursor(0, 0);
    lcd1.print("FIT SMALL SPOT DRILL");
    lcd1.setCursor(0, 1);
    lcd1.print("                    ");
    lcd1.setCursor(0, 2);
    lcd1.print(" Press # to Move to ");
    lcd1.setCursor(0, 3);
    lcd1.print("       START        ");
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    G00 (-4,25,0);
    lcd1.setCursor(0, 0);
    lcd1.print("PIXEL FEAT MEDI EDGE");
    lcd1.setCursor(0, 1);
    lcd1.print("                    ");
    lcd1.setCursor(0, 2);
    lcd1.print("     Press # to     ");
    lcd1.setCursor(0, 3);
    lcd1.print("        RUN         ");
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    G00 (5.88,25,0);
    G01 (5.88,-4,0,1);
    G00 (11.25,-4,0);
    G01 (11.25,25,0,1);
    G00 (16.63,25,0);
    G01 (16.63,-4,0,1);
    G00 (22,-4,0);
    G01 (22,25,0,1);
    G00 (27.38,25,0);
    G01 (27.38,-4,0,1);
    G00 (32.75,-4,0);
    G01 (32.75,25,0,1);
    G00 (38.13,25,0);
    G01 (38.13,-4,0,1);
    G00 (50,-4,0);
    G00 (50,5.88,0);
    G01 (-4,5.88,0,1);
    G00 (-4,11,0);
    G01 (50,11,0,1);
    G00 (50,16.13,0);
    G01 (-4,16.13,0,1);  
    G00 (-4,-4,0);
    G00 (-4,25,0);    
  }

  if (menuSelect == 6 && KeyPadRetrieve() == 6) { // Pixel Feature Short Edge 
    while (KeyPadRetrieve() < 12){
    }
    LCD2_UpdateDelay(1);
    lcd1.setCursor(0, 0);
    lcd1.print("FIT SMALL SPOT DRILL");
    lcd1.setCursor(0, 1);
    lcd1.print("                    ");
    lcd1.setCursor(0, 2);
    lcd1.print(" Press # to Move to ");
    lcd1.setCursor(0, 3);
    lcd1.print("       START        ");
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    G00 (-4,25,0);
    lcd1.setCursor(0, 0);
    lcd1.print("PIXEL FEAT SHRT EDGE");
    lcd1.setCursor(0, 1);
    lcd1.print("                    ");
    lcd1.setCursor(0, 2);
    lcd1.print("     Press # to     ");
    lcd1.setCursor(0, 3);
    lcd1.print("        RUN         ");
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    G00 (5.88,25,0);
    G01 (5.88,-4,0,1);
    G00 (11.25,-4,0);
    G01 (11.25,25,0,1);
    G00 (16.63,25,0);
    G01 (16.63,-4,0,1);
    G00 (25,-4,0);
    G00 (25,5.88,0);
    G01 (-4,5.88,0,1);
    G00 (-4,11,0);
    G01 (25,11,0,1);
    G00 (25,16.13,0);
    G01 (-4,16.13,0,1);      
    G00 (-4,-4,0);
    G00 (-4,25,0);    
  }

  if (menuSelect == 6 && KeyPadRetrieve() == 7) { // Chamfer Remove from Zeroed X 
    while (KeyPadRetrieve() < 12){
    }
    LCD2_UpdateDelay(1);
    lcd1.setCursor(0, 0);
    lcd1.print("  FIT 6mm END MILL  ");
    lcd1.setCursor(0, 1);
    lcd1.print("                    ");
    lcd1.setCursor(0, 2);
    lcd1.print(" Press # to Move to ");
    lcd1.setCursor(0, 3);
    lcd1.print("       START        ");
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    G00 (0,0,0);
    lcd1.setCursor(0, 0);
    lcd1.print("   CHAMFER REMOVE   ");
    lcd1.setCursor(0, 1);
    lcd1.print("                    ");
    lcd1.setCursor(0, 2);
    lcd1.print("     Press # to     ");
    lcd1.setCursor(0, 3);
    lcd1.print("        RUN         ");
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    G01 (0,30,0,0.5);
    G00 (0,0,0);
  }
 */
  
  if (menuSelect == 6 && KeyPadRetrieve() == 10) {
    while (KeyPadRetrieve() < 12){
    }
    lCD2_UpdatePause = false;
    menuSelect = 5;
  }

//______________________________MENU 7__________________________________
  
  if (menuSelect == 7) {
    lCD2_UpdatePause = true;
    lcd1.setCursor(0, 0);
    lcd1.print("1:Right Arm         ");
    lcd1.setCursor(0, 1);
    lcd1.print("2:Left Arm          ");
    lcd1.setCursor(0, 2);
    lcd1.print("3:Right Leg         ");
    lcd1.setCursor(0, 3);
    lcd1.print("4:Left Leg          ");
    lcd2.setCursor(0, 0);
    lcd2.print("5:Body Trunk        ");
    lcd2.setCursor(0, 1);
    lcd2.print("6:Magnet Slot 6mm   ");
    lcd2.setCursor(0, 2);
    lcd2.print("7:Magnet Slot 5.1mm ");
    lcd2.setCursor(0, 3);
    lcd2.print("8:Mgn Slt 9mm *:Back");
  }

/*

  if (menuSelect == 7 && KeyPadRetrieve() == 1) { // Internal Recesses, Right Arm
    count = 1;
    while (KeyPadRetrieve() < 12){
    }
    LCD2_UpdateDelay(1);
    lcd1.setCursor(0, 0);
    lcd1.print("  FIT 6mm END MILL  ");
    lcd1.setCursor(0, 1);
    lcd1.print("    & TOUCH OFF     ");
    lcd1.setCursor(0, 2);
    lcd1.print(" Press # to Move to ");
    lcd1.setCursor(0, 3);
    lcd1.print("    START 1 of 18   ");
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    G00 (6,-18,0);
    lcd1.setCursor(0, 0);
    lcd1.print(" LOWER CUT BY 0.5mm ");
    lcd1.setCursor(0, 1);
    lcd1.print("   DURING MOVEMENT  ");
    lcd1.setCursor(0, 2);
    lcd1.print("    Press # to RUN  ");
    lcd1.setCursor(0, 3);
    lcd1.print("     Pass No.    *Qt");
    lcd1.setCursor(14, 3);
    lcd1.print(count);
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    while (count < 19){
      G01 (6,-62.5,0,1);
      G01 (10.3,-62.5,0,1);
      G01 (10.3,-27,0,1); //
      G01 (10.3,-60.5,0,1);
      G02 (16.3,-54.5,0,16.3,-60.5,0,1);
      G01 (16.5,-54.5,0,1);
      G01 (16.5,-27,0,1);
      G01 (13.5,-54.5,0,1); //
      G01 (13.5,-27,0,1);
      G01 (16.5,-27,0,1);
      G01 (13.5,-27,0,1);
      G02 (7.5,-21,0,13.5,-21,0,1);
      G01 (7.5,-18,0,1);
      G01 (6,-18,0,1);
      G01 (6,-62.5,0,1);
      G00 (6,-18,0);
      count ++;
      if (count < 19){
        lcd1.setCursor(0, 0);
        lcd1.print(" LOWER CUT BY 0.5mm ");
        lcd1.setCursor(0, 1);
        lcd1.print("   DURING MOVEMENT  ");
        lcd1.setCursor(0, 2);
        lcd1.print("    Press # to RUN  ");
        lcd1.setCursor(0, 3);
        lcd1.print("     Pass No.    *Qt");
        lcd1.setCursor(14, 3);
        lcd1.print(count);
      }else{        
        lcd1.setCursor(14, 3);
        lcd1.print("END");
      }
      while (KeyPadRetrieve() < 12){
      }
      while (KeyPadRetrieve() != 11){
        if (KeyPadRetrieve() == 10){
          count = 19;
        }
      }
    }

    count = 1;
    
    LCD2_UpdateDelay(1);
    lcd1.setCursor(0, 0);
    lcd1.print(" LIFT CUTTER FULLY  ");
    lcd1.setCursor(0, 1);
    lcd1.print("   FOR NEXT PART    ");
    lcd1.setCursor(0, 2);
    lcd1.print(" Press # to Move to ");
    lcd1.setCursor(0, 3);
    lcd1.print("    START 1 of 18   ");
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    G00 (42,-18,0);
    lcd1.setCursor(0, 0);
    lcd1.print(" LOWER CUT BY 0.5mm ");
    lcd1.setCursor(0, 1);
    lcd1.print("   DURING MOVEMENT  ");
    lcd1.setCursor(0, 2);
    lcd1.print("    Press # to RUN  ");
    lcd1.setCursor(0, 3);
    lcd1.print("     Pass No.    *Qt");
    lcd1.setCursor(14, 3);
    lcd1.print(count);
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    while (count < 19){
      G01 (42,-62.5,0,1);
      G01 (37.7,-62.5,0,1);
      G01 (37.7,-27,0,1); //
      G01 (37.7,-60.5,0,1);
      G03 (31.7,-54.5,0,31.7,-60.5,0,1);
      G01 (31.5,-54.5,0,1);
      G01 (31.5,-27,0,1);
      G01 (34.5,-54.5,0,1); //
      G01 (34.5,-27,0,1);
      G01 (31.5,-27,0,1);
      G01 (34.5,-27,0,1);
      G03 (40.5,-21,0,34.5,-21,0,1);
      G01 (40.5,-18,0,1);
      G01 (42,-18,0,1);
      G01 (42,-62.5,0,1);
      G00 (42,-18,0);
      count ++;
      if (count < 19){
        lcd1.setCursor(0, 0);
        lcd1.print(" LOWER CUT BY 0.5mm ");
        lcd1.setCursor(0, 1);
        lcd1.print("   DURING MOVEMENT  ");
        lcd1.setCursor(0, 2);
        lcd1.print("    Press # to RUN  ");
        lcd1.setCursor(0, 3);
        lcd1.print("     Pass No.    *Qt");
        lcd1.setCursor(14, 3);
        lcd1.print(count);
      }else{        
        lcd1.setCursor(14, 3);
        lcd1.print("END");
      }
      while (KeyPadRetrieve() < 12){
      }
      while (KeyPadRetrieve() != 11){
        if (KeyPadRetrieve() == 10){
          count = 19;
        }
      }
    }
  }

if (menuSelect == 7 && KeyPadRetrieve() == 2) { // Internal Recesses, Left Arm
    count = 1;
    while (KeyPadRetrieve() < 12){
    }
    LCD2_UpdateDelay(1);
    lcd1.setCursor(0, 0);
    lcd1.print("      LEFT ARM      ");
    lcd1.setCursor(0, 1);
    lcd1.print("    & TOUCH OFF     ");
    lcd1.setCursor(0, 2);
    lcd1.print(" Press # to Move to ");
    lcd1.setCursor(0, 3);
    lcd1.print("    START 1 of 18   ");
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    G00 (67.5,-18,0);
    lcd1.setCursor(0, 0);
    lcd1.print(" LOWER CUT BY 0.5mm ");
    lcd1.setCursor(0, 1);
    lcd1.print("   DURING MOVEMENT  ");
    lcd1.setCursor(0, 2);
    lcd1.print("    Press # to RUN  ");
    lcd1.setCursor(0, 3);
    lcd1.print("     Pass No.    *Qt");
    lcd1.setCursor(14, 3);
    lcd1.print(count);
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    while (count < 19){
      G01 (67.5,-62.5,0,1);
      G01 (63.2,-62.5,0,1);
      G01 (63.2,-27,0,1); //
      G01 (63.2,-60.5,0,1);
      G03 (57.2,-54.5,0,57.2,-60.5,0,1);
      G01 (57,-54.5,0,1);
      G01 (57,-27,0,1);
      G01 (60,-54.5,0,1); //
      G01 (60,-27,0,1);
      G01 (57,-27,0,1);
      G01 (60,-27,0,1);
      G03 (66,-21,0,60,-21,0,1);
      G01 (66,-18,0,1);
      G01 (67.5,-18,0,1);
      G01 (67.5,-62.5,0,1);
      G00 (67.5,-18,0);
      count ++;
      if (count < 19){  
        lcd1.setCursor(0, 0);
        lcd1.print(" LOWER CUT BY 0.5mm ");
        lcd1.setCursor(0, 1);
        lcd1.print("   DURING MOVEMENT  ");
        lcd1.setCursor(0, 2);
        lcd1.print("    Press # to RUN  ");
        lcd1.setCursor(0, 3);
        lcd1.print("     Pass No.    *Qt");
        lcd1.setCursor(14, 3);
        lcd1.print(count);
      }else{        
        lcd1.setCursor(14, 3);
        lcd1.print("END");
      }
      while (KeyPadRetrieve() < 12){
      }
      while (KeyPadRetrieve() != 11){
        if (KeyPadRetrieve() == 10){
          count = 19;
        }
      }
    }

    count = 1;
    
    LCD2_UpdateDelay(1);
    lcd1.setCursor(0, 0);
    lcd1.print(" LIFT CUTTER FULLY  ");
    lcd1.setCursor(0, 1);
    lcd1.print("   FOR NEXT PART    ");
    lcd1.setCursor(0, 2);
    lcd1.print(" Press # to Move to ");
    lcd1.setCursor(0, 3);
    lcd1.print("    START 1 of 18   ");
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    G00 (82.5,-18,0);
    lcd1.setCursor(0, 0);
    lcd1.print(" LOWER CUT BY 0.5mm ");
    lcd1.setCursor(0, 1);
    lcd1.print("   DURING MOVEMENT  ");
    lcd1.setCursor(0, 2);
    lcd1.print("    Press # to RUN  ");
    lcd1.setCursor(0, 3);
    lcd1.print("     Pass No.    *Qt");
    lcd1.setCursor(14, 3);
    lcd1.print(count);
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    while (count < 19){
      G01 (82.5,-62.5,0,1);
      G01 (86.8,-62.5,0,1);
      G01 (86.8,-27,0,1); //
      G01 (86.8,-60.5,0,1);
      G02 (92.8,-54.5,0,92.8,-60.5,0,1);
      G01 (93,-54.5,0,1);
      G01 (93,-27,0,1);
      G01 (90,-54.5,0,1); //
      G01 (90,-27,0,1);
      G01 (93,-27,0,1);
      G01 (90,-27,0,1);
      G02 (84,-21,0,90,-21,0,1);
      G01 (84,-18,0,1);
      G01 (82.5,-18,0,1);
      G01 (82.5,-62.5,0,1);
      G00 (82.5,-18,0);
      count ++;
      if (count < 19){
        lcd1.setCursor(0, 0);
        lcd1.print(" LOWER CUT BY 0.5mm ");
        lcd1.setCursor(0, 1);
        lcd1.print("   DURING MOVEMENT  ");
        lcd1.setCursor(0, 2);
        lcd1.print("    Press # to RUN  ");
        lcd1.setCursor(0, 3);
        lcd1.print("     Pass No.    *Qt");
        lcd1.setCursor(14, 3);
        lcd1.print(count);
      }else{        
        lcd1.setCursor(14, 3);
        lcd1.print("END");
      }
      while (KeyPadRetrieve() < 12){
      }
      while (KeyPadRetrieve() != 11){
        if (KeyPadRetrieve() == 10){
          count = 19;
        }
      }
    }
  }

if (menuSelect == 7 && KeyPadRetrieve() == 3) { // Internal Recesses, Right Leg
    count = 1;
    while (KeyPadRetrieve() < 12){
    }
    LCD2_UpdateDelay(1);
    lcd1.setCursor(0, 0);
    lcd1.print("     RIGHT LEG      ");
    lcd1.setCursor(0, 1);
    lcd1.print("                    ");
    lcd1.setCursor(0, 2);
    lcd1.print(" Press # to Move to ");
    lcd1.setCursor(0, 3);
    lcd1.print("    START 1 of 18   ");
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    G00 (108,-35,0);
    lcd1.setCursor(0, 0);
    lcd1.print(" LOWER CUT BY 0.5mm ");
    lcd1.setCursor(0, 1);
    lcd1.print("   DURING MOVEMENT  ");
    lcd1.setCursor(0, 2);
    lcd1.print("    Press # to RUN  ");
    lcd1.setCursor(0, 3);
    lcd1.print("     Pass No.    *Qt");
    lcd1.setCursor(14, 3);
    lcd1.print(count);
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    while (count < 19){
      G01 (108,-58,0,1);
      G01 (118,-58,0,1);
      G01 (108,-58,0,1);
      G01 (118,-58,0,1);
      G01 (118,-35,0,1);
      G01 (108,-35,0,1);
      G00 (113,-35,0);
      G01 (113,-58,0,1);
      G01 (108,-58,0,1);
      G01 (108,-35,0,1);
      count ++;
      if (count < 19){  
        lcd1.setCursor(0, 0);
        lcd1.print(" LOWER CUT BY 0.5mm ");
        lcd1.setCursor(0, 1);
        lcd1.print("   DURING MOVEMENT  ");
        lcd1.setCursor(0, 2);
        lcd1.print("    Press # to RUN  ");
        lcd1.setCursor(0, 3);
        lcd1.print("     Pass No.    *Qt");
        lcd1.setCursor(14, 3);
        lcd1.print(count);
      }else{        
        lcd1.setCursor(14, 3);
        lcd1.print("END");
      }
      while (KeyPadRetrieve() < 12){
      }
      while (KeyPadRetrieve() != 11){
        if (KeyPadRetrieve() == 10){
          count = 19;
        }
      }
    }

    count = 1;
    
    LCD2_UpdateDelay(1);
    lcd1.setCursor(0, 0);
    lcd1.print(" LIFT CUTTER FULLY  ");
    lcd1.setCursor(0, 1);
    lcd1.print("   FOR NEXT PART    ");
    lcd1.setCursor(0, 2);
    lcd1.print(" Press # to Move to ");
    lcd1.setCursor(0, 3);
    lcd1.print("    START 1 of 18   ");
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    G00 (133,-35,0);
    lcd1.setCursor(0, 0);
    lcd1.print(" LOWER CUT BY 0.5mm ");
    lcd1.setCursor(0, 1);
    lcd1.print("   DURING MOVEMENT  ");
    lcd1.setCursor(0, 2);
    lcd1.print("    Press # to RUN  ");
    lcd1.setCursor(0, 3);
    lcd1.print("     Pass No.    *Qt");
    lcd1.setCursor(14, 3);
    lcd1.print(count);
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    while (count < 19){
      G01 (133,-58,0,1);
      G01 (143,-58,0,1);
      G01 (133,-58,0,1);
      G01 (143,-58,0,1);
      G01 (143,-35,0,1);
      G01 (133,-35,0,1);
      G00 (138,-35,0);
      G01 (138,-58,0,1);
      G01 (133,-58,0,1);
      G01 (133,-35,0,1);
      count ++;
      if (count < 19){
        lcd1.setCursor(0, 0);
        lcd1.print(" LOWER CUT BY 0.5mm ");
        lcd1.setCursor(0, 1);
        lcd1.print("   DURING MOVEMENT  ");
        lcd1.setCursor(0, 2);
        lcd1.print("    Press # to RUN  ");
        lcd1.setCursor(0, 3);
        lcd1.print("     Pass No.    *Qt");
      }else{        
        lcd1.setCursor(14, 3);
        lcd1.print("END");
      }
      while (KeyPadRetrieve() < 12){
      }
      while (KeyPadRetrieve() != 11){
        if (KeyPadRetrieve() == 10){
          count = 19;
        }
      }
    }
  }

  if (menuSelect == 7 && KeyPadRetrieve() == 4) { // Internal Recesses, Left Leg
    count = 1;
    while (KeyPadRetrieve() < 12){
    }
    LCD2_UpdateDelay(1);
    lcd1.setCursor(0, 0);
    lcd1.print("     RIGHT LEG      ");
    lcd1.setCursor(0, 1);
    lcd1.print("                    ");
    lcd1.setCursor(0, 2);
    lcd1.print(" Press # to Move to ");
    lcd1.setCursor(0, 3);
    lcd1.print("    START 1 of 18   ");
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    G00 (158,-35,0);
    lcd1.setCursor(0, 0);
    lcd1.print(" LOWER CUT BY 0.5mm ");
    lcd1.setCursor(0, 1);
    lcd1.print("   DURING MOVEMENT  ");
    lcd1.setCursor(0, 2);
    lcd1.print("    Press # to RUN  ");
    lcd1.setCursor(0, 3);
    lcd1.print("     Pass No.    *Qt");
    lcd1.setCursor(14, 3);
    lcd1.print(count);
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    while (count < 19){
      G01 (158,-58,0,1);
      G01 (168,-58,0,1);
      G01 (158,-58,0,1);
      G01 (168,-58,0,1);
      G01 (168,-35,0,1);
      G01 (158,-35,0,1);
      G00 (163,-35,0);
      G01 (163,-58,0,1);
      G01 (158,-58,0,1);
      G01 (158,-35,0,1);
      count ++;
      if (count < 19){  
        lcd1.setCursor(0, 0);
        lcd1.print(" LOWER CUT BY 0.5mm ");
        lcd1.setCursor(0, 1);
        lcd1.print("   DURING MOVEMENT  ");
        lcd1.setCursor(0, 2);
        lcd1.print("    Press # to RUN  ");
        lcd1.setCursor(0, 3);
        lcd1.print("     Pass No.    *Qt");
        lcd1.setCursor(14, 3);
        lcd1.print(count);
      }else{        
        lcd1.setCursor(14, 3);
        lcd1.print("END");
      }
      while (KeyPadRetrieve() < 12){
      }
      while (KeyPadRetrieve() != 11){
        if (KeyPadRetrieve() == 10){
          count = 19;
        }
      }
    }

    count = 1;
    
    LCD2_UpdateDelay(1);
    lcd1.setCursor(0, 0);
    lcd1.print(" LIFT CUTTER FULLY  ");
    lcd1.setCursor(0, 1);
    lcd1.print("   FOR NEXT PART    ");
    lcd1.setCursor(0, 2);
    lcd1.print(" Press # to Move to ");
    lcd1.setCursor(0, 3);
    lcd1.print("    START 1 of 18   ");
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    G00 (183,-35,0);
    lcd1.setCursor(0, 0);
    lcd1.print(" LOWER CUT BY 0.5mm ");
    lcd1.setCursor(0, 1);
    lcd1.print("   DURING MOVEMENT  ");
    lcd1.setCursor(0, 2);
    lcd1.print("    Press # to RUN  ");
    lcd1.setCursor(0, 3);
    lcd1.print("     Pass No.    *Qt");
    lcd1.setCursor(14, 3);
    lcd1.print(count);
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    while (count < 19){
      G01 (183,-58,0,1);
      G01 (193,-58,0,1);
      G01 (183,-58,0,1);
      G01 (193,-58,0,1);      
      G01 (193,-35,0,1);
      G01 (183,-35,0,1);
      G00 (188,-35,0);
      G01 (188,-58,0,1);
      G01 (183,-58,0,1);
      G01 (183,-35,0,1);
      count ++;
      if (count < 19){
        lcd1.setCursor(0, 0);
        lcd1.print(" LOWER CUT BY 0.5mm ");
        lcd1.setCursor(0, 1);
        lcd1.print("   DURING MOVEMENT  ");
        lcd1.setCursor(0, 2);
        lcd1.print("    Press # to RUN  ");
        lcd1.setCursor(0, 3);
        lcd1.print("     Pass No.    *Qt");
        lcd1.setCursor(14, 3);
        lcd1.print(count);
      }else{        
        lcd1.setCursor(14, 3);
        lcd1.print("END");
      }
      while (KeyPadRetrieve() < 12){
      }
      while (KeyPadRetrieve() != 11){
        if (KeyPadRetrieve() == 10){
          count = 19;
        }
      }
    }
  }

  if (menuSelect == 7 && KeyPadRetrieve() == 5) { // Internal Recesses, Body Trunk
    count = 1;
    while (KeyPadRetrieve() < 12){
    }
    LCD2_UpdateDelay(1);
    lcd1.setCursor(0, 0);
    lcd1.print("  FIT 6mm END MILL  ");
    lcd1.setCursor(0, 1);
    lcd1.print("    & TOUCH OFF     ");
    lcd1.setCursor(0, 2);
    lcd1.print(" Press # to Move to ");
    lcd1.setCursor(0, 3);
    lcd1.print("    START 1 of 18   ");
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    G00 (208,-30,0);
    lcd1.setCursor(0, 0);
    lcd1.print(" LOWER CUT BY 0.5mm ");
    lcd1.setCursor(0, 1);
    lcd1.print("   DURING MOVEMENT  ");
    lcd1.setCursor(0, 2);
    lcd1.print("    Press # to RUN  ");
    lcd1.setCursor(0, 3);
    lcd1.print("     Pass No.    *Qt");
    lcd1.setCursor(14, 3);
    lcd1.print(count);
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    while (count < 19){
      G01 (240,-30,0,1);
      G01 (240,-52,0,1);
      G01 (238.5,-52,0,1);
      G03 (232.5,-58,0,238.5,-58,0,1);
      G01 (232.5,-64,0,1);
      G01 (232.5,-58,0,1);
      G03 (226.5,-52,0,226.5,-58,0,1);
      G01 (221.5,-52,0,1);
      G03 (215.5,-58,0,221.5,-58,0,1);
      G01 (215.5,-64,0,1);
      G01 (215.5,-58,0,1);
      G03 (209.5,-52,0,209.5,-58,0,1);
      G01 (208,-52,0,1);
      G01 (208,-30,0,1);
      G00 (208,-35,0);
      G01 (240,-35,0,1);
      G00 (240,-40,0);
      G01 (208,-40,0,1);
      G00 (208,-45,0);
      G01 (240,-45,0,1);
      G00 (240,-50,0);
      G01 (208,-50,0,1);
      G00 (208,-30,0);      
      count ++;
      if (count < 19){
        lcd1.setCursor(0, 0);
        lcd1.print(" LOWER CUT BY 0.5mm ");
        lcd1.setCursor(0, 1);
        lcd1.print("   DURING MOVEMENT  ");
        lcd1.setCursor(0, 2);
        lcd1.print("    Press # to RUN  ");
        lcd1.setCursor(0, 3);
        lcd1.print("     Pass No.    *Qt");
        lcd1.setCursor(14, 3);
        lcd1.print(count);
      }else{        
        lcd1.setCursor(14, 3);
        lcd1.print("END");
      }
      while (KeyPadRetrieve() < 12){
      }
      while (KeyPadRetrieve() != 11){
        if (KeyPadRetrieve() == 10){
          count = 19;
        }
      }
    }

    count = 1;
    
    LCD2_UpdateDelay(1);
    lcd1.setCursor(0, 0);
    lcd1.print(" LIFT CUTTER FULLY  ");
    lcd1.setCursor(0, 1);
    lcd1.print("   FOR NEXT PART    ");
    lcd1.setCursor(0, 2);
    lcd1.print(" Press # to Move to ");
    lcd1.setCursor(0, 3);
    lcd1.print("    START 1 of 18   ");
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    G00 (255,-30,0);
    lcd1.setCursor(0, 0);
    lcd1.print(" LOWER CUT BY 0.5mm ");
    lcd1.setCursor(0, 1);
    lcd1.print("   DURING MOVEMENT  ");
    lcd1.setCursor(0, 2);
    lcd1.print("    Press # to RUN  ");
    lcd1.setCursor(0, 3);
    lcd1.print("     Pass No.    *Qt");
    lcd1.setCursor(14, 3);
    lcd1.print(count);
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    while (count < 19){
      G01 (287,-30,0,1);
      G01 (287,-52,0,1);
      G01 (285.5,-52,0,1);
      G03 (279.5,-58,0,285.5,-58,0,1);
      G01 (279.5,-64,0,1);
      G01 (279.5,-58,0,1);
      G03 (273.5,-52,0,273.5,-58,0,1);
      G01 (268.5,-52,0,1);
      G03 (262.5,-58,0,268.5,-58,0,1);
      G01 (262.5,-64,0,1);
      G01 (262.5,-58,0,1);
      G03 (256.5,-52,0,256.5,-58,0,1);
      G01 (255,-52,0,1);
      G01 (255,-30,0,1);
      G00 (255,-35,0);
      G01 (287,-35,0,1);
      G00 (287,-40,0);
      G01 (255,-40,0,1);
      G00 (255,-45,0);
      G01 (287,-45,0,1);
      G00 (287,-50,0);
      G01 (255,-50,0,1);
      G00 (255,-30,0);   
      count ++;
      if (count < 19){
        lcd1.setCursor(0, 0);
        lcd1.print(" LOWER CUT BY 0.5mm ");
        lcd1.setCursor(0, 1);
        lcd1.print("   DURING MOVEMENT  ");
        lcd1.setCursor(0, 2);
        lcd1.print("    Press # to RUN  ");
        lcd1.setCursor(0, 3);
        lcd1.print("     Pass No.    *Qt");
        lcd1.setCursor(14, 3);
        lcd1.print(count);
      }else{        
        lcd1.setCursor(14, 3);
        lcd1.print("END");
      }
      while (KeyPadRetrieve() < 12){
      }
      while (KeyPadRetrieve() != 11){
        if (KeyPadRetrieve() == 10){
          count = 19;
        }
      }
    }
  }

  if (menuSelect == 7 && KeyPadRetrieve() == 6) { // Internal Recesses, Magnet Slot 6mm Depth
    count = 1;
    while (KeyPadRetrieve() < 12){
    }
    LCD2_UpdateDelay(1);
    lcd1.setCursor(0, 0);
    lcd1.print("  6mm MAGNET SLOTs  ");
    lcd1.setCursor(0, 1);
    lcd1.print("                    ");
    lcd1.setCursor(0, 2);
    lcd1.print(" Press # to Move to ");
    lcd1.setCursor(0, 3);
    lcd1.print("    START 1 of 28   ");
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    G00 (16.8,-61,0);
    lcd1.setCursor(0, 0);
    lcd1.print(" LOWER CUT BY 0.25mm ");
    lcd1.setCursor(0, 1);
    lcd1.print("   DURING MOVEMENT  ");
    lcd1.setCursor(0, 2);
    lcd1.print("    Press # to RUN  ");
    lcd1.setCursor(0, 3);
    lcd1.print("St1  Pass No.    *Qt");
    lcd1.setCursor(14, 3);
    lcd1.print(count);
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    while (count < 29){
      G01 (16.8,-71,0,0.5);
      G01 (19,-71,0,0.5);
      G01 (19,-61,0,0.5);
      G01 (16.8,-61,0,0.5);
      G01 (16.8,-71,0,0.5);
      G00 (16.8,-61,0);
      count ++;
      if (count < 29){
        lcd1.setCursor(0, 3);
        lcd1.print("St1  Pass No.    *Qt");
        lcd1.setCursor(14, 3);
        lcd1.print(count);
      }else{        
        lcd1.setCursor(14, 3);
        lcd1.print("END");
      }
      while (KeyPadRetrieve() < 12){
      }
      while (KeyPadRetrieve() != 11){
        if (KeyPadRetrieve() == 10){
          count = 29;
        }
      }
    }

    count = 1;
        
    LCD2_UpdateDelay(1);
    lcd1.setCursor(0, 0);
    lcd1.print("  6mm MAGNET SLOTs  ");
    lcd1.setCursor(0, 1);
    lcd1.print("                    ");
    lcd1.setCursor(0, 2);
    lcd1.print(" Press # to Move to ");
    lcd1.setCursor(0, 3);
    lcd1.print("    START 1 of 28   ");
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    G00 (29,-61,0);
    lcd1.setCursor(0, 0);
    lcd1.print(" LOWER CUT BY 0.25mm ");
    lcd1.setCursor(0, 1);
    lcd1.print("   DURING MOVEMENT  ");
    lcd1.setCursor(0, 2);
    lcd1.print("    Press # to RUN  ");
    lcd1.setCursor(0, 3);
    lcd1.print("St2  Pass No.    *Qt");
    lcd1.setCursor(14, 3);
    lcd1.print(count);
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    while (count < 29){
      G01 (29,-71,0,0.5);
      G01 (31.2,-71,0,0.5);
      G01 (31.2,-61,0,0.5);
      G01 (29,-61,0,0.5);
      G01 (29,-71,0,0.5);
      G00 (29,-61,0);
      count ++;
      if (count < 29){
        lcd1.setCursor(0, 3);
        lcd1.print("St2  Pass No.    *Qt");
        lcd1.setCursor(14, 3);
        lcd1.print(count);
      }else{        
        lcd1.setCursor(14, 3);
        lcd1.print("END");
      }
      while (KeyPadRetrieve() < 12){
      }
      while (KeyPadRetrieve() != 11){
        if (KeyPadRetrieve() == 10){
          count = 29;
        }
      }
    }

      count = 1;
        
    LCD2_UpdateDelay(1);
    lcd1.setCursor(0, 0);
    lcd1.print("  6mm MAGNET SLOTs  ");
    lcd1.setCursor(0, 1);
    lcd1.print("                    ");
    lcd1.setCursor(0, 2);
    lcd1.print(" Press # to Move to ");
    lcd1.setCursor(0, 3);
    lcd1.print("    START 1 of 28   ");
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    G00 (54.5,-61,0);
    lcd1.setCursor(0, 0);
    lcd1.print(" LOWER CUT BY 0.25mm ");
    lcd1.setCursor(0, 1);
    lcd1.print("   DURING MOVEMENT  ");
    lcd1.setCursor(0, 2);
    lcd1.print("    Press # to RUN  ");
    lcd1.setCursor(0, 3);
    lcd1.print("St3  Pass No.    *Qt");
    lcd1.setCursor(14, 3);
    lcd1.print(count);
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    while (count < 29){
      G01 (54.5,-71,0,0.5);
      G01 (56.7,-71,0,0.5);
      G01 (56.7,-61,0,0.5);
      G01 (54.5,-61,0,0.5);
      G01 (54.5,-71,0,0.5);
      G00 (54.5,-61,0);
      count ++;
      if (count < 29){
        lcd1.setCursor(0, 3);
        lcd1.print("St3  Pass No.    *Qt");
        lcd1.setCursor(14, 3);
        lcd1.print(count);
      }else{        
        lcd1.setCursor(14, 3);
        lcd1.print("END");
      }
      while (KeyPadRetrieve() < 12){
      }
      while (KeyPadRetrieve() != 11){
        if (KeyPadRetrieve() == 10){
          count = 29;
        }
      }
    }

        count = 1;
        
    LCD2_UpdateDelay(1);
    lcd1.setCursor(0, 0);
    lcd1.print("  6mm MAGNET SLOTs  ");
    lcd1.setCursor(0, 1);
    lcd1.print("                    ");
    lcd1.setCursor(0, 2);
    lcd1.print(" Press # to Move to ");
    lcd1.setCursor(0, 3);
    lcd1.print("    START 1 of 28   ");
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    G00 (93.3,-61,0);
    lcd1.setCursor(0, 0);
    lcd1.print(" LOWER CUT BY 0.25mm ");
    lcd1.setCursor(0, 1);
    lcd1.print("   DURING MOVEMENT  ");
    lcd1.setCursor(0, 2);
    lcd1.print("    Press # to RUN  ");
    lcd1.setCursor(0, 3);
    lcd1.print("St4  Pass No.    *Qt");
    lcd1.setCursor(14, 3);
    lcd1.print(count);
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    while (count < 29){
      G01 (93.3,-71,0,0.5);
      G01 (95.5,-71,0,0.5);
      G01 (95.5,-61,0,0.5);
      G01 (93.3,-61,0,0.5);
      G01 (93.3,-71,0,0.5);
      G00 (93.3,-61,0);
      count ++;
      if (count < 29){
        lcd1.setCursor(0, 3);
        lcd1.print("St4  Pass No.    *Qt");
        lcd1.setCursor(14, 3);
        lcd1.print(count);
      }else{        
        lcd1.setCursor(14, 3);
        lcd1.print("END");
      }
      while (KeyPadRetrieve() < 12){
      }
      while (KeyPadRetrieve() != 11){
        if (KeyPadRetrieve() == 10){
          count = 29;
        }
      }
    }

        count = 1;
        
    LCD2_UpdateDelay(1);
    lcd1.setCursor(0, 0);
    lcd1.print("  6mm MAGNET SLOTs  ");
    lcd1.setCursor(0, 1);
    lcd1.print("                    ");
    lcd1.setCursor(0, 2);
    lcd1.print(" Press # to Move to ");
    lcd1.setCursor(0, 3);
    lcd1.print("    START 1 of 28   ");
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    G00 (113.8,-64.5,0);
    lcd1.setCursor(0, 0);
    lcd1.print(" LOWER CUT BY 0.25mm ");
    lcd1.setCursor(0, 1);
    lcd1.print("   DURING MOVEMENT  ");
    lcd1.setCursor(0, 2);
    lcd1.print("    Press # to RUN  ");
    lcd1.setCursor(0, 3);
    lcd1.print("St5  Pass No.    *Qt");
    lcd1.setCursor(14, 3);
    lcd1.print(count);
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    while (count < 29){
      G01 (113.8,-74.5,0,0.5);
      G01 (116,-74.5,0,0.5);
      G01 (116,-64.5,0,0.5);
      G01 (113.8,-64.5,0,0.5);
      G01 (113.8,-74.5,0,0.5);
      G00 (113.8,-64.5,0);
      count ++;
      if (count < 29){
        lcd1.setCursor(0, 3);
        lcd1.print("St5  Pass No.    *Qt");
        lcd1.setCursor(14, 3);
        lcd1.print(count);
      }else{        
        lcd1.setCursor(14, 3);
        lcd1.print("END");
      }
      while (KeyPadRetrieve() < 12){
      }
      while (KeyPadRetrieve() != 11){
        if (KeyPadRetrieve() == 10){
          count = 29;
        }
      }
    }

        count = 1;
        
    LCD2_UpdateDelay(1);
    lcd1.setCursor(0, 0);
    lcd1.print("  6mm MAGNET SLOTs  ");
    lcd1.setCursor(0, 1);
    lcd1.print("                    ");
    lcd1.setCursor(0, 2);
    lcd1.print(" Press # to Move to ");
    lcd1.setCursor(0, 3);
    lcd1.print("    START 1 of 28   ");
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    G00 (135,-64.5,0);
    lcd1.setCursor(0, 0);
    lcd1.print(" LOWER CUT BY 0.25mm ");
    lcd1.setCursor(0, 1);
    lcd1.print("   DURING MOVEMENT  ");
    lcd1.setCursor(0, 2);
    lcd1.print("    Press # to RUN  ");
    lcd1.setCursor(0, 3);
    lcd1.print("St6  Pass No.    *Qt");
    lcd1.setCursor(14, 3);
    lcd1.print(count);
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    while (count < 29){
      G01 (135,-74.5,0,0.5);
      G01 (137.2,-74.5,0,0.5);
      G01 (137.2,-64.5,0,0.5);
      G01 (135,-64.5,0,0.5);
      G01 (135,-74.5,0,0.5);
      G00 (135,-64.5,0);
      count ++;
      if (count < 29){
        lcd1.setCursor(0, 3);
        lcd1.print("St6  Pass No.    *Qt");
        lcd1.setCursor(14, 3);
        lcd1.print(count);
      }else{        
        lcd1.setCursor(14, 3);
        lcd1.print("END");
      }
      while (KeyPadRetrieve() < 12){
      }
      while (KeyPadRetrieve() != 11){
        if (KeyPadRetrieve() == 10){
          count = 29;
        }
      }
    }

        count = 1;
        
    LCD2_UpdateDelay(1);
    lcd1.setCursor(0, 0);
    lcd1.print("  6mm MAGNET SLOTs  ");
    lcd1.setCursor(0, 1);
    lcd1.print("                    ");
    lcd1.setCursor(0, 2);
    lcd1.print(" Press # to Move to ");
    lcd1.setCursor(0, 3);
    lcd1.print("    START 1 of 28   ");
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    G00 (160,-64.5,0);
    lcd1.setCursor(0, 0);
    lcd1.print(" LOWER CUT BY 0.25mm ");
    lcd1.setCursor(0, 1);
    lcd1.print("   DURING MOVEMENT  ");
    lcd1.setCursor(0, 2);
    lcd1.print("    Press # to RUN  ");
    lcd1.setCursor(0, 3);
    lcd1.print("St7  Pass No.    *Qt");
    lcd1.setCursor(14, 3);
    lcd1.print(count);
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    while (count < 29){
      G01 (160,-74.5,0,0.5);
      G01 (162.2,-74.5,0,0.5);
      G01 (162.2,-64.5,0,0.5);
      G01 (160,-64.5,0,0.5);
      G01 (160,-74.5,0,0.5);
      G00 (160,-64.5,0);
      count ++;
      if (count < 29){
        lcd1.setCursor(0, 3);
        lcd1.print("St7  Pass No.    *Qt");
        lcd1.setCursor(14, 3);
        lcd1.print(count);
      }else{        
        lcd1.setCursor(14, 3);
        lcd1.print("END");
      }
      while (KeyPadRetrieve() < 12){
      }
      while (KeyPadRetrieve() != 11){
        if (KeyPadRetrieve() == 10){
          count = 29;
        }
      }
    }

        count = 1;
        
    LCD2_UpdateDelay(1);
    lcd1.setCursor(0, 0);
    lcd1.print("  6mm MAGNET SLOTs  ");
    lcd1.setCursor(0, 1);
    lcd1.print("                    ");
    lcd1.setCursor(0, 2);
    lcd1.print(" Press # to Move to ");
    lcd1.setCursor(0, 3);
    lcd1.print("    START 1 of 28   ");
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    G00 (188.8,-64.5,0);
    lcd1.setCursor(0, 0);
    lcd1.print(" LOWER CUT BY 0.25mm ");
    lcd1.setCursor(0, 1);
    lcd1.print("   DURING MOVEMENT  ");
    lcd1.setCursor(0, 2);
    lcd1.print("    Press # to RUN  ");
    lcd1.setCursor(0, 3);
    lcd1.print("St8  Pass No.    *Qt");
    lcd1.setCursor(14, 3);
    lcd1.print(count);
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    while (count < 29){
      G01 (188.8,-74.5,0,0.5);
      G01 (191,-74.5,0,0.5);
      G01 (191,-64.5,0,0.5);
      G01 (188.8,-64.5,0,0.5);
      G01 (188.8,-74.5,0,0.5);
      G00 (188.8,-64.5,0);
      count ++;
      if (count < 29){
        lcd1.setCursor(0, 3);
        lcd1.print("St8  Pass No.    *Qt");
        lcd1.setCursor(14, 3);
        lcd1.print(count);
      }else{        
        lcd1.setCursor(14, 3);
        lcd1.print("END");
      }
      while (KeyPadRetrieve() < 12){
      }
      while (KeyPadRetrieve() != 11){
        if (KeyPadRetrieve() == 10){
          count = 29;
        }
      }
    }

        count = 1;
        
    LCD2_UpdateDelay(1);
    lcd1.setCursor(0, 0);
    lcd1.print("  6mm MAGNET SLOTs  ");
    lcd1.setCursor(0, 1);
    lcd1.print("                    ");
    lcd1.setCursor(0, 2);
    lcd1.print(" Press # to Move to ");
    lcd1.setCursor(0, 3);
    lcd1.print("    START 1 of 28   ");
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    G00 (205.5,-59.13,0);
    lcd1.setCursor(0, 0);
    lcd1.print(" LOWER CUT BY 0.25mm ");
    lcd1.setCursor(0, 1);
    lcd1.print("   DURING MOVEMENT  ");
    lcd1.setCursor(0, 2);
    lcd1.print("    Press # to RUN  ");
    lcd1.setCursor(0, 3);
    lcd1.print("St9  Pass No.    *Qt");
    lcd1.setCursor(14, 3);
    lcd1.print(count);
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    while (count < 29){
      G01 (205.5,-71.13,0,0.5);
      G01 (207.65,-71.13,0,0.5);
      G01 (207.65,-59.13,0,0.5);
      G01 (205.5,-59.13,0,0.5);
      G01 (205.5,-71.13,0,0.5);
      G00 (205.5,-59.13,0);
      count ++;
      if (count < 29){
        lcd1.setCursor(0, 3);
        lcd1.print("St9  Pass No.    *Qt");
        lcd1.setCursor(14, 3);
        lcd1.print(count);
      }else{        
        lcd1.setCursor(14, 3);
        lcd1.print("END");
      }
      while (KeyPadRetrieve() < 12){
      }
      while (KeyPadRetrieve() != 11){
        if (KeyPadRetrieve() == 10){
          count = 29;
        }
      }
    }

        count = 1;
        
    LCD2_UpdateDelay(1);
    lcd1.setCursor(0, 0);
    lcd1.print("  6mm MAGNET SLOTs  ");
    lcd1.setCursor(0, 1);
    lcd1.print("                    ");
    lcd1.setCursor(0, 2);
    lcd1.print(" Press # to Move to ");
    lcd1.setCursor(0, 3);
    lcd1.print("    START 1 of 28   ");
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    G00 (240.35,-59.13,0);
    lcd1.setCursor(0, 0);
    lcd1.print(" LOWER CUT BY 0.25mm ");
    lcd1.setCursor(0, 1);
    lcd1.print("   DURING MOVEMENT  ");
    lcd1.setCursor(0, 2);
    lcd1.print("    Press # to RUN  ");
    lcd1.setCursor(0, 3);
    lcd1.print("St10 Pass No.    *Qt");
    lcd1.setCursor(14, 3);
    lcd1.print(count);
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    while (count < 29){
      G01 (240.35,-71.13,0,0.5);
      G01 (242.5,-71.13,0,0.5);
      G01 (242.5,-59.13,0,0.5);
      G01 (240.35,-59.13,0,0.5);
      G01 (240.35,-71.13,0,0.5);
      G00 (240.35,-59.13,0);
      count ++;
      if (count < 29){
        lcd1.setCursor(0, 3);
        lcd1.print("St10 Pass No.    *Qt");
        lcd1.setCursor(14, 3);
        lcd1.print(count);
      }else{        
        lcd1.setCursor(14, 3);
        lcd1.print("END");
      }
      while (KeyPadRetrieve() < 12){
      }
      while (KeyPadRetrieve() != 11){
        if (KeyPadRetrieve() == 10){
          count = 29;
        }
      }
    }

        count = 1;
        
    LCD2_UpdateDelay(1);
    lcd1.setCursor(0, 0);
    lcd1.print("  6mm MAGNET SLOTs  ");
    lcd1.setCursor(0, 1);
    lcd1.print("                    ");
    lcd1.setCursor(0, 2);
    lcd1.print(" Press # to Move to ");
    lcd1.setCursor(0, 3);
    lcd1.print("    START 1 of 28   ");
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    G00 (252.5,-59.13,0);
    lcd1.setCursor(0, 0);
    lcd1.print(" LOWER CUT BY 0.25mm ");
    lcd1.setCursor(0, 1);
    lcd1.print("   DURING MOVEMENT  ");
    lcd1.setCursor(0, 2);
    lcd1.print("    Press # to RUN  ");
    lcd1.setCursor(0, 3);
    lcd1.print("St11 Pass No.    *Qt");
    lcd1.setCursor(14, 3);
    lcd1.print(count);
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    while (count < 29){
      G01 (252.5,-71,0,0.5);
      G01 (254.65,-71,0,0.5);
      G01 (254.65,-59.13,0,0.5);
      G01 (252.5,-59.13,0,0.5);
      G01 (252.5,-71,0,0.5);
      G00 (252.5,-59.13,0);
      count ++;
      if (count < 29){
        lcd1.setCursor(0, 3);
        lcd1.print("St11 Pass No.    *Qt");
        lcd1.setCursor(14, 3);
        lcd1.print(count);
      }else{        
        lcd1.setCursor(14, 3);
        lcd1.print("END");
      }
      while (KeyPadRetrieve() < 12){
      }
      while (KeyPadRetrieve() != 11){
        if (KeyPadRetrieve() == 10){
          count = 29;
        }
      }
    }

        count = 1;
        
    LCD2_UpdateDelay(1);
    lcd1.setCursor(0, 0);
    lcd1.print("  6mm MAGNET SLOTs  ");
    lcd1.setCursor(0, 1);
    lcd1.print("                    ");
    lcd1.setCursor(0, 2);
    lcd1.print(" Press # to Move to ");
    lcd1.setCursor(0, 3);
    lcd1.print("    START 1 of 28   ");
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    G00 (287.35,-59.13,0);
    lcd1.setCursor(0, 0);
    lcd1.print(" LOWER CUT BY 0.25mm ");
    lcd1.setCursor(0, 1);
    lcd1.print("   DURING MOVEMENT  ");
    lcd1.setCursor(0, 2);
    lcd1.print("    Press # to RUN  ");
    lcd1.setCursor(0, 3);
    lcd1.print("St12 Pass No.    *Qt");
    lcd1.setCursor(14, 3);
    lcd1.print(count);
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    while (count < 29){
      G01 (287.35,-71.13,0,0.5);
      G01 (289.5,-71.13,0,0.5);
      G01 (289.5,-59.13,0,0.5);
      G01 (287.35,-59.13,0,0.5);
      G01 (287.35,-71.13,0,0.5);
      G00 (287.35,-59.13,0);
      count ++;
      if (count < 29){
        lcd1.setCursor(0, 3);
        lcd1.print("St12 Pass No.    *Qt");
        lcd1.setCursor(14, 3);
        lcd1.print(count);
      }else{        
        lcd1.setCursor(14, 3);
        lcd1.print("END");
      }
      while (KeyPadRetrieve() < 12){
      }
      while (KeyPadRetrieve() != 11){
        if (KeyPadRetrieve() == 10){
          count = 29;
        }
      }
    }

        count = 1;
        
    LCD2_UpdateDelay(1);
    lcd1.setCursor(0, 0);
    lcd1.print("  6mm MAGNET SLOTs  ");
    lcd1.setCursor(0, 1);
    lcd1.print("                    ");
    lcd1.setCursor(0, 2);
    lcd1.print(" Press # to Move to ");
    lcd1.setCursor(0, 3);
    lcd1.print("    START 1 of 28   ");
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    G00 (266,-71.85,0);
    lcd1.setCursor(0, 0);
    lcd1.print(" LOWER CUT BY 0.25mm ");
    lcd1.setCursor(0, 1);
    lcd1.print("   DURING MOVEMENT  ");
    lcd1.setCursor(0, 2);
    lcd1.print("    Press # to RUN  ");
    lcd1.setCursor(0, 3);
    lcd1.print("St13 Pass No.    *Qt");
    lcd1.setCursor(14, 3);
    lcd1.print(count);
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    while (count < 29){
      G01 (276,-71.85,0,0.5);
      G01 (276,-74,0,0.5);
      G01 (266,-74,0,0.5);
      G01 (266,-71.85,0,0.5);
      G01 (276,-71.85,0,0.5);
      G00 (266,-71.85,0);
      count ++;
      if (count < 29){
        lcd1.setCursor(0, 3);
        lcd1.print("St13 Pass No.    *Qt");
        lcd1.setCursor(14, 3);
        lcd1.print(count);
      }else{        
        lcd1.setCursor(14, 3);
        lcd1.print("END");
      }
      while (KeyPadRetrieve() < 12){
      }
      while (KeyPadRetrieve() != 11){
        if (KeyPadRetrieve() == 10){
          count = 29;
        }
      }
    }
  }

    if (menuSelect == 7 && KeyPadRetrieve() == 7) { // Internal Recesses, Magnet Slot 5.1mm Depth
    count = 1;
    while (KeyPadRetrieve() < 12){
    }
    LCD2_UpdateDelay(1);
    lcd1.setCursor(0, 0);
    lcd1.print(" 5.1mm MAGNET SLOTs ");
    lcd1.setCursor(0, 1);
    lcd1.print("                    ");
    lcd1.setCursor(0, 2);
    lcd1.print(" Press # to Move to ");
    lcd1.setCursor(0, 3);
    lcd1.print("    START 1 of 25   ");
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    G00 (222.93,-2.5,0);
    lcd1.setCursor(0, 0);
    lcd1.print(" LOWER CUT BY 0.25mm ");
    lcd1.setCursor(0, 1);
    lcd1.print("   DURING MOVEMENT  ");
    lcd1.setCursor(0, 2);
    lcd1.print("    Press # to RUN  ");
    lcd1.setCursor(0, 3);
    lcd1.print("St1  Pass No.    *Qt");
    lcd1.setCursor(14, 3);
    lcd1.print(count);
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    while (count < 26){
      G01 (222.93,-13.5,0,0.5);
      G01 (225.08,-13.5,0,0.5);
      G01 (225.08,-2.5,0,0.5);
      G01 (222.93,-2.5,0,0.5);
      G01 (222.93,-13.5,0,0.5);
      G00 (222.93,-2.5,0);
      count ++;
      if (count < 26){
        lcd1.setCursor(0, 3);
        lcd1.print("St1  Pass No.    *Qt");
        lcd1.setCursor(14, 3);
        lcd1.print(count);
      }else{        
        lcd1.setCursor(14, 3);
        lcd1.print("END");
      }
      while (KeyPadRetrieve() < 12){
      }
      while (KeyPadRetrieve() != 11){
        if (KeyPadRetrieve() == 10){
          count = 26;
        }
      }
    }

    count = 1;
        
    LCD2_UpdateDelay(1);
    lcd1.setCursor(0, 0);
    lcd1.print(" 5.1mm MAGNET SLOTs ");
    lcd1.setCursor(0, 1);
    lcd1.print("                    ");
    lcd1.setCursor(0, 2);
    lcd1.print(" Press # to Move to ");
    lcd1.setCursor(0, 3);
    lcd1.print("    START 1 of 25   ");
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    G00 (269.93,-2.5,0);
    lcd1.setCursor(0, 0);
    lcd1.print(" LOWER CUT BY 0.25mm ");
    lcd1.setCursor(0, 1);
    lcd1.print("   DURING MOVEMENT  ");
    lcd1.setCursor(0, 2);
    lcd1.print("    Press # to RUN  ");
    lcd1.setCursor(0, 3);
    lcd1.print("St2  Pass No.    *Qt");
    lcd1.setCursor(14, 3);
    lcd1.print(count);
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    while (count < 26){
      G01 (269.93,-13.5,0,0.5);
      G01 (272.08,-13.5,0,0.5);
      G01 (272.08,-2.5,0,0.5);
      G01 (269.93,-2.5,0,0.5);
      G01 (269.93,-13.5,0,0.5);
      G00 (269.93,-2.5,0);
      count ++;
      if (count < 26){
        lcd1.setCursor(0, 3);
        lcd1.print("St2  Pass No.    *Qt");
        lcd1.setCursor(14, 3);
        lcd1.print(count);
      }else{        
        lcd1.setCursor(14, 3);
        lcd1.print("END");
      }
      while (KeyPadRetrieve() < 12){
      }
      while (KeyPadRetrieve() != 11){
        if (KeyPadRetrieve() == 10){
          count = 26;
        }
      }
    }
  }

    if (menuSelect == 7 && KeyPadRetrieve() == 8) { // Internal Recesses, Magnet Slot 9mm Depth
    count = 1;
    while (KeyPadRetrieve() < 12){
    }
    LCD2_UpdateDelay(1);
    lcd1.setCursor(0, 0);
    lcd1.print("  9mm MAGNET SLOTs  ");
    lcd1.setCursor(0, 1);
    lcd1.print("                    ");
    lcd1.setCursor(0, 2);
    lcd1.print(" Press # to Move to ");
    lcd1.setCursor(0, 3);
    lcd1.print("    START 1 of 40   ");
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    G00 (219,-71.85,0);
    lcd1.setCursor(0, 0);
    lcd1.print(" LOWER CUT BY 0.25mm ");
    lcd1.setCursor(0, 1);
    lcd1.print("   DURING MOVEMENT  ");
    lcd1.setCursor(0, 2);
    lcd1.print("    Press # to RUN  ");
    lcd1.setCursor(0, 3);
    lcd1.print("     Pass No.    *Qt");
    lcd1.setCursor(14, 3);
    lcd1.print(count);
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    while (count < 41){
      G01 (229,-71.85,0,0.5);
      G01 (229,-74,0,0.5);
      G01 (219,-74,0,0.5);
      G01 (219,-71.85,0,0.5);
      G01 (229,-71.85,0,0.5);
      G00 (219,-71.85,0);
      count ++;
      if (count < 41){
        lcd1.setCursor(0, 3);
        lcd1.print("     Pass No.    *Qt");
        lcd1.setCursor(14, 3);
        lcd1.print(count);
      }else{        
        lcd1.setCursor(14, 3);
        lcd1.print("END");
      }
      while (KeyPadRetrieve() < 12){
      }
      while (KeyPadRetrieve() != 11){
        if (KeyPadRetrieve() == 10){
          count = 41;
        }
      }
    }
  }
*/
  
  if (menuSelect == 7 && KeyPadRetrieve() == 10) {
    while (KeyPadRetrieve() < 12){
    }
    lCD2_UpdatePause = false;
    menuSelect = 6;
  }
  
//______________________________MENU 8__________________________________
  
  if (menuSelect == 8) {
    lCD2_UpdatePause = true;
    lcd1.setCursor(0, 0);
    lcd1.print("1:Rip using Y-Axis  ");
    lcd1.setCursor(0, 1);
    lcd1.print("2:Block Face Off X-A");
    lcd1.setCursor(0, 2);
    lcd1.print("3:Front Recess Rough");
    lcd1.setCursor(0, 3);
    lcd1.print("4:Front Recess Finsh");
    lcd2.setCursor(0, 0);
    lcd2.print("5:Rear Lft Rcs Rough");
    lcd2.setCursor(0, 1);
    lcd2.print("6:Rear Lft Rcs Fnish");
    lcd2.setCursor(0, 2);
    lcd2.print("7:Rear Rgt Rcs Rough");
    lcd2.setCursor(0, 3);
    lcd2.print("8:NEXT        *:Back");
  }
  
  if (menuSelect == 8 && KeyPadRetrieve() == 1) { // Blank cut down using ripper 
    while (KeyPadRetrieve() < 12){
    }
    LCD2_UpdateDelay(1);
    lcd1.setCursor(0, 0);
    lcd1.print("FIT RIPPER END MILL ");
    lcd1.setCursor(0, 1);
    lcd1.print("+115mm to 0mm Y-Axis");
    lcd1.setCursor(0, 2);
    lcd1.print(" Press # to Move to ");
    lcd1.setCursor(0, 3);
    lcd1.print("       START        ");
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    G00 (0,115,0);
    lcd1.setCursor(0, 0);
    lcd1.print("   Blank Cut Down   ");
    lcd1.setCursor(0, 1);
    lcd1.print("+115mm to 0mm Y-Axis");
    lcd1.setCursor(0, 2);
    lcd1.print("     Press # to     ");
    lcd1.setCursor(0, 3);
    lcd1.print("        RUN         ");
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    G01 (0,0,0,3);
    G00 (0,115,0);
  }

  if (menuSelect == 8 && KeyPadRetrieve() == 2) { // Face Off to Square Head Block 
    while (KeyPadRetrieve() < 12){
    }
    LCD2_UpdateDelay(1);
    lcd1.setCursor(0, 0);
    lcd1.print("FIT 40mm SHELL MILL ");
    lcd1.setCursor(0, 1);
    lcd1.print("-45mm to +145mm X-Axis");
    lcd1.setCursor(0, 2);
    lcd1.print("-10 to +65mm Y-Axis ");
    lcd1.setCursor(0, 3);
    lcd1.print(" Press # GOTO START ");
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    G00 (-45,-10,0);
    lcd1.setCursor(0, 0);
    lcd1.print("   Blank Cut Down   ");
    lcd1.setCursor(0, 1);
    lcd1.print("+115mm to 0mm Y-Axis");
    lcd1.setCursor(0, 2);
    lcd1.print("     Press # to     ");
    lcd1.setCursor(0, 3);
    lcd1.print("        RUN         ");
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    G01 (105,-10,0,3);
    G00 (105,25,0);
    G01 (-45,25,0,3);
    G00 (-45,-10,0);
  }

  if (menuSelect == 8 && KeyPadRetrieve() == 3) { // Front Recess Feature and Part Separation
    count = 1;
    while (KeyPadRetrieve() < 12){
    }
    LCD2_UpdateDelay(1);
    lcd1.setCursor(0, 0);
    lcd1.print(" FIT 8mm ROUGH MILL ");
    lcd1.setCursor(0, 1);
    lcd1.print("    Drop by 0.9mm   ");
    lcd1.setCursor(0, 2);
    lcd1.print("                    ");
    lcd1.setCursor(0, 3);
    lcd1.print(" Press # GOTO START ");
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    while (count < 24){
      G00 (7.2,-5,0);
      lcd1.setCursor(0, 0);
      lcd1.print("FRONT RECESS & SPLIT");
      lcd1.setCursor(0, 1);
      lcd1.print("Pass    of 23       ");
      lcd1.setCursor(5, 1);
      lcd1.print(count);
      if (count < 23){
        lcd1.setCursor(0, 2);
        lcd1.print("Drop by 0.9mm  18g");
      }else{
        lcd1.setCursor(0, 2);
        lcd1.print("Drop by 0.2mm  4g ");
      }
      lcd1.setCursor(0, 3);
      lcd1.print("  #:RUN   *:Quit  ");
      while (KeyPadRetrieve() < 12){
      }
      while (KeyPadRetrieve() != 11){
        if (KeyPadRetrieve() == 10){
          count = 24;
        }
      }
      if (count < 24){
        G01 (7.2,16.8,0,2);
        G01 (38.8,16.8,0,2);
        G01 (38.8,-5,0,2);
        G00 (38.8,9.8,0);
        G01 (7.2,9.8,0,2);
        G00 (7.2,2.8,0);
        G01 (38.8,2.8,0,2);
        G00 (38.8,-5,0);
        G00 (61.2,-5,0);
        G01 (61.2,16.8,0,2);
        G01 (92.8,16.8,0,2);
        G00 (71.2,16.8,0);
        G01 (71.2,21.8,0,2);
        G01 (82.8,21.8,0,2);
        G01 (82.8,16.8,0,2);
        G00 (92.8,16.8,0);
        G01 (92.8,-5,0,2);
        G00 (92.8,9.8,0);
        G01 (61.2,9.8,0,2);
        G00 (61.2,2.8,0);
        G01 (92.8,2.8,0,2);
        G00 (92.8,-5,0);
        G00 (50,-5,0);
        G01 (50,50,0,2);
        G01 (-5,50,0,2);
        G00 (50,50,0);
        G01 (108,50,0,2);
        G00 (108,-5,0);
        G00 (7.2,-5,0);
        count ++;
      }
    }
  }

  if (menuSelect == 8 && KeyPadRetrieve() == 4) { // Front Recess Feature Finishing Pass
    count = 1;
    while (KeyPadRetrieve() < 12){
    }
    LCD2_UpdateDelay(1);
    lcd1.setCursor(0, 0);
    lcd1.print("FIT 8mm SLOT ENDMILL");
    lcd1.setCursor(0, 1);
    lcd1.print("    Drop by 0.9mm   ");
    lcd1.setCursor(0, 2);
    lcd1.print("                    ");
    lcd1.setCursor(0, 3);
    lcd1.print(" Press # GOTO START ");
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    while (count < 24){
      G00 (7,-5,0);
      lcd1.setCursor(0, 0);
      lcd1.print("FIT 8mm SLOT ENDMILL");
      lcd1.setCursor(0, 1);
      lcd1.print("Pass    of 23       ");
      lcd1.setCursor(5, 1);
      lcd1.print(count);
      if (count < 23){
        lcd1.setCursor(0, 2);
        lcd1.print("Drop by 0.9mm  18g");
      }else{
        lcd1.setCursor(0, 2);
        lcd1.print("Drop by 0.2mm  4g ");
      }
      lcd1.setCursor(0, 3);
      lcd1.print("  #:RUN   *:Quit  ");
      while (KeyPadRetrieve() < 12){
      }
      while (KeyPadRetrieve() != 11){
        if (KeyPadRetrieve() == 10){
          count = 24;
        }
      }
      if (count < 24){
        G01 (7,17,0,2);
        G01 (39,17,0,2);
        G01 (39,-5,0,2);
        G00 (61,-5,0);
        G01 (61,17,0,2);
        G01 (71,17,0,2);
        G01 (71,22,0,2);
        G01 (83,22,0,2);
        G01 (83,17,0,2);
        G01 (93,17,0,2);
        G01 (93,-5,0,2);
        G00 (7,-5,0);
        count ++;
      }else{  
        G01 (7,17,0,2);
        G01 (39,17,0,2);
        G01 (39,-5,0,2);
        G00 (39,10,0);
        G01 (7,10,0,2);
        G00 (7,3,0);
        G01 (39,3,0,2);
        G00 (39,-5,0);
        G00 (61,-5,0);
        G01 (61,17,0,2);
        G01 (93,17,0,2);
        G00 (71,17,0);
        G01 (71,22,0,2);
        G01 (83,22,0,2);
        G01 (83,17,0,2);
        G00 (93,17,0);
        G01 (93,-5,0,2);
        G00 (93,10,0);
        G01 (61,10,0,2);
        G00 (61,3,0);
        G01 (93,3,0,2);
        G00 (93,-5,0);
        G00 (7,-5,0);
        count ++;
      }
    }
  }

  if (menuSelect == 8 && KeyPadRetrieve() == 5) { // Rear Left Recess Roughing
    count = 1;
    while (KeyPadRetrieve() < 12){
    }
    LCD2_UpdateDelay(1);
    lcd1.setCursor(0, 0);
    lcd1.print("FIT 8mm ROUGHING MIL");
    lcd1.setCursor(0, 1);
    lcd1.print("Drop by 0.9mm during");
    lcd1.setCursor(0, 2);
    lcd1.print(" cut                ");
    lcd1.setCursor(0, 3);
    lcd1.print(" Press # GOTO START ");
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    while (count < 24){
      G00 (10.2,36.2,0);
      lcd1.setCursor(0, 0);
      lcd1.print("REAR LEFT RCSS ROUGH");
      lcd1.setCursor(0, 1);
      lcd1.print("Pass    of 23       ");
      lcd1.setCursor(5, 1);
      lcd1.print(count);
      if (count < 23){
        lcd1.setCursor(0, 2);
        lcd1.print("Drop by 0.9mm  18g");
      }else{
        lcd1.setCursor(0, 2);
        lcd1.print("Drop by 0.2mm  4g ");
      }
      lcd1.setCursor(0, 3);
      lcd1.print("  #:RUN   *:Quit  ");
      while (KeyPadRetrieve() < 12){
      }
      while (KeyPadRetrieve() != 11){
        if (KeyPadRetrieve() == 10){
          count = 24;
        }
      }
      G01 (34.8,36.2,0,2);
      G01 (34.8,37.8,0,2);
      G01 (10.2,37.8,0,2);
      G01 (10.2,36.2,0,2);
      count ++;
    }
  }

  if (menuSelect == 8 && KeyPadRetrieve() == 6) { // Rear Left Recess Finishing
    count = 1;
    while (KeyPadRetrieve() < 12){
    }
    LCD2_UpdateDelay(1);
    lcd1.setCursor(0, 0);
    lcd1.print("   FIT 8mm END MIL  ");
    lcd1.setCursor(0, 1);
    lcd1.print("Drop by 0.9mm during");
    lcd1.setCursor(0, 2);
    lcd1.print(" cut                ");
    lcd1.setCursor(0, 3);
    lcd1.print(" Press # GOTO START ");
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    while (count < 24){
      G00 (10,36,0);
      lcd1.setCursor(0, 0);
      lcd1.print("REAR LEFT RCSS FNISH");
      lcd1.setCursor(0, 1);
      lcd1.print("Pass    of 23       ");
      lcd1.setCursor(5, 1);
      lcd1.print(count);
      if (count < 23){
        lcd1.setCursor(0, 2);
        lcd1.print("Drop by 0.9mm  18g");
      }else{
        lcd1.setCursor(0, 2);
        lcd1.print("Drop by 0.2mm  4g ");
      }
      lcd1.setCursor(0, 3);
      lcd1.print("  #:RUN   *:Quit  ");
      while (KeyPadRetrieve() < 12){
      }
      while (KeyPadRetrieve() != 11){
        if (KeyPadRetrieve() == 10){
          count = 24;
        }
      }
      G01 (35,36,0,2);
      G01 (35,38,0,2);
      G01 (10,38,0,2);
      G01 (10,36,0,2);
      count ++;
    }
  }

  if (menuSelect == 8 && KeyPadRetrieve() == 7) { // Rear Right Recess Roughing
    count = 1;
    while (KeyPadRetrieve() < 12){
    }
    LCD2_UpdateDelay(1);
    lcd1.setCursor(0, 0);
    lcd1.print("FIT 8mm ROUGHING MIL");
    lcd1.setCursor(0, 1);
    lcd1.print("Drop by 0.9mm during");
    lcd1.setCursor(0, 2);
    lcd1.print(" cut                ");
    lcd1.setCursor(0, 3);
    lcd1.print(" Press # GOTO START ");
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    while (count < 24){
      G00 (64.2,36.2,0);
      lcd1.setCursor(0, 0);
      lcd1.print("REAR RIGHT RCS ROUGH");
      lcd1.setCursor(0, 1);
      lcd1.print("Pass    of 23       ");
      lcd1.setCursor(5, 1);
      lcd1.print(count);
      if (count < 23){
        lcd1.setCursor(0, 2);
        lcd1.print("Drop by 0.9mm  18g");
      }else{
        lcd1.setCursor(0, 2);
        lcd1.print("Drop by 0.2mm  4g ");
      }
      lcd1.setCursor(0, 3);
      lcd1.print("  #:RUN   *:Quit  ");
      while (KeyPadRetrieve() < 12){
      }
      while (KeyPadRetrieve() != 11){
        if (KeyPadRetrieve() == 10){
          count = 24;
        }
      }
      G01 (89.8,36.2,0,2);
      G01 (89.8,37.8,0,2);
      G01 (64.2,37.8,0,2);
      G01 (64.2,36.2,0,2);
      count ++;
    }
  }

  if (menuSelect == 8 && KeyPadRetrieve() == 8) {
    while (KeyPadRetrieve() < 12){
    }
    lCD2_UpdatePause = false;
    menuSelect = 9;
  }
  
  if (menuSelect == 8 && KeyPadRetrieve() == 10) {
    while (KeyPadRetrieve() < 12){
    }
    lCD2_UpdatePause = false;
    menuSelect = 6;
  }

//______________________________MENU 9__________________________________
  
  if (menuSelect == 9) {
    lCD2_UpdatePause = true;
    lcd1.setCursor(0, 0);
    lcd1.print("1:Rear Rgt Rcs Fnish");
    lcd1.setCursor(0, 1);
    lcd1.print("2:Right Center Rough");
    lcd1.setCursor(0, 2);
    lcd1.print("3:Right Center Fnish");
    lcd1.setCursor(0, 3);
    lcd1.print("4:Holes             ");
    lcd2.setCursor(0, 0);
    lcd2.print("5:Face Off Sides    ");
    lcd2.setCursor(0, 1);
    lcd2.print("6:Pixel Feature Head");
    lcd2.setCursor(0, 2);
    lcd2.print("7:Chamfer Head Edges");
    lcd2.setCursor(0, 3);
    lcd2.print("8:            *:Back");
  }
  
  if (menuSelect == 9 && KeyPadRetrieve() == 10) {
    while (KeyPadRetrieve() < 12){
    }
    lCD2_UpdatePause = false;
    menuSelect = 8;
  }

  if (menuSelect == 9 && KeyPadRetrieve() == 1) { // Rear Right Recess Finishing
    count = 1;
    while (KeyPadRetrieve() < 12){
    }
    LCD2_UpdateDelay(1);
    lcd1.setCursor(0, 0);
    lcd1.print("   FIT 8mm END MIL  ");
    lcd1.setCursor(0, 1);
    lcd1.print("Drop by 0.9mm during");
    lcd1.setCursor(0, 2);
    lcd1.print(" cut                ");
    lcd1.setCursor(0, 3);
    lcd1.print(" Press # GOTO START ");
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    while (count < 24){
      G00 (64,36,0);
      lcd1.setCursor(0, 0);
      lcd1.print("REAR RIGHT RCS FNISH");
      lcd1.setCursor(0, 1);
      lcd1.print("Pass    of 23       ");
      lcd1.setCursor(5, 1);
      lcd1.print(count);
      if (count < 23){
        lcd1.setCursor(0, 2);
        lcd1.print("Drop by 0.9mm  18g");
      }else{
        lcd1.setCursor(0, 2);
        lcd1.print("Drop by 0.2mm  4g ");
      }
      lcd1.setCursor(0, 3);
      lcd1.print("  #:RUN   *:Quit  ");
      while (KeyPadRetrieve() < 12){
      }
      while (KeyPadRetrieve() != 11){
        if (KeyPadRetrieve() == 10){
          count = 24;
        }
      }
      G01 (90,36,0,2);
      G01 (90,38,0,2);
      G01 (64,38,0,2);
      G01 (64,36,0,2);
      count ++;
    }
  }

  if (menuSelect == 9 && KeyPadRetrieve() == 2) { // Right Center Recess Roughing
    count = 1;
    while (KeyPadRetrieve() < 12){
    }
    LCD2_UpdateDelay(1);
    lcd1.setCursor(0, 0);
    lcd1.print(" FIT 8mm ROUGH MILL ");
    lcd1.setCursor(0, 1);
    lcd1.print("Drop by 0.9mm during");
    lcd1.setCursor(0, 2);
    lcd1.print(" cut                ");
    lcd1.setCursor(0, 3);
    lcd1.print(" Press # GOTO START ");
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    while (count < 18){
      G00 (71.2,17,0);
      lcd1.setCursor(0, 0);
      lcd1.print(" RIGHT CENTRE ROUGH ");
      lcd1.setCursor(0, 1);
      lcd1.print("Pass    of 17       ");
      lcd1.setCursor(5, 1);
      lcd1.print(count);
      if (count < 17){
        lcd1.setCursor(0, 2);
        lcd1.print("Drop by 0.9mm  18g");
      }else{
        lcd1.setCursor(0, 2);
        lcd1.print("Drop by 0.6mm  12g ");
      }
      lcd1.setCursor(0, 3);
      lcd1.print("  #:RUN   *:Quit  ");
      while (KeyPadRetrieve() < 12){
      }
      while (KeyPadRetrieve() != 11){
        if (KeyPadRetrieve() == 10){
          count = 18;
        }
      }
      G01 (71.2,36,0,2);
      G01 (78,36,0,2);
      G01 (78,17,0,2);
      G01 (82.8,17,0,2);
      G01 (82.8,36,0,2);
      G00 (71.2,17,0);
      count ++;
    }
  }

  if (menuSelect == 9 && KeyPadRetrieve() == 3) { // Right Center Recess Finishing
    count = 1;
    while (KeyPadRetrieve() < 12){
    }
    LCD2_UpdateDelay(1);
    lcd1.setCursor(0, 0);
    lcd1.print("FIT 8mm SLT END MILL");
    lcd1.setCursor(0, 1);
    lcd1.print("Drop by 0.9mm during");
    lcd1.setCursor(0, 2);
    lcd1.print(" cut                ");
    lcd1.setCursor(0, 3);
    lcd1.print(" Press # GOTO START ");
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    while (count < 18){
      G00 (71,17,0);
      lcd1.setCursor(0, 0);
      lcd1.print(" RIGHT CENTRE FNISH ");
      lcd1.setCursor(0, 1);
      lcd1.print("Pass    of 17       ");
      lcd1.setCursor(5, 1);
      lcd1.print(count);
      if (count < 17){
        lcd1.setCursor(0, 2);
        lcd1.print("Drop by 0.9mm  18g");
      }else{
        lcd1.setCursor(0, 2);
        lcd1.print("Drop by 0.6mm  12g ");
      }
      lcd1.setCursor(0, 3);
      lcd1.print("  #:RUN   *:Quit  ");
      while (KeyPadRetrieve() < 12){
      }
      while (KeyPadRetrieve() != 11){
        if (KeyPadRetrieve() == 10){
          count = 18;
        }
      }
      G01 (71,36,0,2);
      G01 (78,36,0,2);
      G01 (78,17,0,2);
      G01 (83,17,0,2);
      G01 (83,36,0,2);
      G00 (71,17,0);
      count ++;
    }
  }

  if (menuSelect == 9 && KeyPadRetrieve() == 4) { // All Holes
    count = 1;
    while (KeyPadRetrieve() < 12){
    }
    LCD2_UpdateDelay(1);
    lcd1.setCursor(0, 0);
    lcd1.print(" FIT 10mm DRILL BIT ");
    lcd1.setCursor(0, 1);
    lcd1.print("Drill to 20mm Depth ");
    lcd1.setCursor(0, 2);
    lcd1.print(" Then Counter Bore  ");
    lcd1.setCursor(0, 3);
    lcd1.print("  Press # to move   ");
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    G00 (23,26.5,0);
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    LCD2_UpdateDelay(1);
    lcd1.setCursor(0, 0);
    lcd1.print(" FIT 3mm DRILL BIT  ");
    lcd1.setCursor(0, 1);
    lcd1.print("Drill past 22mm Dpth");
    lcd1.setCursor(0, 2);
    lcd1.print("                    ");
    lcd1.setCursor(0, 3);
    lcd1.print(" Press # for Hole 1 ");
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    G00 (8,26.5,0);
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    LCD2_UpdateDelay(1);
    lcd1.setCursor(0, 0);
    lcd1.print(" FIT 3mm DRILL BIT  ");
    lcd1.setCursor(0, 1);
    lcd1.print("Drill past 22mm Dpth");
    lcd1.setCursor(0, 2);
    lcd1.print("                    ");
    lcd1.setCursor(0, 3);
    lcd1.print(" Press # for Hole 2 ");
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    G00 (38,26.5,0);
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    LCD2_UpdateDelay(1);
    lcd1.setCursor(0, 0);
    lcd1.print("FIT 2.5mm DRILL BIT ");
    lcd1.setCursor(0, 1);
    lcd1.print("Drill to 17mm Depth ");
    lcd1.setCursor(0, 2);
    lcd1.print("                    ");
    lcd1.setCursor(0, 3);
    lcd1.print(" Press # for Hole 1 ");
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    G00 (62,26.5,0);
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    LCD2_UpdateDelay(1);
    lcd1.setCursor(0, 0);
    lcd1.print("FIT 2.5mm DRILL BIT ");
    lcd1.setCursor(0, 1);
    lcd1.print("Drill to 17mm Depth ");
    lcd1.setCursor(0, 2);
    lcd1.print("                    ");
    lcd1.setCursor(0, 3);
    lcd1.print(" Press # for Hole 2 ");
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    G00 (92,26.5,0);
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }    
  }

  if (menuSelect == 9 && KeyPadRetrieve() == 5) { // 1 Pass 40 Mill Shell Mill to Face Off Sides
    while (KeyPadRetrieve() < 12){
    }
    LCD2_UpdateDelay(1);
    lcd1.setCursor(0, 0);
    lcd1.print("FIT 40mm SHELL MILL ");
    lcd1.setCursor(0, 1);
    lcd1.print("  1mm/s 0.5mm Drop  ");
    lcd1.setCursor(0, 2);
    lcd1.print("-45 to +145mm X-Axis");
    lcd1.setCursor(0, 3);
    lcd1.print(" Press # GOTO START ");
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    G00 (-45,-5,0);
    lcd1.setCursor(0, 0);
    lcd1.print("   FACE OFF 1 PASS  ");
    lcd1.setCursor(0, 1);
    lcd1.print("                    ");
    lcd1.setCursor(0, 3);
    lcd1.print("                    ");
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    G01 (50,-5,0,2);
    G00 (50,30,0);
    G01 (-45,30,0,2);
    G00 (-45,0,0);
  }

  if (menuSelect == 9 && KeyPadRetrieve() == 6) { // Pixel Feature Head Sides
    while (KeyPadRetrieve() < 12){
    }
    LCD2_UpdateDelay(1);
    lcd1.setCursor(0, 0);
    lcd1.print("FIT SMALL SPOT DRILL");
    lcd1.setCursor(0, 1);
    lcd1.print("                    ");
    lcd1.setCursor(0, 2);
    lcd1.print(" Press # to Move to ");
    lcd1.setCursor(0, 3);
    lcd1.print("       START        ");
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    G00 (-4,-4,0);
    lcd1.setCursor(0, 0);
    lcd1.print("PIXEL FEAT HEAD SIDE");
    lcd1.setCursor(0, 1);
    lcd1.print("                    ");
    lcd1.setCursor(0, 2);
    lcd1.print("     Press # to     ");
    lcd1.setCursor(0, 3);
    lcd1.print("        RUN         ");
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    G00 (5.88,-4,0);
    G01 (5.88,48,0,1);
    G00 (11.25,48,0);
    G01 (11.25,-4,0,1);
    G00 (16.63,-4,0);
    G01 (16.63,48,0,1);
    G00 (22,48,0);
    G01 (22,-4,0,1);
    G00 (27.38,-4,0);
    G01 (27.38,48,0,1);
    G00 (32.75,48,0);
    G01 (32.75,-4,0,1);
    G00 (38.13,-4,0);
    G01 (38.13,48,0,1);
    G00 (-4,48,0);
    G00 (-4,5.88,0);
    G01 (48,5.88,0,1);
    G00 (48,11.25,0);
    G01 (-4,11.25,0,1);
    G00 (-4,16.63,0);
    G01 (48,16.63,0,1);
    G00 (48,22,0);
    G01 (-4,22,0,1);
    G00 (-4,27.38,0);
    G01 (48,27.38,0,1);
    G00 (48,32.75,0);
    G01 (-4,32.75,0,1);
    G00 (-4,38.13,0);
    G01 (48,38.13,0,1);
    G00 (48,48,0);
    G00 (48,-4,0);
    G00 (-4,-4,0);
  }

  if (menuSelect == 9 && KeyPadRetrieve() == 7) { // Chamfer side edges, setup each one individually
    while (KeyPadRetrieve() < 12){
    }
    LCD2_UpdateDelay(1);
    lcd1.setCursor(0, 0);
    lcd1.print("FIT 6mm CHAMFER TOOL");
    lcd1.setCursor(0, 1);
    lcd1.print("                    ");
    lcd1.setCursor(0, 2);
    lcd1.print(" Press # to Move to ");
    lcd1.setCursor(0, 3);
    lcd1.print("       START        ");
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    G00 (1,-4,0);
    lcd1.setCursor(0, 0);
    lcd1.print(" CHAMFER EDGE HEAD  ");
    lcd1.setCursor(0, 1);
    lcd1.print("                    ");
    lcd1.setCursor(0, 2);
    lcd1.print("     Press # to     ");
    lcd1.setCursor(0, 3);
    lcd1.print("        RUN         ");
    while (KeyPadRetrieve() < 12){
    }
    while (KeyPadRetrieve() != 11){
    }
    G01 (1,50,0,0.5);
    G00 (2,50,0);
    G00 (2,-4,0);
    G00 (1,-4,0);
  }
  
  if (menuSelect == 9 && KeyPadRetrieve() == 10) {
    while (KeyPadRetrieve() < 12){
    }
    lCD2_UpdatePause = false;
    menuSelect = 6;
  }
   
} //Void loop END





//========================================================================================\/ G CODE COMMANDS \/======================================================================

void G00 (float xDestination_mm, float yDestination_mm, float zDestination_mm){
  Serial.println("---G00---");
  GlobalMovemm (250000, xDestination_mm, 250000, yDestination_mm, 0, zDestination_mm, true);
}

void G01 (float xDestination_mm, float yDestination_mm, float zDestination_mm, float feedSpeed_mms){
  Serial.println("---G01---");
  xyzSpeed = feedSpeed_mms * 16384;
  LineMill (xyzSpeed, xDestination_mm, xyzSpeed, yDestination_mm, xyzSpeed, zDestination_mm);
}

void G02 (double xDestination_mm, double yDestination_mm, double zDestination_mm, double iCenter_mm, double jCenter_mm, double kCenter_mm, float feedSpeed_mms){
  Serial.println("---G02---");
  double xDiffStart = 0.0d;
  double yDiffStart = 0.0d;
  double xDiffEnd = 0.0d;
  double yDiffEnd = 0.0d;
  double radius = 0.0d;
  double radiusStart = 0.0d;
  double radiusEnd = 0.0d;
  double radiusSq = 0.0d;
  double startRadians = 0.0d;
  double endRadians = 0.0d;
  double xEndOfArc_mm = 0.0d;
  double yEndOfArc_mm = 0.0d;
  long feedSpeed = 0;

  // Calculates the Current Position in mm
  xDiffStart = (double) xTab_m1 + (double) xDisplayOffset;
  yDiffStart = (double) yTab_m0 + (double) yDisplayOffset;
  Serial.print("xDiffStart a = ");
  Serial.print(xDiffStart, 7);
  Serial.print("  yDiffStart a = ");
  Serial.print(yDiffStart, 7);
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
  Serial.print("  xDiffEnd a = ");
  Serial.print(xDiffEnd);
  Serial.print("  yDiffEnd a = ");
  Serial.println(yDiffEnd);
  xEndOfArc_mm = xDiffEnd;
  yEndOfArc_mm = yDiffEnd;
  endRadians = atan2 (yDiffEnd,xDiffEnd);
  xDiffEnd = sq(xDiffEnd);
  yDiffEnd = sq(yDiffEnd);
  radiusSq = xDiffEnd + yDiffEnd;
  radiusEnd = sqrt(radiusSq);

  feedSpeed = feedSpeed_mms * 16384;
  
  Serial.print("xDiffStart = ");
  Serial.print(xDiffStart);
  Serial.print("  yDiffStart = ");
  Serial.print(yDiffStart);
  Serial.print("  xDiffEnd = ");
  Serial.print(xDiffEnd);
  Serial.print("  yDiffEnd = ");
  Serial.print(yDiffEnd);
  Serial.print("  radiusStart = ");
  Serial.print(radiusStart, 7);
  Serial.print("  radiusEnd = ");
  Serial.print(radiusEnd, 7);
  Serial.print("  startRadians = ");
  Serial.print(startRadians, 7);
  Serial.print("  endRadians = ");
  Serial.println(endRadians, 7);
  
  // Checks the Arc movement is in line with the last move (G01, then G02)
  if (radiusStart < radiusEnd + 0.0001 || radiusStart > radiusEnd - 0.0001){
    Serial.println("RADIUS MATCH");
    radius = radiusStart;
    ArcMillG02 (xEndOfArc_mm, yEndOfArc_mm, iCenter_mm, jCenter_mm, startRadians, endRadians, radius, feedSpeed);
  }else{
    Serial.println("ERROR: Start and End Radia do not match");
  }
}

void G03 (double xDestination_mm, double yDestination_mm, double zDestination_mm, double iCenter_mm, double jCenter_mm, double kCenter_mm, float feedSpeed_mms){
  Serial.println("---G03---");
  double xDiffStart = 0.0d;
  double yDiffStart = 0.0d;
  double xDiffEnd = 0.0d;
  double yDiffEnd = 0.0d;
  double radius = 0.0d;
  double radiusStart = 0.0d;
  double radiusEnd = 0.0d;
  double radiusSq = 0.0d;
  double startRadians = 0.0d;
  double endRadians = 0.0d;
  double xEndOfArc_mm = 0.0d;
  double yEndOfArc_mm = 0.0d;
  long feedSpeed = 0;

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

  feedSpeed = feedSpeed_mms * 16384;
    
  Serial.print("xDiffStart = ");
  Serial.print(xDiffStart);
  Serial.print("  yDiffStart = ");
  Serial.print(yDiffStart);
  Serial.print("  xDiffEnd = ");
  Serial.print(xDiffEnd);
  Serial.print("  yDiffEnd = ");
  Serial.print(yDiffEnd);
  Serial.print("  radiusStart = ");
  Serial.print(radiusStart, 7);
  Serial.print("  radiusEnd = ");
  Serial.print(radiusEnd, 7);
  Serial.print("  startRadians = ");
  Serial.print(startRadians, 7);
  Serial.print("  endRadians = ");
  Serial.println(endRadians, 7);
  
  // Checks the Arc movement is in line with the last move (G01, then G02)
  if (radiusStart < radiusEnd + 0.0001 || radiusStart > radiusEnd - 0.0001){
    Serial.println("RADIUS MATCH");
    radius = radiusStart;
    ArcMillG03 (xEndOfArc_mm, yEndOfArc_mm, iCenter_mm, jCenter_mm, startRadians, endRadians, radius, feedSpeed);
  }else{
    Serial.println("ERROR: Start and End Radia do not match");
  }
}

//========================================================================================/\ G CODE COMMANDS /\======================================================================

void ArcMillG02 (double xEndOfArc_mm, double yEndOfArc_mm, double iCentre_mm, double jCentre_mm, double startRadians, double endRadians, double radius, long feedSpeed){

  double sin_n = 0.0d;
  double cos_n = 0.0d;
  double radStep = 0.0d;
  long xCirPos_m1 = 0;
  long yCirPos_m0 = 0;
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
  xCirPos_m1 = xCirPos_m1 - xDisplayOffset;
  yCirPos_m0 = yCirPos_m0 - yDisplayOffset;

  CircleMillDisplay();

  radStep = -PI * feedSpeed * 0.00000022 / radius; // 0.00000022 is negative with -Pi*2 for reverse circle at the moment, 0.00000022 is the factor for radian steps to set speed in mm/s (tweak to adjust)
  
  if (feedSpeed < 32768){
    feedSpeed = 32768;
  }

  Serial.print("xCirPos_m1 = ");
  Serial.print(xCirPos_m1);
  Serial.print("  yCirPos_m0 = ");
  Serial.print(yCirPos_m0);
  Serial.print("  radius = ");
  Serial.print(radius);
  Serial.print("  radStep = ");
  Serial.print(radStep, 5);
  Serial.print("  phase = ");
  Serial.print(phase, 5);
  Serial.print("  startRadians = ");
  Serial.print(startRadians, 5);
  Serial.print("  endRadians = ");
  Serial.println(endRadians, 5);

  lcdRefreshTimeCir = millis();
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
    xCirPos_m1 = xCirPos_m1 + cos_cps;
    yCirPos_m0 = yCirPos_m0 + sin_cps;
    GlobalMove(feedSpeed,xCirPos_m1,feedSpeed,yCirPos_m0,0,0,firstCircleLoop);
    firstCircleLoop = false;
    xCirPos_m1 = xCirPos_m1 - cos_cps;
    yCirPos_m0 = yCirPos_m0 - sin_cps;
    
  //Serial.print("phase = ");
  //Serial.print(phase, 10);
  //Serial.print("  endRadians = ");
  //Serial.println(endRadians, 10);
    if(lcdRefreshTimeCir + 100 < millis()){
      lcdRefreshTimeCir = millis();
      lcd1.setCursor(13, 1);
      lcd1.print(cos(phase), 3);
      lcd1.setCursor(13, 2);
      lcd1.print(sin(phase), 3);
      lcd1.setCursor(14, 3);
      lcd1.print(phase / 6.2831 ,3);
    }
  }
  Serial.println("FINISH");
}




void ArcMillG03 (double xEndOfArc_mm, double yEndOfArc_mm, double iCentre_mm, double jCentre_mm, double startRadians, double endRadians, double radius, long feedSpeed){

  double sin_n = 0.0d;
  double cos_n = 0.0d;
  double radStep = 0.0d;
  long xCirPos_m1 = 0;
  long yCirPos_m0 = 0;
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

  CircleMillDisplay();

  radStep = PI * feedSpeed * 0.00000022 / radius; // 0.00000022 is negative with -Pi*2 for reverse circle at the moment, 0.00000022 is the factor for radian steps to set speed in mm/s (tweak to adjust)
  
  if (feedSpeed < 32768){
    feedSpeed = 32768;
  }

  Serial.print("xCirPos_m1 = ");
  Serial.print(xCirPos_m1);
  Serial.print("  yCirPos_m0 = ");
  Serial.print(yCirPos_m0);
  Serial.print("  radius = ");
  Serial.print(radius);
  Serial.print("  radStep = ");
  Serial.print(radStep, 5);
  Serial.print("  phase = ");
  Serial.print(phase, 5);
  Serial.print("  startRadians = ");
  Serial.print(startRadians, 5);
  Serial.print("  endRadians = ");
  Serial.println(endRadians, 5);

  lcdRefreshTimeCir = millis();
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
    xCirPos_m1 = xCirPos_m1 + cos_cps;
    yCirPos_m0 = yCirPos_m0 + sin_cps;
    GlobalMove(feedSpeed,xCirPos_m1,feedSpeed,yCirPos_m0,0,0,firstCircleLoop);
    firstCircleLoop = false;
    xCirPos_m1 = xCirPos_m1 - cos_cps;
    yCirPos_m0 = yCirPos_m0 - sin_cps;
    
  //Serial.print("phase = ");
 // Serial.print(phase, 10);
 // Serial.print("  endRadians = ");
  //Serial.println(endRadians, 10);
    if(lcdRefreshTimeCir + 100 < millis()){
      lcdRefreshTimeCir = millis();
      lcd1.setCursor(13, 1);
      lcd1.print(cos(phase), 3);
      lcd1.setCursor(13, 2);
      lcd1.print(sin(phase), 3);
      lcd1.setCursor(14, 3);
      lcd1.print(phase / 6.2831 ,3);
    }
  }
  Serial.println("FINISH");
}

//____________________________________________________ODRIVE FULL CALIBRATION FUNCTION______________________________________________

void FullCalibration(int c){
    
  int motornum = c;
  int requested_state;

  lcd1.setCursor(0, 0);
  lcd1.print("ODrive______________");
  lcd1.setCursor(0, 2);
  lcd1.print("  FULL CALIBRATION  ");
  lcd1.setCursor(0, 3);
  lcd1.print("      SEQUENCE      ");
  if (motornum == 0){
    lcd1.setCursor(0, 1);
    lcd1.print("Y-Axis              ");
  }else{
    if (motornum == 1){
      lcd1.setCursor(0, 1);
      lcd1.print("X-Axis              ");
    }
  }
  requested_state = ODriveArduino::AXIS_STATE_FULL_CALIBRATION_SEQUENCE;
  odrive.run_state(motornum, requested_state, false);

  lcd1.blink();
  for (int prgrs = 6; prgrs < 20; prgrs++){
    if (prgrs == 9){
      lcd1.setCursor(7, 1);
      lcd1.print("BEEP!");
      delay(200);
      Serial.println("Calibration");
    }
    
    if (prgrs == 10){
      lcd1.setCursor(7, 1);
      lcd1.print("     ");
    }
    
    lcd1.setCursor(prgrs, 0);
    delay(1000);
  }
  
  lcd1.noBlink();
  lcd1.setCursor(17, 0);
  lcd1.print("   ");
  lcd1.setCursor(0, 3);
  lcd1.print("                    ");
  lcd1.setCursor(0, 2);
  lcd1.print("CLOSED LOOP CONTROL ");
  
  requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
  odrive.run_state(motornum, requested_state, false); // don't wait
  delay(10);

  EncoderPosition();
  switch (motornum){
    case 0:
      yTab_m0_New = yTab_m0 - yTab_m0 - yTab_m0;
      odrive.SetPosition(0, yPos_m0);
      EncoderPosition();
      LCD2_Update();
      lcd1.setCursor(0, 0);
      lcd1.print("                    ");
      lcd1.setCursor(0, 1);
      lcd1.print("                    ");
      lcd1.setCursor(0, 2);
      lcd1.print("                    ");
      oDrive0m0CLC = true;
      eStopPressed = false;
      break;
      
    case 1:
      xTab_m1 = 0;
      xTab_m1_New = xEnc_m1;
      odrive.SetPosition(1, xPos_m1);
      EncoderPosition();
      LCD2_Update();
      lcd1.setCursor(0, 0);
      lcd1.print("                    ");
      lcd1.setCursor(0, 1);
      lcd1.print("                    ");
      lcd1.setCursor(0, 2);
      lcd1.print("                    ");
      oDrive0m1CLC = true;
      eStopPressed = false;
      break;

    default:
      break;
  }
}

//____________________________________________________Stop all ODrive Motors (make IDLE)____________________________________________

void AllStop(void){
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
}

//____________________________________________Setup a Circle to Mill using Sin, Cos _________________________________OOO______

void CircleMill(void){
  while (KeyPadRetrieve() < 12){
  }
  bool circleDisplay = false;
  int xyzSelect = 0;
  bool case0FirstTime = true;
  long cirSetSpeed = 32768;
  long xCirPos_m1 = 0;
  long yCirPos_m0 = 0;

  matrix.clear();
  matrix.drawBitmap(0, 0, millCircle, 8, 8, LED_ON);
  matrix.writeDisplay();
  
  Serial.println("Mill an internal or external circle");

  while (KeyPadRetrieve() != 10){
    if (circleDisplay == false){
      lcd1.setCursor(0, 0);
      lcd1.print("Set param's   *:Back");
      lcd1.setCursor(0, 1);
      lcd1.print("1: Tool    2: Circle");
      lcd1.setCursor(0, 2);
      lcd1.print("3: Speed   4: Zero  ");
      lcd1.setCursor(0, 3);
      lcd1.print("S: Run Circle Mill  ");
      if (toolSet == true){
        lcd1.setCursor(2, 1);
        lcd1.print((char)B11111111);
      }
      if (circleSet == true){
        lcd1.setCursor(13, 1);
        lcd1.print((char)B11111111);
      }
      if (speedSet == true){
        lcd1.setCursor(2, 2);
        lcd1.print((char)B11111111);
      }
      if (zeroSet == true){
        lcd1.setCursor(13, 2);
        lcd1.print((char)B11111111);
      }
      circleDisplay = true;
    }

    switch (KeyPadRetrieve()){
      case 1:
        lcd1.setCursor(0, 0);
        lcd1.print("      Set Tool      ");
        lcd1.setCursor(0, 1);
        lcd1.print("1: Dia =  mm        ");
        lcd1.setCursor(0, 2);
        lcd1.print("2: Climb/Conventionl");
        lcd1.setCursor(0, 3);
        lcd1.print("*: Back             ");
        if (toolDiaSet == true){
          lcd1.setCursor(9, 1);
          lcd1.print(toolDiameter, 2);
          lcd1.print(" mm   ");
        }
        if (climbSet == true){
          if (climbMill == true){
            lcd1.setCursor(3, 2);
            lcd1.print("Climb Mill       ");
          }else{
            lcd1.setCursor(3, 2);
            lcd1.print("Conventional Mill");             
          }
        }
        
        while (KeyPadRetrieve() < 12){
        }
        while (KeyPadRetrieve() != 10){
          switch (KeyPadRetrieve()){
            case 1:
              KeypadEnterValue(true, 9, 1);
              lcd1.print(" mm");
              toolDiameter = keypadEnteredValue;
              Serial.print("Tool Diameter = ");
              Serial.print(toolDiameter);
              Serial.println("mm");
              toolDiaSet = true;
              break;
            case 2:
              while (KeyPadRetrieve() < 12){
              }
              if (climbMill == true){
                climbMill = false;
                lcd1.setCursor(3, 2);
                lcd1.print("Conventional Mill");
                Serial.println("Conventional Mill");
              }else{
                climbMill = true;
                lcd1.setCursor(3, 2);
                lcd1.print("Climb Mill       "); 
                Serial.println("Climb Mill");                 
              }
              climbSet = true;
              break;
            default:
              break;
          }
        }
        circleDisplay = false;
        if (toolDiaSet == true && climbSet == true){
          toolSet = true;
        }
        while (KeyPadRetrieve() < 12){
        }
        break;
      case 2:
        lcd1.setCursor(0, 0);
        lcd1.print("Set Circle   *: Back");
        lcd1.setCursor(0, 1);
        lcd1.print("1: Internal/External");
        lcd1.setCursor(0, 2);
        lcd1.print("2: Diameter mm      ");
        lcd1.setCursor(0, 3);
        lcd1.print("3: Depth mm         ");
        if (intSet == true){
          if (internalCircle == true){
            lcd1.setCursor(3, 1);
            lcd1.print("Internal Cut     ");
          }else{
            lcd1.setCursor(3, 1);
            lcd1.print("External Cut     ");                 
          }
        }
        if (circleDiaSet == true){
          lcd1.setCursor(6, 2);
          lcd1.print(" =            ");
          lcd1.setCursor( 9, 2);
          lcd1.print(circleDiameter, 2);
          lcd1.print(" mm   ");
        }
        if (depSet == true){
          lcd1.setCursor(6, 3);
          lcd1.print(" =            ");
          lcd1.setCursor(9, 3);
          lcd1.print(circleDepth, 2);
          lcd1.print(" mm");
        }
        while (KeyPadRetrieve() < 12){
        }
        while (KeyPadRetrieve() != 10){
          switch (KeyPadRetrieve()){
            case 1:
              while (KeyPadRetrieve() < 12){
              }
              if (internalCircle == true){
                internalCircle = false;
                lcd1.setCursor(3, 1);
                lcd1.print("External Cut     "); 
                Serial.println("External Cut");
              }else{
                internalCircle = true;
                lcd1.setCursor(3, 1);
                lcd1.print("Internal Cut     ");
                Serial.println("Internal Cut");              
              }
              intSet = true;
              break;
            case 2:
              lcd1.setCursor(6, 2);
              lcd1.print(" =            ");
              KeypadEnterValue(true, 9, 2);
              lcd1.print(" mm   ");
              circleDiameter = keypadEnteredValue;
              Serial.print("Circle Diameter = ");
              Serial.print(circleDiameter);
              Serial.println("mm");
              circleDiaSet = true;
              break;
            case 3:
              lcd1.setCursor(6, 3);
              lcd1.print(" =            ");
              KeypadEnterValue(true, 9, 3);
              lcd1.print(" mm");
              circleDepth = keypadEnteredValue;
              Serial.print("Circle Depth = ");
              Serial.print(circleDepth);
              Serial.println("mm");
              depSet = true;
              break;
            default:
              break;
          }
        }
        if (intSet == true && circleDiaSet == true && depSet == true){
          circleSet = true;
        }
        circleDisplay = false;
        while (KeyPadRetrieve() < 12){
        }
        break;
      case 3:
        lcd1.setCursor(0, 0);
        lcd1.print("      Set Speed     ");
        lcd1.setCursor(0, 1);
        lcd1.print("Speed = mm/s        ");
        lcd1.setCursor(0, 2);
        lcd1.print("1: Enter New Value  ");
        lcd1.setCursor(0, 3);
        lcd1.print("*: Back             ");
        if (speedSet == true){
          lcd1.setCursor(8, 1);
          lcd1.print(feedSpeed, 2);
          lcd1.print(" mm/s  ");
        }
        while (KeyPadRetrieve() < 12){
        }
        while (KeyPadRetrieve() != 10){
          if (KeyPadRetrieve() == 1){
              lcd1.setCursor(8, 1);
              lcd1.print("            ");
              KeypadEnterValue(true, 8, 1);
              lcd1.print(" mm/s");
              feedSpeed = keypadEnteredValue;
              cirSetSpeed = feedSpeed * 16384;
              Serial.print("Feed Speed = ");
              Serial.print(feedSpeed);
              Serial.println("mm");
              speedSet = true;
          }
        }
        circleDisplay = false;
        while (KeyPadRetrieve() < 12){
        }
        break;
      case 4:
        lcd1.setCursor(0, 0);
        lcd1.print("Jog & Zero    *:Back");
        lcd1.setCursor(0, 1);
        lcd1.print("X+:X-: Y+:Y-: Z+:Z-:");
        lcd1.setCursor(0, 2);
        lcd1.print("Rot:X,Y,Z     0:Zero");
        lcd1.setCursor(0, 3);
        lcd1.print("-: +:               ");
        xJogPos_m1 = xTab_m1;
        yJogPos_m0 = yTab_m0;
        xfirstLoop = true;
        yfirstLoop = true;
        while (KeyPadRetrieve() != 10){
          if (KeyPadRetrieve() == -2){
            xyzSelect++;
            while (KeyPadRetrieve() < 12){
            }
            if (xyzSelect > 3){
              xyzSelect = 0;
            }
          }
          EncoderPosition();
          LCD2_Update();
          lcd1.setCursor(6, 3);
          lcd1.print(mmJogIncr, 3);
          lcd1.print("mm ");          
          switch (KeyPadRetrieve()){
            case -4:
              while(KeyPadRetrieve() < 12){
              }
              mmJogIncr = mmJogIncr / 10;
              if (mmJogIncr < 0.001){
                mmJogIncr = 0.001;
              }
              jogIncr = mmJogIncr * 16384;
              break;
            case -5:
              while(KeyPadRetrieve() < 12){
              }
              yJogPos_m0 = yTab_m0 - jogIncr;
              GlobalMove(jogSpeed,xJogPos_m1,jogSpeed,yJogPos_m0,0,0,yfirstLoop);
              yfirstLoop = false;
              break;
            case -6:
              while(KeyPadRetrieve() < 12){
              }
              break;
            case -7:
              while(KeyPadRetrieve() < 12){
              }
              xJogPos_m1 = xTab_m1 - jogIncr;
              GlobalMove(jogSpeed,xJogPos_m1,jogSpeed,yJogPos_m0,0,0,xfirstLoop);
              xfirstLoop = false;
              break;
            case -9:
              while(KeyPadRetrieve() < 12){
              }
              xJogPos_m1 = xTab_m1 + jogIncr;
              GlobalMove(jogSpeed,xJogPos_m1,jogSpeed,yJogPos_m0,0,0,xfirstLoop);
              xfirstLoop = false;
              break;
            case -10:
              while(KeyPadRetrieve() < 12){
              }
              mmJogIncr = mmJogIncr * 10;
              if (mmJogIncr > 10){
                mmJogIncr = 10;
              }
              jogIncr = mmJogIncr * 16384;
              break;
            case -11:
              while(KeyPadRetrieve() < 12){
              }
              break;
            case -12:
              while(KeyPadRetrieve() < 12){
              }
              yJogPos_m0 = yTab_m0 + jogIncr;
              GlobalMove(jogSpeed,xJogPos_m1,jogSpeed,yJogPos_m0,0,0,yfirstLoop);
              yfirstLoop = false;
              break;
            
          }
          switch (xyzSelect){
            default:
              EncoderPosition();
              LCD2_Update();
              break;
            case 0:                        // X,Y & Z-Axes Selected
              if (case0FirstTime == true){
                lcd2.noBlink();
                lcd1.blink();
                lcd1.setCursor(15, 3);
                lcd1.print("X,Y&Z");
                lcd1.setCursor(18, 3);
              case0FirstTime = false;
              }
              if (KeyPadRetrieve() == 0){
                while (KeyPadRetrieve() < 12){
                }
                xDisplay = 0;
                xDisplayOffset = xDisplay - xTab_m1;
                yDisplay = 0;
                yDisplayOffset = yDisplay - yTab_m0;
                EncoderPosition();
                LCD2_Update();
                xZero = true;
                yZero = true;
                zZero = true;
              }
              break;
            case 1:                        // X-Axis Selected
              case0FirstTime = true;
              lcd2.setCursor(0, 1);
              lcd2.blink();
              lcd1.setCursor(13, 3);
              lcd1.print("       ");
              lcd1.noBlink();
              if (KeyPadRetrieve() == 0){
                while (KeyPadRetrieve() < 12){
                }
                xDisplay = 0;
                xDisplayOffset = xDisplay - xTab_m1;
                EncoderPosition();
                LCD2_Update();
                xZero = true;
              }
              break;
            case 2:                        // Y-Axis Selected
              case0FirstTime = true;
              lcd2.setCursor(0, 2);
              lcd2.blink();
              lcd1.setCursor(13, 3);
              lcd1.print("       ");
              lcd1.noBlink();
              if (KeyPadRetrieve() == 0){
                while (KeyPadRetrieve() < 12){
                }
                yDisplay = 0;
                yDisplayOffset = yDisplay - yTab_m0;
                EncoderPosition();
                LCD2_Update();
                yZero = true;
              }
              break;
            case 3:                        // Z-Axis Selected
              case0FirstTime = true;
              lcd2.setCursor(0, 3);
              lcd2.blink();
              lcd1.setCursor(13, 3);
              lcd1.print("       ");
              lcd1.noBlink();
              zZero = true;
              break;
          }
        }
        if (xZero == true && yZero == true && zZero == true){
          zeroSet = true;
        }
        circleDisplay = false;
        while (KeyPadRetrieve() < 12){
        }
        lcd1.noBlink();
        break;
      case -8:
        xCirPos_m1 = xJogPos_m1 + xDisplayOffset;
        yCirPos_m0 = yJogPos_m0 + yDisplayOffset;
        xCirPos_mm = (float) xCirPos_m1 / 16384;
        yCirPos_mm = (float) yCirPos_m0 / 16384;
        RunCircleMillmm (cirSetSpeed, xCirPos_mm,yCirPos_mm,circleDiameter,toolDiameter,internalCircle,circleDepth);
        circleDisplay = false;
        delay(1000);
        break;
    }
  }
  lcd1.setCursor(0, 0);
  lcd1.print("                    ");
}

//____________________________________________________ENCODER POSITION________________________________________________________

void EncoderPosition(void){

  odrive_serial << "r axis" << 1 << ".encoder.pos_estimate " << '\n';
  xEnc_m1 = odrive.readFloat();
  if (xBacklashIn == true){
    xDisplay = xEnc_m1 - xBacklash + xDisplayOffset;
    x_mmDisplay = (float)xDisplay / 16384 ;
  }else{
    xDisplay = xEnc_m1 + xDisplayOffset;
    x_mmDisplay = (float)xDisplay / 16384;
  }
  
  odrive_serial << "r axis" << 0 << ".encoder.pos_estimate " << '\n';
  yEnc_m0 = odrive.readFloat();
  if (yBacklashIn == true){
    yDisplay = yEnc_m0 - yBacklash + yDisplayOffset;
    y_mmDisplay = (float)yDisplay / 16384 ;
  }else{
    yDisplay = yEnc_m0 + yDisplayOffset;
    y_mmDisplay = (float)yDisplay / 16384;
  }
}

//______________________________________________________LCD 2 UPDATE WITH DELAY_________________________________________

// Call this function to refresh the full Right LCD (lcd2) display, update the values and
//   to provide a delay - e.g. LCD2_UpdateDelay(100); 

void LCD2_UpdateDelay(int dispDelay){
count = 100;

  // Main Right Display Layout
  
  lcd2.setCursor(0, 0);
  lcd2.print("Spindle Speed:  Idle");
  lcd2.setCursor(0, 1);
  lcd2.print("X:                  ");
  lcd2.setCursor(0, 2);
  lcd2.print("Y:                  ");
  lcd2.setCursor(0, 3);
  lcd2.print("Z: 0.0000 mm    Idle");

  // X-Axis Update
  
  if(oDrive0m1CLC == true){
    lcd2.setCursor(17, 1);
    lcd2.print("CLC");
  }else{
    lcd2.setCursor(16, 1);
    lcd2.print("Idle");
  }
  
  // Y-Axis Update
  
  if(oDrive0m0CLC == true){
    lcd2.setCursor(17, 2);
    lcd2.print("CLC");
  }else{
    lcd2.setCursor(16, 2);
    lcd2.print("Idle");
  }
  
  if (dispDelay > 0){
    while (dispDelay > 0){
      if (count > 99){
        EncoderPosition();
        LCD2_Update();
        count = 0;
      }
      count++;
      dispDelay--;
      delay(1);
    }
  }
}

void LCD2_Update(void){

  lcd2.setCursor(3, 1);
  lcd2.print(x_mmDisplay, 4);
  lcd2.print(" mm"); 

  lcd2.setCursor(3, 2);
  lcd2.print(y_mmDisplay, 4);
  lcd2.print(" mm");
}

void Dev255Rotate(void){
  if(dev255LoopTime1 + 10000 < millis()){
    if(dev255LoopTime2 + 100 < millis() && dev255LoopCount < 11){
      matrix.clear();
      matrix.setRotation(dev255LoopCount);
      matrix.drawBitmap(0, 0, dev255, 8, 8, LED_ON);
      matrix.writeDisplay();
      dev255LoopTime2 = millis();
      dev255LoopCount++;
      if(dev255LoopCount > 10){
        dev255LoopCount = 2;
        dev255LoopTime1 = millis();
      }
    }
  }
}



//________________________________________________________________CIRCLE MILL in mm_______________________________OOO____

void RunCircleMillmm (long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles){
  unsigned long circleLoopTime = 0;
  float sin_n = 0.0f;
  float cos_n = 0.0f;
  float radStep = 0.0f;
  float yRadius = 0.0f;
  float xRadius = 0.0f;
  float revCount = 0.0f;
  float radianCount = 0.0f;
  long xCirPos_m1 = 0;
  long yCirPos_m0 = 0;

  circleDiameter = diameter;
  toolDiameter = toolDiam;
  internalCircle = internal;
  circleDepth = cycles;

  xCirPos_m1 = xCentermm * 16384;
  yCirPos_m0 = yCentermm * 16384;
  xCirPos_m1 = xCirPos_m1 - xDisplayOffset;
  yCirPos_m0 = yCirPos_m0 - yDisplayOffset;
  
  if (internalCircle == true){
    yRadius = circleDiameter - toolDiameter;
    yRadius = yRadius / 2;
    xRadius = circleDiameter - toolDiameter;
    xRadius = xRadius / 2;
  }else{
    yRadius = circleDiameter + toolDiameter;
    yRadius = yRadius / 2;
    xRadius = circleDiameter + toolDiameter;
    xRadius = xRadius / 2;
  }
  if (climbMill == true){
    yRadius = -yRadius;
    xRadius = -xRadius;
  }
  revCount = circleDepth;                   // multiply for more rotations, 1 = 1 cycle, possibly change from circleDepth to cycles
  radianCount = revCount * -6.2831853;
  circleLoopTime = millis();
  CircleMillDisplay();
  
  bool firstCircleLoop = true;
  long sin_cps = 0;
  long cos_cps = 0;
  float phase = 0.0f;

  radStep = 3.14159265 * (float)cirSpeed * -0.00000011 / diameter; // -0.00000012 is negative with - Pi*2 for reverse circle at the moment, 0.00000012 is the factor for radian steps to set speed in mm/s (tweak to adjust)
  
  if (cirSpeed < 32768){ // Put in as Global move doesn't yet allow for speeds below 32768, speed is controlled by radStep
    cirSpeed = 32768;
  }
  
  lcdRefreshTimeCir = millis();
  for (phase; phase > radianCount; phase += radStep) {
    cos_n = xRadius * cos(phase); // If this value is different from sin_n, then a Horizontal Oval is cut
    sin_n = yRadius * sin(phase); // A value of 8.192f is 1mm of travel & 819.2f is 100mm of travel
    cos_n = cos_n * 16384;
    sin_n = sin_n * 16384;
    cos_cps = (long) cos_n;
    sin_cps = (long) sin_n;
    xCirPos_m1 = xCirPos_m1 + cos_cps;
    yCirPos_m0 = yCirPos_m0 + sin_cps;
    GlobalMove(cirSpeed,xCirPos_m1,cirSpeed,yCirPos_m0,0,0,firstCircleLoop);
    firstCircleLoop = false;
    xCirPos_m1 = xCirPos_m1 - cos_cps;
    yCirPos_m0 = yCirPos_m0 - sin_cps;
    if(lcdRefreshTimeCir + 100 < millis()){
      lcdRefreshTimeCir = millis();
      lcd1.setCursor(13, 1);
      lcd1.print(cos(phase), 3);
      lcd1.setCursor(13, 2);
      lcd1.print(sin(phase), 3);
      lcd1.setCursor(14, 3);
      lcd1.print(phase / 6.2831 ,3);
    }
  }
}

// ---------------------------------------------------Circle Display Update for LCD 1-----------------------------------------------

void CircleMillDisplay (void){ 
  lcd1.setCursor(0, 0);
  lcd1.print("   (Circle Mill)    ");
  lcd1.setCursor(0, 1);
  lcd1.print("X Axis Cos =        ");
  lcd1.setCursor(0, 2);
  lcd1.print("Y Axis Sin =        ");
  lcd1.setCursor(0, 3);
  lcd1.print("Revolutions =       ");
}

//_____________________________________________________________________LINE MILL_____________________________________________---_____________

void LineMill (long xLMSpeed, float xLMDestination_mm, long yLMSpeedNU, float yLMDestination_mm, long zLMSpeed, long zLMDestination){ // Mill or move from one point to another with interpollation (ALL Axes reach target at same time)

  float loopTime = 0.01f; // Adjust to get the feed speed (mm/s) correct
  long loopCount = 0;
  float progress = 0.0f; // Changed to a float from long
  float xDelta_m1 = 0.0f;
  float yDelta_m0 = 0.0f;
 // long zDelta_m2 = 0;  
  float xRatio = 0.0f;
  float yRatio = 0.0f;
 // float zRatio = 0.0f:
  float xStep = 0.0f;
  float yStep = 0.0f;
 // long zStep = 0;
  float xLinPos_m1 = 0.0f; // Changed to a float from long
  float yLinPos_m0 = 0.0f; // Changed to a float from long
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
  delay (100);
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

  xDelta_m1 = (float)xLMDestination - (float)xTab_m1;
  yDelta_m0 = (float)yLMDestination - (float)yTab_m0;
  
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
    xStep = (float)xLMSpeed * loopTime;
    yStep = (float)xLMSpeed * loopTime * yRatio;
  }else{
    xDeltaPrimary = false;
    yLMSpeed = xLMSpeed;
    xLMSpeed = yLMSpeed * xRatio;
    xStep = (float)yLMSpeed * loopTime * xRatio;
    yStep = (float)yLMSpeed * loopTime;
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
    }
  }
}

//_____________________________________________________________________Global Move of all Axes in mm (Converter)__________________________________________________________

void GlobalMovemm (long xSpeedPassThrough, float xDestinationmm, long ySpeedPassThrough, float yDestinationmm, long zSpeedPassThrough, float zDestinationmm, bool firstLoopPassThrough){
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

//_____________________________________________________________________Global Move of all Axes__________________________________________________________

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
      if(lcdRefreshTime + 100 < millis()){
        LCD2_Update();
        lcdRefreshTime = millis();
      }
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
      if(lcdRefreshTime + 100 < millis()){
        LCD2_Update();
        lcdRefreshTime = millis();
      }
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
      if(lcdRefreshTime + 100 < millis()){
        LCD2_Update();
        lcdRefreshTime = millis();
      }
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
      if(lcdRefreshTime + 100 < millis()){
        LCD2_Update();
        lcdRefreshTime = millis();
      }
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
      if(lcdRefreshTime + 100 < millis()){
        LCD2_Update();
        lcdRefreshTime = millis();
      }
    }
  }
}

//________________________________________________________KEY PAD SCAN & RETRIEVE FUNCTION_____________________________________________________

int KeyPadRetrieve(void){
  int keyVal; // pinMode(keyPadY1, INPUT); pinMode(keyPadY2, INPUT); pinMode(keyPadY3, INPUT); pinMode(keyPadY4, INPUT); pinMode(keyPadX1, OUTPUT); pinMode(keyPadX2, OUTPUT); pinMode(keyPadX3, OUTPUT);
  int Y1;
  int Y2;
  int Y3;
  int Y4;
  int Y5;
  int Y6;
  int Y7;
  int Y8;
  keyVal = 12;
  
  digitalWrite(keyPadX1, LOW);
  digitalWrite(keyPadX2, HIGH);
  digitalWrite(keyPadX3, HIGH);
  Y1 = digitalRead(keyPadY1);
  Y2 = digitalRead(keyPadY2);
  Y3 = digitalRead(keyPadY3);
  Y4 = digitalRead(keyPadY4);
  Y5 = digitalRead(keyPad2Y1);
  Y6 = digitalRead(keyPad2Y2);
  Y7 = digitalRead(keyPad2Y3);
  Y8 = digitalRead(keyPad2Y4);

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
        }else{
          if(Y5 == LOW){
            keyVal = -1;
            return keyVal;
          }else{
            if (Y6 == LOW){
              keyVal = -4;
              return keyVal;
            }else{
              if(Y7 == LOW){
                keyVal = -7;
                return keyVal;
              }else{
                if(Y8 == LOW){
                  keyVal = -10;
                  return keyVal;
                }
              }
            }
          }
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
  Y5 = digitalRead(keyPad2Y1);
  Y6 = digitalRead(keyPad2Y2);
  Y7 = digitalRead(keyPad2Y3);
  Y8 = digitalRead(keyPad2Y4);

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
        }else{
          if(Y5 == LOW){
            keyVal = -2;
            return keyVal;
          }else{
            if (Y6 == LOW){
              keyVal = -5;
              return keyVal;
            }else{
              if(Y7 == LOW){
                keyVal = -8;
                return keyVal;
              }else{
                if(Y8 == LOW){
                  keyVal = -12;
                  return keyVal;
                }
              }
            }
          }
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
  Y5 = digitalRead(keyPad2Y1);
  Y6 = digitalRead(keyPad2Y2);
  Y7 = digitalRead(keyPad2Y3);
  Y8 = digitalRead(keyPad2Y4);

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
        }else{
          if(Y5 == LOW){
            keyVal = -3;
            return keyVal;
          }else{
            if (Y6 == LOW){
              keyVal = -6;
              return keyVal;
            }else{
              if(Y7 == LOW){
                keyVal = -9;
                return keyVal;
              }else{
                if(Y8 == LOW){
                  keyVal = -11;
                  return keyVal;
                }
              }
            }
          }
        }
      }
    }
  }
  return keyVal;
}

// ------------------------------------------------------Keypad Entry into LCD field-----------------------------------------------

void KeypadEnterValue (bool sign, int lcdHrzn, int lcdVrt){
  String keyPadString = " ";
  byte dec_count = 0;
  byte sign_count = 0;
  lcd1.blink();
  lcd1.setCursor(lcdHrzn, lcdVrt);
  lcd1.print("       ");
  lcd1.setCursor(lcdHrzn, lcdVrt);
  while(KeyPadRetrieve() < 12){
    delay(100);
  }
  
  while(KeyPadRetrieve() != 11){
    switch (KeyPadRetrieve()){
      case 0:
        keyPadString += (char)48;
        lcd1.print("0");
        while(KeyPadRetrieve() == 0){
        delay(100);
        }
      break;
      case 1:
        keyPadString += (char)49;
        lcd1.print("1");
        while(KeyPadRetrieve() == 1){
        delay(100);
        }
      break;
      case 2:
        keyPadString += (char)50;
        lcd1.print("2");
        while(KeyPadRetrieve() == 2){
        delay(100);
        }
      break;
      case 3:
        keyPadString += (char)51;
        lcd1.print("3");
        while(KeyPadRetrieve() == 3){
        delay(100);
        }
      break;
      case 4:
        keyPadString += (char)52;
        lcd1.print("4");
        while(KeyPadRetrieve() == 4){
        delay(100);
        }
      break;
      case 5:
        keyPadString += (char)53;
        lcd1.print("5");
        while(KeyPadRetrieve() == 5){
        delay(100);
        }
      break;
      case 6:
        keyPadString += (char)54;
        lcd1.print("6");
        while(KeyPadRetrieve() == 6){
        delay(100);
        }
      break;
      case 7:
        keyPadString += (char)55;
        lcd1.print("7");
        while(KeyPadRetrieve() == 7){
        delay(100);
        }
      break;
      case 8:
        keyPadString += (char)56;
        lcd1.print("8");
        while(KeyPadRetrieve() == 8){
        delay(100);
        }
      break;
      case 9:
        keyPadString += (char)57;
        lcd1.print("9");
        while(KeyPadRetrieve() == 9){
        delay(100);
        }
      break;
      case 10:
        if(dec_count < 1){
          keyPadString += (char)46;
          lcd1.print(".");
          dec_count++;
        }
        while(KeyPadRetrieve() == 10){
          delay(100);
        }
      break;
      case -1:
        while(KeyPadRetrieve() == -1){
        delay(100);
        }

      break;
      case -2:
        while(KeyPadRetrieve() == -2){
        delay(100);
        }
      break;
      case -3:
        keyPadString = " ";
        dec_count = 0;
        sign_count = 0;
        lcd1.setCursor(lcdHrzn, lcdVrt);
        lcd1.print("          ");
        lcd1.setCursor(lcdHrzn, lcdVrt);
        while(KeyPadRetrieve() == -3){
          delay(100);
        }

      break;
      case -4:
        if(sign_count < 1){
          keyPadString += (char)45;
          lcd1.print("-");
          sign_count++;
        }
        while(KeyPadRetrieve() == -4){
        delay(100);
        }
      break;
      case -5:
        while(KeyPadRetrieve() == -5){
        delay(100);
        }
      break;
      case -6:
        while(KeyPadRetrieve() == -6){
        delay(100);
        }
      break;
      case -7:
        while(KeyPadRetrieve() == -7){
        delay(100);
        }
      break;
      case -8:
        while(KeyPadRetrieve() == -8){
        delay(100);
        }
      break;
      case -9:
        while(KeyPadRetrieve() == -9){
        delay(100);
        }
      break;
      case -10:
        if(sign_count < 1){
          keyPadString += (char)43;
          lcd1.print("+");
          sign_count++;
        }
        while(KeyPadRetrieve() == -10){
        delay(100);
        }
      break;
      case -11:
        while(KeyPadRetrieve() == -11){
        delay(100);
        }
      break;
      case -12:
        while(KeyPadRetrieve() == -12){
        delay(100);
        }
      break;
      default:
        if(lcdRefreshTime + 100 < millis()){
          EncoderPosition();
          LCD2_Update();
          lcdRefreshTime = millis();
        }
      break;
    }
    keypadEnteredValue = keyPadString.toFloat();
  }
  while(KeyPadRetrieve() == 11){
  delay(100);
  }
  lcd1.noBlink();
}
