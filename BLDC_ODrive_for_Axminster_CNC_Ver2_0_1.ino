
/*
Neil Devonshire - Dev255
YouTube Dev255
www.dev255.co.uk

Joypad test sketch to ensure all the contols work across all range of motion
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

//----------Version Major_Minor_Patch changes-----Ver2_0_1----------

// Adding all the X-Axis code as the drive unit is now connected to ODrive 0 Motor 1.
// X-Axis added to Circle cosine control
// Added tool, circle, zero and speed parameters within circle milling function
// Ability to clear entry while entering values (had to reset the device to stop machine moving to wrong input)

//----------Things to address-----------

// Look at velocity control either here or on ODrive for CNC accuracy
// GCode test menu with GCode test (runs GCode above the work)
// Bite size chunks
// SD Card reader?
// Joypad controlled menu?
// Reverse recorded Y-Axis travel
// Jog mode frezes when coming out of CNC Mode
// Use physical microswitch endstops connected to ODrive for safety with ODrive system and connect to Arduino for program control also
// After Emergency Stop or during, reset the ODrive from the Arduino to reset the Position Counter.

//---------------------------------------------Functions---------------------------------------------
//
// int KeyPadRetrieve(void); -------------- Returns the key press value from the Key Pad. i.e. 1=1, 2=2, 3=3, etc, *=10 & #=11.
// LiveKeypad(void); ------------------ Similar to KeyPadRetrieve, although waits for the # (Enter) key to be pressed after displaying a number. Can work without displaying (silentKeyPad = true;)
// FullCalibration(void); ------------- Calibrates each motor in turn, X,Y, then Z
// CircleTest(void); ------------------ Uses Pi to draw a circle using X & Y coordinates
// JogMode(void); --------------------- Allows all axis to be jogged by 1mm, 10mm, 100mm in both directions
// ManualSoftStops(void); ------------- Sets Soft Limits to the table movement to prevent clamps crashing into the machine (aditional to hard limits(not yet configured))Used temporarily as CNC for 1 axis.
// ZeroAxes(void); -------------------- Set all Axes to zero individually or all at once
// YBacklashComp(float yTab_m0_New); -- Takes the new commanded position and tests if the direction will change, adds or removes backlash before handing control back
// AllStop (void); -------------------- Sets all motors idle (AXIS_STATE_IDLE), thereby stopping all movement
// RunCNCySoftStops(void); ------------ Set temporary Soft Stops in addition to Manual Soft Stops above to run one axis in manual or CNC control, also able to set different speeds in both directions
// EncoderPosition (float yEnc_m0); --- Sets yTmm_m0 to mm travelled from the current reported encoder position, + y_mmBacklash if true

#include <LiquidCrystal.h>
#include <ODriveArduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_LEDBackpack.h>

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
long xPos_m1 = 0;                    // Commanded position of Y-Axis Motor in encoder counts
long xEnc_m1 = 0;                    // Current Estimated encoder counts of Y-Axis rotary encoder
long xTab_m1 = 0;                    // Current table position in encoder counts
long xTab_m1_New = 0;                // New table position before Y-Axis move in encoder counts
long xDisplayOffset = 0;             // Difference between Table position yTab_m0 and displayed position for Y-Axis in encoder counts
long xDisplay = 0;                   // Displayed position of Y-Axis in encoder counts, offset from yTab_m0 when axis zeroed
float x_mmDisplay = 0;               // Displayed position of Y-Axis in mm, offset from yTab_m0 when axis zeroed
long xStopPlus = 0;                  // Software End Stop for Y-Axis Plus direction in encoder counts
long xStopMinus = 0;                 // Software End Stop for Y-Axis Minus direction in encoder counts
float xAxisDelta = 0.0f;
long xPlusFeedSpeed = 60000;         // 
long xMinusFeedSpeed = 30000;        // 
const long xSpeedLimitLow = 32768;   // Minimum speed in counts per second for ODrive stability
const long xSpeedLimitHigh = 360284; // Maximum speed in counts per second for ODrive stability

// Y-Axis variables
long yPos_m0 = 0;                    // Commanded position of Y-Axis Motor in encoder counts
long yEnc_m0 = 0;                    // Current Estimated encoder counts of Y-Axis rotary encoder
long yTab_m0 = 0;                    // Current table position in encoder counts
long yTab_m0_New = 0;                // New table position before Y-Axis move in encoder counts
long yDisplayOffset = 0;             // Difference between Table position yTab_m0 and displayed position for Y-Axis in encoder counts
long yDisplay = 0;                   // Displayed position of Y-Axis in encoder counts, offset from yTab_m0 when axis zeroed
float y_mmDisplay = 0;               // Displayed position of Y-Axis in mm, offset from yTab_m0 when axis zeroed
long yStopPlus = 0;                  // Software End Stop for Y-Axis Plus direction in encoder counts
long yStopMinus = 0;                 // Software End Stop for Y-Axis Minus direction in encoder counts
float yAxisDelta = 0.0f;
long yPlusFeedSpeed = 60000;         // 
long yMinusFeedSpeed = 30000;        // 
const long ySpeedLimitLow = 32768;   // Minimum speed in counts per second for ODrive stability
const long ySpeedLimitHigh = 360284; // Maximum speed in counts per second for ODrive stability

// Z-Axis variables

// Backlash variables
const long backLashSpeed = 300000;      // Constant - Speed for taking up backlash (all axis affected at present)

// X-Axis Backlash variables
const float x_mmBacklash = 0.25f;     // Y AXIS BACKLASH - ADJUST THIS TO SET NATURAL SLOP IN AXIS DRIVE (use actual mm measured)0.1 at slow speed and 0.0001 at high. 
const int x_BacklashDelay = 0;
const long xBacklash = 2500;          // Number of encoder counts to remove backlash, tune on table with .pos_gain from ODriveTools.
bool xBacklashIn = false;             // Set to true when Y-Axis moving in plus (count increases) direction.
bool xPreBacklash = false;            // When true takes up backlash before any operation.

// Y-Axis Backlash variables
const float y_mmBacklash = 0.25f;     // Y AXIS BACKLASH - ADJUST THIS TO SET NATURAL SLOP IN AXIS DRIVE (use actual mm measured)0.1 at slow speed and 0.0001 at high. 
const int y_BacklashDelay = 0;
const long yBacklash = 5243;          // Number of encoder counts to remove backlash, tune on table with .pos_gain from ODriveTools.
bool yBacklashIn = false;             // Set to true when Y-Axis moving in plus (count increases) direction.
bool yPreBacklash = false;            // When true takes up backlash before any operation.

// Joystick Pin config
const int topPotPin = A0;    //Joystic top potentiometer X-Axis (Left/Right) analog input pin
int topPotVal = 0;           //Current value of the top potentiometer
const int xAxisPin = A1;     //Joystic right stick X-Axis (Left/Right) analog input pin
int xAxisVal = 0;            //Current value of the X-Axis
const int yAxisPin = A2;     //Joystic right stick Y-Axis (Left/Right) analog input pin
int yAxisVal = 0;            //Current value of the Y-Axis
const int zAxisPin = A3;     //Joystic left Stick Z-Axis (Left/Right) analog input pin
int zAxisVal = 0;            //Current value of the Z-Axis
const int topSwtchPin = 7;  //Top switch digital input pin
int topSwtchVal = 0;         //Top switch value
const int redLedPin = 6;    //Red LED digital output pin
const int yellowLedPin = 5; //Red LED digital output pin

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
volatile bool oDrive0m1CLC = false; // Is ODrive 0, motor 0 (Y-Axis) in Closed Loop Control? Volatile because used in Emergency Stop ISR (interrupt)
bool endStopX = false; // Has End Stop for X been set?
bool endStopY = false; // Has End Stop for Y been set?
bool cursorOn = false; // lcd1 Cursor latch
bool keyPadEnabled = false; // Only true if keypad required
bool keyPressed = false; // Only true if keypad key pressed
bool keySeleced = false; // Only true if keypad # (enter) pressed after key pressed
bool jogMode = false; // Only true if manually jogging
bool silentKeyPad = false; // Set to true when a flashing cursor os not to be shown
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
int keyPressedVal = 12;
int keySelect = 12;

// Keypad entered values
float keypadEnteredValue = 0.0f;
int kEV_01 = 0;
int kEV_02 = 0;
float displaySpeed = 0.0f;

//Menu select
int menuSelect = 0;

float vBus = 0.0f;
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
float xJogPos_mm = 0.0;
float yJogPos_mm = 0.0;
long jogSpeed = 150000;
bool xfirstLoop = true;
bool yfirstLoop = true;

//_____________________________________________________________VOID SETUP()_________________________________________________________

void setup() {
  // Seup the Emergency Stop Interrupt
  pinMode(interruptESPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptESPin), ISR_EStop, LOW);
  
  // Set the joystick pins mode
  pinMode(topSwtchPin, INPUT); //Set digital pin 46 (called topSwitchPin for ease) to an Input
  pinMode(topSwtchPin, INPUT_PULLUP); //Turn on the pull-up resistor on this input (pulls the voltage of the input up to near 5V when Open/Circuit input)
  pinMode(redLedPin, OUTPUT); //Set digital pin 50 (called redLedPin) to an Output
  pinMode(yellowLedPin, OUTPUT); //Set digital pin 50 (called yellowLedPin) to an Output
  
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
    
  // odrive_serial << "w axis" << 0 << ".controller.config.pos_gain " << 62.0f << '\n';
  // odrive_serial << "w axis" << 0 << ".controller.config.vel_gain " << 5.0f << " / " << 10000.0f << '\n';
  // odrive_serial << "w axis" << 0 << ".controller.config.vel_integrator_gain " << 25.0f << " / " << 10000.0f << '\n';
  odrive_serial << "w axis" << 0 << ".motor.config.current_lim " << 40.0f << '\n';  // Y-Axis Motor
  odrive_serial << "w axis" << 0 << ".controller.config.vel_limit " << 36028.0f << '\n';  // Y-Axis Motor
  odrive_serial << "w axis" << 1 << ".motor.config.current_lim " << 40.0f << '\n';  // X-Axis Motor
  odrive_serial << "w axis" << 1 << ".controller.config.vel_limit " << 36028.0f << '\n';  // X-Axis Motor
  // odrive_serial << ".config.brake_resistance " << 0.5f << '\n';
  // odrive_serial << "w axis" << 0 << ".motor.config.calibration_current " << 20.0f << '\n';
  // odrive_serial << "w axis" << 0 << ".motor.config.pole_pairs " << 7.0f << '\n';
  // odrive_serial << "w axis" << 0 << ".motor.config.motor_type " << "MOTOR_TYPE_HIGH_CURRENT" << '\n';
  // odrive_serial << "w axis" << 0 << ".encoder.config.cpr " << 8192.0f << '\n'; 

  lcd1.begin(20, 4);
  analogWrite(lcd1Backlight, 255);
  lcd2.begin(20, 4);
  analogWrite(lcd2Backlight, 255);


/*
  lcd1.setCursor(0, 0);
  lcd1.print("    Hello, World    ");
  delay(1000);
  lcd1.setCursor(0, 0);
  lcd1.print("Ver 1.8.1     Dev255");
  lcd1.setCursor(0, 1);
  lcd1.print("   Latest Changes   ");
  lcd1.setCursor(0, 2);
  lcd1.print("X-Axis code added   ");
  lcd1.setCursor(0, 3);
  lcd1.print("X-Axis added to circ");
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
*/


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
  LCD2_UpdateDelay(1);

  if (oDrive0m0CLC == false && menuSelect == 0 || oDrive0m1CLC == false && menuSelect == 0){
    keyPadEnabled = true;
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

  if (menuSelect == 1) {
    keyPadEnabled = true;
    lcd1.setCursor(0, 1);
    lcd1.print("1:Crcl Cut  2:Jog   ");
    lcd1.setCursor(0, 2);
    lcd1.print("3:Pspx Cut  4:<>Stop");
    if(oDrive0m0CLC == false || oDrive0m1CLC == false){
      lcd1.setCursor(0, 3);
      lcd1.print("5:Test    6:Back  ");
    }else{
      lcd1.setCursor(0, 3);
      lcd1.print("5:Home Set 6:CLC Off");
    }
  }
 
  if (keyPadEnabled == true){
    LiveKeypad();
  }
  
  if (menuSelect == 0 && KeyPadRetrieve() == 0) {
    while (KeyPadRetrieve() == 0){
    }
    FullCalibration(0);
    if(oDrive0m0CLC == false || oDrive0m1CLC == false){
      menuSelect = 0;
    }else{
      menuSelect = 1;
    }
  }
  
  if (menuSelect == 0 && KeyPadRetrieve() == 1) {
    while (KeyPadRetrieve() == 1){
    }
    FullCalibration(1);
    if(oDrive0m0CLC == false || oDrive0m1CLC == false){
      menuSelect = 0;
    }else{
      menuSelect = 1;
    }
  }

  if (menuSelect == 0 && KeyPadRetrieve() == 5){
    while (KeyPadRetrieve() == 5){
    }
    menuSelect = 1;
    lcd1.setCursor(17, 0);
    lcd1.print("   ");
  } 
  
  if (menuSelect == 1 && keySelect == 6 && oDrive0m0CLC == false || menuSelect == 1 && keySelect == 6 && oDrive0m1CLC == false){
    keySelect = 12;
    keyPressedVal = 12;
    menuSelect = 0;
    lcd1.setCursor(17, 0);
    lcd1.print("   ");
  } 
 
  if (keySelect < 12) {
    
    if (keySelect == 1 && menuSelect == 1) {
      CircleMill();
    }

    if (keySelect == 2 && menuSelect == 1) {
      menuSelect = 2;
     // JogMode();
    }

    if (keySelect == 3 && menuSelect == 1) {
      PerspexCut(16384);
    }
    
    if (keySelect == 4 && menuSelect == 1) {
      menuSelect = 4;
      ManualSoftStops();
    }
    
    if (keySelect == 5 && menuSelect == 1) {
      menuSelect = 5;
      TestPatternEye(300000);
    }
    
    if (keySelect == 6 && menuSelect == 1 && oDrive0m0CLC == true) {
      AllStop();
    }
  }
  
  topSwtchVal = digitalRead(topSwtchPin); //Put the current value of digital pin 46 into memory location called "topSwtchVal"
  if (topSwtchVal > 0) {
    JoypadControl();
  }
}



//_______________________________________________________KEY PAD PRESS FUNCTION_____________________________________________________

void LiveKeypad(void){
  if (cursorOn == false && keyPressed == false){
    if (silentKeyPad == false){
       lcd1.setCursor(17, 0);
       lcd1.blink();
       lcd1.noAutoscroll();
       lcd1.print(" _ ");
       lcd1.setCursor(18, 0);  
    }    
    delay(100);
    cursorOn = true;
  }   
  switch (KeyPadRetrieve()){
    case 1:
      keyPressedVal = 1;
      if (silentKeyPad == false){
        lcd1.setCursor(18, 0);
        lcd1.print("1");
        lcd1.noBlink();
      }
      keyPressed = true;
      cursorOn = false;
      break;
    case 2:    
      keyPressedVal = 2;
      if (silentKeyPad == false){
        lcd1.setCursor(18, 0);
        lcd1.print("2");
        lcd1.noBlink();
      }
      keyPressed = true;
      cursorOn = false;
      break;
    case 3:
      keyPressedVal = 3;
      if (silentKeyPad == false){
        lcd1.setCursor(18, 0);
        lcd1.print("3");
        lcd1.noBlink();
      }
      keyPressed = true;
      cursorOn = false;
      break;
    case 4:
      keyPressedVal = 4;
      if (silentKeyPad == false){
        lcd1.setCursor(18, 0);
        lcd1.print("4");
        lcd1.noBlink();
      }
      keyPressed = true;
      cursorOn = false;
      break;
    case 5:
      keyPressedVal = 5;
      if (silentKeyPad == false){
        lcd1.setCursor(18, 0);
        lcd1.print("5");
        lcd1.noBlink();
      }
      keyPressed = true;
      cursorOn = false;
      break;
    case 6:
      keyPressedVal = 6;
      if (silentKeyPad == false){
        lcd1.setCursor(18, 0);
        lcd1.print("6");
        lcd1.noBlink();
      }
      keyPressed = true;
      cursorOn = false;
      break;
    case 7:
      keyPressedVal = 7;
      if (silentKeyPad == false){
        lcd1.setCursor(18, 0);
        lcd1.print("7");
        lcd1.noBlink();
      }
      keyPressed = true;
      cursorOn = false;
      break;
    case 8:
      keyPressedVal = 8;
      if (silentKeyPad == false){
        lcd1.setCursor(18, 0);
        lcd1.print("8");
        lcd1.noBlink();
      }
      keyPressed = true;
      cursorOn = false;
      break;
    case 9:
      keyPressedVal = 9;
      if (silentKeyPad == false){
        lcd1.setCursor(18, 0);
        lcd1.print("9");
        lcd1.noBlink();
      }
      keyPressed = true;
      cursorOn = false;
      break;
    case 0:
      keyPressedVal = 0;
      if (silentKeyPad == false){
        lcd1.setCursor(18, 0);
        lcd1.print("0");
        lcd1.noBlink();
      }
      keyPressed = true;
      cursorOn = false;
      break;
    case 10:
      keyPressedVal = 10;
      if (silentKeyPad == false){
        lcd1.setCursor(18, 0);
        lcd1.print("10");
        lcd1.noBlink();
      }
      keyPressed = true;
      cursorOn = false;
      break;
    case 11:
      keySelect = keyPressedVal;
      keyPressedVal = 11;
      keyPressed = false;
      cursorOn = false;
      while(KeyPadRetrieve() == 11){
      }
      break;
    default:
      cursorOn = false;
      break;
  }
}

//________________________________________________________KEY PAD SCAN FUNCTION_____________________________________________________

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



//____________________________________________________ODRIVE FULL CALIBRATION FUNCTION______________________________________________

//void FullCalibration(char c){
void FullCalibration(int c){
    
  keySelect = 12;
//  int motornum = c-'0';
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

  // This is already in void setup ... odrive_serial << "w axis" << 0 << ".controller.config.vel_limit " << 35000 << '\n';
  // delay(10);
  EncoderPosition();
  switch (motornum){
    case 0:
      // yTab_m0 = 0;
      yTab_m0_New = yTab_m0 - yTab_m0 - yTab_m0; //? if this is working
//      YBacklashComp(yTab_m0_New);
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
//      XBacklashComp(xTab_m1_New);
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
      keySelect = 12;
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
              yJogPos_m0 = yTab_m0 - jogIncr + yDisplayOffset;
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
              xJogPos_m1 = xTab_m1 - jogIncr + xDisplayOffset;
              GlobalMove(jogSpeed,xJogPos_m1,jogSpeed,yJogPos_m0,0,0,xfirstLoop);
              xfirstLoop = false;
              break;
            case -9:
              while(KeyPadRetrieve() < 12){
              }
              xJogPos_m1 = xTab_m1 + jogIncr + xDisplayOffset;
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
              yJogPos_m0 = yTab_m0 + jogIncr + yDisplayOffset;
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
        xJogPos_mm = (float) xJogPos_m1 / 16384;
        yJogPos_mm = (float) yJogPos_m0 / 16384;        
        RunCircleMillmm (cirSetSpeed, xJogPos_mm,yJogPos_mm,circleDiameter,toolDiameter,internalCircle,circleDepth);
        circleDisplay = false;
        delay(1000);
        break;
    }
  }
  menuSelect = 1;
  keySelect = 12;
  lcd1.setCursor(0, 0);
  lcd1.print("                    ");
}


//--------------------------------------------------------Run Circle Mill in mm-----------------------------------------OOO----------

void RunCircleMillmm (long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles){
  unsigned long circleLoopTime = 0;
  unsigned long elapsedTime = 0;
  float sin_n = 0.0f;
  float cos_n = 0.0f;
  float radStep = 0.0f;
  float yRadius = 0.0f;
  float xRadius = 0.0f;
  float radIncrement = 0.0f;
  float revCount = 0.0f;
  float radianCount = 0.0f;
  long xCirPos_m1 = 0;
  long yCirPos_m0 = 0;
  float xDest_mm = 0.0f;
  float yDest_mm = 0.0f;

  circleDiameter = diameter;
  toolDiameter = toolDiam;
  internalCircle = internal;
  circleDepth = cycles;

  xCirPos_m1 = (long)xCentermm * 16384;
  yCirPos_m0 = (long)yCentermm * 16384;
  xCirPos_m1 = xCirPos_m1 + xDisplayOffset;
  yCirPos_m0 = yCirPos_m0 + yDisplayOffset;
  
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
  radIncrement = 3.1415927 * circleDiameter;
  radIncrement = radIncrement / feedSpeed;
  radIncrement = 0.0062831853 / radIncrement;   // Radian increment per milli-second
  circleLoopTime = millis();
  CircleMillDisplay();
  
  bool firstCircleLoop = true;
  long sin_cps = 0;
  long cos_cps = 0;
  float phase = 0.0f;
  radStep = -0.05;
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
    //elapsedTime = millis() - circleLoopTime;
    //circleLoopTime = millis();
    // radStep = (float)elapsedTime * radIncrement;
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
  xDest_mm = (float)xCirPos_m1 / 16384;
  yDest_mm = (float)yCirPos_m0 / 16384;
  GlobalMove(cirSpeed,xCirPos_m1,cirSpeed,yCirPos_m0,0,0,true);  // Return to centre
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

//__________________________________________________________Move To Position________________________________________________________
/*----------------------------------------------------------------------------------------------------------------------------------
                                                              lcd1 DSIPLAY
                                                     ####       {Move To}      ####
                                                     #### New Pos:             ####
                                                     #### 1: Enter New Value   ####
                                                     #### #: Execute Move      #### 
*/
void MoveTo(void){
  yPreBacklash = true;
  byte loopCount = 10;
  matrix.clear();
  matrix.setRotation(2);
  matrix.drawBitmap(0, 0, moveTo, 8, 8, LED_ON);
  matrix.writeDisplay();
  lcd1.setCursor(0, 0);
  lcd1.print("(Move to &/or Zero) ");
  lcd1.setCursor(0, 1);
  lcd1.print("New Pos:            ");
  lcd1.setCursor(0, 2);
  lcd1.print("1: Enter New Value  ");
  lcd1.setCursor(0, 3);
  lcd1.print("#: Return to Menu   ");
  EncoderPosition();
  LCD2_Update();
  while(KeyPadRetrieve() != 11 && eStopPressed == false){
    if (KeyPadRetrieve() == 1){
      //lcd1.setCursor(9, 1);
      KeypadEnterValue(true, 9, 1);
      lcd1.print(" mm");
      lcd1.setCursor(0, 3);
      lcd1.print("#: Execute Move     ");
      while(KeyPadRetrieve() != 11 && eStopPressed == false){
        delay(100);
        }
      yTab_m0_New = keypadEnteredValue - yDisplayOffset;
      yTab_m0_New = yTab_m0_New * 16384;
//      YBacklashComp(yTab_m0_New);
      odrive_serial << "w axis" << 0 << ".controller.config.vel_limit " << 200000 << '\n';
      delay(100);
      odrive.SetPosition(0, yPos_m0);
      while(yPos_m0 - yEnc_m0 < -16 || yPos_m0 - yEnc_m0 > 16 && eStopPressed == false){
        if(lcdRefreshTime + 100 < millis()){
          EncoderPosition();
          LCD2_Update();
          lcdRefreshTime = millis();
        }
      }
      lcd1.setCursor(0, 1);
      lcd1.print("New Pos:            ");
      lcd1.setCursor(0, 3);
      lcd1.print("#: Return to Menu   ");
    }
    if(lcdRefreshTime + 100 < millis()){
      EncoderPosition();
      LCD2_Update();
      lcdRefreshTime = millis();
    }
  }
  menuSelect = 1;
  keySelect = 12;
  yPreBacklash = false;
}

//__________________________________________________________Manual Soft Stops_______________________________________________________
/*------------------------------------------------------Set Axis-XYZ Soft Stops-----------------------------------------------------
                                                              lcd1 DSIPLAY
                                                     ####    Soft End Stops    ####
                                                     #### 1: Y+= 0.000mm       ####
                                                     #### 2: Y-= 0.000mm       #### 
                                                     #### 0:Zero 5:XYZ  #:Next ####
*/
void ManualSoftStops (void){
  cursorOn = false;

  while(KeyPadRetrieve() == 11){
    delay(100);
  }
  matrix.clear();
  matrix.setRotation(2);
  matrix.drawBitmap(0, 0, softStopY, 8, 8, LED_ON);
  matrix.writeDisplay();
  
  MSSDispplayRefresh();
  while(KeyPadRetrieve() != 11){
    if(lcdRefreshTime + 100 < millis()){
      EncoderPosition();
      LCD2_Update();
      lcdRefreshTime = millis();
    }

    switch (KeyPadRetrieve()){
      case 0:
        yStopMinus = 0 + yDisplayOffset;
        yStopPlus = 0 + yDisplayOffset;
        MSSDispplayRefresh();
      break;
      case 1:
        lcd1.setCursor(7, 1);
        lcd1.print("             ");
        //lcd1.setCursor(7, 1);
        lcd1.blink();
        KeypadEnterValue(true, 7, 1);
        lcd1.print(" mm");
        yStopPlus = keypadEnteredValue + yDisplayOffset;
        yStopPlus = yStopPlus * 16384;
        MSSDispplayRefresh();
      break;
      case 2:
        lcd1.setCursor(7, 2);
        lcd1.print("             ");
        lcd1.setCursor(7, 2);
        lcd1.blink();
        KeypadEnterValue(true, 7, 2);
        lcd1.print(" mm");
        yStopMinus = keypadEnteredValue + yDisplayOffset;
        yStopMinus = yStopMinus * 16384;
        MSSDispplayRefresh();        
      break;
    }
  }
  while(KeyPadRetrieve() == 11){
    delay(100);
  }
  cursorOn = false;
  lcd1.noBlink();
/*-------------------------------------------------Set Feed Speed for Both Directions-----------------------------------------------
                                                              lcd1 DSIPLAY
                                                     #### ODrive ****          ####
                                                     #### Set Feed Speeds mm/s ####
                                                     #### 1: Y+= 0.00mm/s      ####
                                                     #### 2: Y-= 0.00mm/s      #### 
*/
  SFSDisplayRefresh();        
  SetFeedSpeed();
  RunCNCySoftStop();
  silentKeyPad = false;
  cursorOn = false;
  keyPressed = false;
  menuSelect = 1;
  keySelect = 12;
  lcd1.setCursor(12, 0);
  lcd1.print("        ");
  Serial.println("CNC Complete");
}



void MSSDispplayRefresh(void){
  float yStopPlusDisplay = 0.0f;
  float yStopMinusDisplay = 0.0f;
  lcd1.setCursor(0, 0);
  lcd1.print("   Soft End Stops   ");
  lcd1.setCursor(0, 1);
  lcd1.print("1: Y+=              ");
  lcd1.setCursor(0, 2);
  lcd1.print("2: Y-=              ");
  lcd1.setCursor(0, 3);
  lcd1.print("0:Zero  S:XYZ #:Next");
  lcd1.setCursor(7, 1);
  yStopPlusDisplay = yStopPlus/16384;
  yStopPlusDisplay = yStopPlusDisplay - yDisplayOffset;
  lcd1.print(yStopPlusDisplay, 4);
  lcd1.print("mm ");
  lcd1.setCursor(7, 2);
  yStopMinusDisplay = yStopMinus/16384;
  yStopMinusDisplay = yStopMinusDisplay - yDisplayOffset;
  lcd1.print(yStopMinusDisplay, 4);
  lcd1.print("mm ");
}

void SFSDisplayRefresh(void){
  lcd1.setCursor(0, 1);
  lcd1.print("Set Feed Speeds mm/s");
  lcd1.setCursor(0, 2);
  lcd1.print("1: Y+=              ");
  lcd1.setCursor(0, 3);
  lcd1.print("2: Y-=              ");
  lcd1.setCursor(7, 2);
  lcd1.print((float)kEV_01 / 100);
  lcd1.print("mm/s ");
  lcd1.setCursor(7, 3);
  lcd1.print((float)kEV_02 / 100);
  lcd1.print("mm/s ");
}




//---------------------------------------------------------Y-Axis Soft Stops--------------------------------------------------------
/*                                                            lcd1 DSIPLAY
                                                     #### 1 Axis CNC Soft Stop ####
                                                     #### Dir: Y+   * to Start ####
                                                     #### Speed:               ####
                                                     #### Pos:                 #### 
*/

void RunCNCySoftStop(void){
  yPreBacklash = true;
  jogMode = false;
  unsigned long feedDelay = 0;
  float backLashDiff = 0.0f;
  long yCNC_m0 = 0;
  byte slowDown = 1;

  odrive_serial << "w axis" << 0 << ".controller.config.vel_limit " << ySpeedLimitLow << '\n';
  lcd1.setCursor(0, 0);
  lcd1.print("1 Axis CNC Soft Stop");
  lcd1.setCursor(0, 1);
  lcd1.print("Dir:      * to Start");
  lcd1.setCursor(0, 2);
  lcd1.print("Speed:              ");
  lcd1.setCursor(0, 3);
  lcd1.print("Stop Pos:           ");
  displaySpeed = kEV_01;
  RunCNCDisplayRefresh();
  lcd1.setCursor(4, 1);
  lcd1.print("Y+");
  lcd1.setCursor(10, 3);
  lcd1.print(yStopPlus/16384, 4);
  lcd1.print(" mm");
  while(KeyPadRetrieve() != 10){ // 2 functions, wait for * to be pressed and refresh LCD 2's Axes positions
    if(lcdRefreshTime + 100 < millis()){
      EncoderPosition();
      LCD2_Update();
      lcdRefreshTime = millis();
    }
    switch (KeyPadRetrieve()){
      case 11:
        lcd1.setCursor(6, 1);
        lcd1.print("  User Stopped");
        delay(2000);
        return;
      break;
      default:
      break;
    }
  }
  for(int count = 0; count < 60; ++count){
    lcd1.setCursor(10, 1);
    lcd1.print(" # to STOP");
    displaySpeed = kEV_01;
    RunCNCDisplayRefresh();
    lcd1.setCursor(4, 1);
    lcd1.print("Y+");
    lcd1.setCursor(10, 3);
    lcd1.print(yStopPlus/16384, 4);
    lcd1.print(" mm");
    yTab_m0_New = yStopPlus;
    yCNC_m0 = yPos_m0 + 16384 * y_mmBacklash;
    //YBacklashComp(yTab_m0_New);
    if (kEV_01 < 200 && kEV_01 != 0){   // Positive direction Fine control of speed if below 2mm/s (slower than the ODrive min speed)
      while(yPos_m0 - yEnc_m0 < -16.384 || yPos_m0 - yEnc_m0 > 16.384){
        EncoderPosition();
        if(kEV_01 > yPos_m0 - yEnc_m0){
          yCNC_m0 = yPos_m0;
        }else{
          yCNC_m0 = yCNC_m0 + kEV_01 * 1.38; // Change Multiplier to adjust mm/s speed
        }
        odrive.SetPosition(0, yCNC_m0);
        if(lcdRefreshTime + 100 < millis()){
          LCD2_Update();
          lcdRefreshTime = millis();
          switch (KeyPadRetrieve()){   // Exit monitor
            case 11:
              odrive_serial << "r axis" << 0 << ".encoder.pos_estimate " << '\n';
              yPos_m0 = odrive.readFloat();
              odrive.SetPosition(0, yPos_m0);
              lcd1.setCursor(6, 1);
              lcd1.print("  User Stopped");
              yPreBacklash = false;
              delay(2000);
            return;
            break;
            default:
            break;
          }
        }
      }
    }// else and next statement, try
    
    odrive.SetPosition(0, yPos_m0);   // Positive direction ODrive direct speed control above 1.99mm/s
    displaySpeed = kEV_01;
    while(yPos_m0 - yEnc_m0 < -16.384 || yPos_m0 - yEnc_m0 > 16.384){    // Wait for the motor to almost finish moving (within 0.001mm)
      if(lcdRefreshTime + 100 < millis()){
        EncoderPosition();
        LCD2_Update();
        lcdRefreshTime = millis();
      }
      switch (KeyPadRetrieve()){
        case 11:
          odrive_serial << "r axis" << 0 << ".encoder.pos_estimate " << '\n';
          yPos_m0 = odrive.readFloat();
          odrive.SetPosition(0, yPos_m0);
          lcd1.setCursor(6, 1);
          lcd1.print("  User Stopped");
          yPreBacklash = false;
          delay(2000);
          return;
        break;
        default:
        break;
      }
      EncoderPosition();
    }
    EncoderPosition();
    displaySpeed = kEV_02;
    RunCNCDisplayRefresh();
    lcd1.setCursor(6, 1);  // Wait for user to press * to continues (allows for droppoing Z-Axis by hand
    lcd1.print(" * to Continue");
    lcd1.setCursor(4, 1);
    lcd1.print("Y-");
    lcd1.setCursor(10, 3);
    lcd1.print(yStopMinus/16384, 4);
    lcd1.print(" mm");
    while(KeyPadRetrieve() != 10){
      if(lcdRefreshTime + 100 < millis()){
        EncoderPosition();
        LCD2_Update();
        lcdRefreshTime = millis();
      }
      switch (KeyPadRetrieve()){
        case 11:
          lcd1.setCursor(6, 1);
          lcd1.print("  User Stopped");
          yPreBacklash = false;
          delay(2000);
          return;
        break;
        default:
        break;
      }
    }
  
    lcd1.setCursor(6, 1);
    lcd1.print("HOLD # to STOP");
    yTab_m0_New = yStopMinus;
    yCNC_m0 = yPos_m0 - 16384 * y_mmBacklash;
//    YBacklashComp(yTab_m0_New);
    if (kEV_02 < 200 && kEV_02 != 0){   // Negative direction Fine control of speed if below 2mm/s (slower than the ODrive min speed)
      while(yPos_m0 - yEnc_m0 < -16.384 || yPos_m0 - yEnc_m0 > 16.384){
        EncoderPosition();
        if(kEV_02 > yEnc_m0 - yPos_m0){
          yCNC_m0 = yPos_m0;
        }else{
          yCNC_m0 = yCNC_m0 - kEV_02 * 1.38; // Change Multiplier to adjust mm/s speed
        }
        odrive.SetPosition(0, yCNC_m0);
        if(lcdRefreshTime + 100 < millis()){
          LCD2_Update();
          lcdRefreshTime = millis();
          switch (KeyPadRetrieve()){   // Exit monitor
            case 11:
              odrive_serial << "r axis" << 0 << ".encoder.pos_estimate " << '\n';
              yPos_m0 = odrive.readFloat();
              odrive.SetPosition(0, yPos_m0);
              lcd1.setCursor(6, 1);
              lcd1.print("  User Stopped");
              yPreBacklash = false;
              delay(2000);
              return;
            break;
            default:
            break;
          }
        }
      }
    }
    
    odrive.SetPosition(0, yPos_m0);  // Negative direction ODrive direct speed control above 1.99mm/s
    displaySpeed = kEV_02;
    while(yPos_m0 - yEnc_m0 < -16.384 || yPos_m0 - yEnc_m0 > 16.384){    // Wait for the motor to almost finish moving (within 0.001mm)
      if(lcdRefreshTime + 100 < millis()){
        EncoderPosition();
        LCD2_Update();
        lcdRefreshTime = millis();
      }
      switch (KeyPadRetrieve()){
        case 11:
          odrive_serial << "r axis" << 0 << ".encoder.pos_estimate " << '\n';
          yPos_m0 = odrive.readFloat();
          odrive.SetPosition(0, yPos_m0);
          lcd1.setCursor(6, 1);
          lcd1.print("  User Stopped");
          yPreBacklash = false;
          delay(2000);
          return;
        break;
        default:
        break;
      }
      EncoderPosition();
    }
    EncoderPosition();
    RunCNCDisplayRefresh();

    lcd1.setCursor(6, 1);  // Wait for user to press * to continues (allows for droppoing Z-Axis by hand
    lcd1.print(" * to Continue");
    while(KeyPadRetrieve() != 10){
      if(lcdRefreshTime + 100 < millis()){
        EncoderPosition();
        LCD2_Update();
        lcdRefreshTime = millis();
      }
      switch (KeyPadRetrieve()){
        case 11:
          lcd1.setCursor(6, 1);
          lcd1.print("  User Stopped");
          yPreBacklash = false;
          delay(2000);
          return;
        break;
        default:
        break;
      }
    }
  }
  yPreBacklash = false;
}

//__________________________________________________________Encoder Position________________________________________________________

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

//_________________________________________________________CNC Display Refresh______________________________________________________

void RunCNCDisplayRefresh(void){
  lcd1.setCursor(0, 2);
  lcd1.print("Speed:              ");
  lcd1.setCursor(0, 3);
  lcd1.print("Stop Pos:           ");
  lcd1.setCursor(7, 2);
  lcd1.print(displaySpeed / 100, 2);
  lcd1.print("mm/s");
}

// ------------------------------------------------------Keypad Enter Value-----------------------------------------------

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

//-------------------------------------JoyPad Control------------------------------------

void JoypadControl (void){

  while(KeyPadRetrieve() != 11){
    lcd1.setCursor(0, 1);
    lcd1.print("Ensure Joypad is OFF");
    lcd1.setCursor(0, 2);
    lcd1.print(" before continuing  ");
    lcd1.setCursor(0, 3);
    lcd1.print("#: to continue [   ]");
    topSwtchVal = digitalRead(topSwtchPin);
    if (topSwtchVal > 0) {
      lcd1.setCursor(16, 3);
      lcd1.print("ON");
    }else{
      lcd1.setCursor(16, 3);
      lcd1.print("OFF");
    }
  }
  while(KeyPadRetrieve() < 12){
  }
    long joyPadSpeed = 0;
    long spdMultiply = 0; //Axis speed multiplier controlled by joystick top potentiometer
    long testY_Pos = 0;
    long xJoyPos_m1 = 0;
    long yJoyPos_m0 = 0;
    
    digitalWrite(redLedPin, HIGH); //Turn the Red LED on that is connected to digital pin 50 (called redLedPin)
    delay(100);
    digitalWrite(redLedPin, LOW); //Turn the Red LED on that is connected to digital pin 50 (called redLedPin)
    delay(100);
    digitalWrite(yellowLedPin, HIGH);
    delay(100);
    digitalWrite(yellowLedPin, LOW);
    delay(100);
    digitalWrite(redLedPin, HIGH);
    delay(100);
    digitalWrite(yellowLedPin, HIGH);
    delay(100);
      
    if (oDrive0m0CLC == false && oDrive0m1CLC == false){
      lcd1.setCursor(0, 0);
      lcd1.print("     NO CONTROL     ");
      lcd1.setCursor(0, 1);
      lcd1.print("   Calibrate Axes   ");
      lcd1.setCursor(0, 2);
      lcd1.print("                    ");
      lcd1.setCursor(0, 3);
      lcd1.print("                    ");
    }

    while(topSwtchVal == 1){ //millis() - start < duration)
      topPotVal = analogRead(topPotPin); //Put the current value of A0 (analog input 0) into memory location called "topPotVal"
      xAxisVal = analogRead(xAxisPin); //Put the current value of A1 (analog input 1) into memory location called "xAxisVal"
      yAxisVal = analogRead(yAxisPin); //Put the current value of A2 (analog input 2) into memory location called "yAxisVal"
      zAxisVal = analogRead(zAxisPin); //Put the current value of A3 (analog input 3) into memory location called "zAxisVal"
      topSwtchVal = digitalRead(topSwtchPin); //Put the current value of digital pin 46 into memory location called "topSwtchVal"

      if (yAxisVal > 400){ // Y-Axis Negative direction (towards rear of machine)
        joyPadSpeed = (long) topPotVal + (long) 20;
        joyPadSpeed = joyPadSpeed * 80 + 32768;
        yJoyPos_m0 = yTab_m0 + (long) topPotVal + (long) yAxisVal -390;
      }
      if (yAxisVal < 300){ // Y-Axis Positive direction (into rear of machine)
        joyPadSpeed = (long) topPotVal + (long) 20;
        joyPadSpeed = joyPadSpeed * 80 + 32768;
        yJoyPos_m0 = yTab_m0 - (long) topPotVal + (long) yAxisVal -350;
      }
      if (xAxisVal > 440){ // X-Axis
        joyPadSpeed = (long) topPotVal + (long) 20;
        joyPadSpeed = joyPadSpeed * 80 + 32768;
        xJoyPos_m1 = xTab_m1 + (long) topPotVal + (long) xAxisVal -390;
      }
      if (xAxisVal < 360){ // X-Axis
        joyPadSpeed = (long) topPotVal + (long) 20;
        joyPadSpeed = joyPadSpeed * 80 + 32768;
        xJoyPos_m1 = xTab_m1 - (long) topPotVal + (long) xAxisVal -350;
      }
      GlobalMove(joyPadSpeed,xJoyPos_m1,joyPadSpeed,yJoyPos_m0,0,0,1);
      Serial.println(xAxisVal);
    }
    digitalWrite(yellowLedPin, LOW);
    digitalWrite(redLedPin, LOW);
}

//____________________________________________________________Set Feed Speed________________________________________________________

void SetFeedSpeed(void){
  bool setFeedSpeed = true;
  while(setFeedSpeed == true){
    switch (KeyPadRetrieve()){
      case 1:
        while(KeyPadRetrieve() == 1){
          delay(100);
        }
        lcd1.setCursor(7, 2);
        lcd1.print("             ");
        //lcd1.setCursor(7, 2);
        KeypadEnterValue(false, 7, 2);
        if (keypadEnteredValue * 16384 > ySpeedLimitHigh){
          keypadEnteredValue = ySpeedLimitHigh / 16384;
          kEV_01 = 2200;
        }
        if (keypadEnteredValue * 16384 < ySpeedLimitLow){
          yPlusFeedSpeed = 2.0f * 16384;
          kEV_01 = keypadEnteredValue * 100;
        }else{
          yPlusFeedSpeed = keypadEnteredValue * 16384;
          kEV_01 = keypadEnteredValue * 100;
        }
        SFSDisplayRefresh();
      break;
             
      case 2:
        while(KeyPadRetrieve() == 2){
          delay(100);
        }
        lcd1.setCursor(7, 3);
        lcd1.print("             ");
        //lcd1.setCursor(7, 3);
        KeypadEnterValue(false, 7, 3);
        if (keypadEnteredValue * 16384 > ySpeedLimitHigh){
          keypadEnteredValue = ySpeedLimitHigh / 16384;
          kEV_02 = 2200;
        }
        if (keypadEnteredValue * 16384 < ySpeedLimitLow){
          yMinusFeedSpeed = 2.0f * 16384;
          kEV_02 = keypadEnteredValue * 100;
        }else{
          yMinusFeedSpeed = keypadEnteredValue * 16384;
          kEV_02 = keypadEnteredValue * 100;
        }
        SFSDisplayRefresh(); 
      break;
            
      case 11:
      setFeedSpeed = false;
      while(KeyPadRetrieve() == 11){
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
  }
}

// Call this function to refresh the full Right LCD (lcd2) display, update the values and
//   to provide a delay - e.g. LCD2_UpdateDelay(100); 

void LCD2_UpdateDelay(int dispDelay){
int count = 100;

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



//_____________________________________________________________________LINE MODE__________________________________________________________

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

  xLMDestination = (long)xLMDestination_mm * 16384;
  yLMDestination = (long)yLMDestination_mm * 16384;
  
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
        Serial.println("L");
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


//_____________________________________________________________________Global Move of all Axes__________________________________________________________


void GlobalMove (long xSpeed, long xDestination, long ySpeed, long yDestination, long zSpeed, long zDestination, bool firstLoop){

// Pre-Move Backlash control & Display Offset
  
  xTab_m1_New = xDestination - xDisplayOffset;
  yTab_m0_New = yDestination - yDisplayOffset;
  /*Serial.print("  xDestination ");
  Serial.print(xDestination / 16384);
  Serial.print("  xDisplay ");
  Serial.print(xDisplay / 16384);
  Serial.print("  xDisplayOffset ");
  Serial.print(xDisplayOffset / 16384);
  Serial.print("  xTab_m1_New ");
  Serial.print(xTab_m1_New / 16384);
  Serial.print("  xPos_m1 ");
  Serial.print(xPos_m1 / 16384);
  Serial.print("  xEnc_m1 ");
  Serial.print(xEnc_m1 / 16384);
  Serial.print("  xTab_m1 ");
  Serial.print(xTab_m1 / 16384);
  */
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
        switch (KeyPadRetrieve()){   // Exit monitor
          case 11:
            odrive_serial << "r axis" << 1 << ".encoder.pos_estimate " << '\n';
            xPos_m1 = odrive.readFloat();
            odrive.SetPosition(1, xPos_m1);
            lcd1.setCursor(6, 1);
            lcd1.print("  User Stopped");
            xPreBacklash = false;
            delay(2000);
          return;
          break;
          default:
          break;
        }
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
        switch (KeyPadRetrieve()){   // Exit monitor
          case 11:
            odrive_serial << "r axis" << 0 << ".encoder.pos_estimate " << '\n';
            yPos_m0 = odrive.readFloat();
            odrive.SetPosition(0, yPos_m0);
            lcd1.setCursor(6, 1);
            lcd1.print("  User Stopped");
            yPreBacklash = false;
            delay(2000);
          return;
          break;
          default:
          break;
        }
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
        switch (KeyPadRetrieve()){   // Exit monitor
          case 11:
            odrive_serial << "r axis" << 1 << ".encoder.pos_estimate " << '\n';
            xPos_m1 = odrive.readFloat();
            odrive.SetPosition(1, xPos_m1);
            lcd1.setCursor(6, 1);
            lcd1.print("  User Stopped");
            xPreBacklash = false;
            delay(2000);
          return;
          break;
          default:
          break;
        }
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
        switch (KeyPadRetrieve()){   // Exit monitor
          case 11:
            odrive_serial << "r axis" << 0 << ".encoder.pos_estimate " << '\n';
            yPos_m0 = odrive.readFloat();
            odrive.SetPosition(0, yPos_m0);
            lcd1.setCursor(6, 1);
            lcd1.print("  User Stopped");
            yPreBacklash = false;
            delay(2000);
          return;
          break;
          default:
          break;
        }
      }
    } 
  }



// X-Axis and Y-Axis moving and both are greater than or equal to 2mm/s, can be different speeds
 
  if (xSpeed > 32767 && ySpeed > 32767 && zSpeed == 0){
    //Serial.println("X and Y Global Move");
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
        switch (KeyPadRetrieve()){   // Exit monitor
          case 11:
            odrive_serial << "r axis" << 1 << ".encoder.pos_estimate " << '\n';
            odrive_serial << "r axis" << 0 << ".encoder.pos_estimate " << '\n';
            xPos_m1 = odrive.readFloat();
            odrive.SetPosition(1, xPos_m1);
            yPos_m0 = odrive.readFloat();
            odrive.SetPosition(0, yPos_m0);
            lcd1.setCursor(6, 1);
            lcd1.print("  User Stopped");
            xPreBacklash = false;
            yPreBacklash = false;
            delay(2000);
          return;
          break;
          default:
          break;
        }
      }
    } 
  }
  /*Serial.print("  xBacklashIn ");
  Serial.print(xBacklashIn);
  Serial.print("  xEnc_m1 ");
  Serial.print(xEnc_m1 / 16384);
  Serial.print("  xTab_m1 ");
  Serial.print(xTab_m1 / 16384);
  Serial.print("  xDisplayOffset ");
  Serial.print(xDisplayOffset / 16384);
  Serial.print("  xDisplay ");
  Serial.println(xDisplay / 16384);
  */
  menuSelect = 1;
  keySelect = 12;
}

//_____________________________________________________________________EMERGENCY STOP (INTERRUPT SERVICE ROUTINE)______________________________________________________

void ISR_EStop(void) {
  int requested_state;
  oDrive0m0CLC = false;
  oDrive0m1CLC = false;
  requested_state = ODriveArduino::AXIS_STATE_IDLE;
  odrive.run_state(0, requested_state, false);
  requested_state = ODriveArduino::AXIS_STATE_IDLE;
  odrive.run_state(1, requested_state, false);
  //requested_state = ODriveArduino::AXIS_STATE_IDLE;
  //odrive.run_state(2, requested_state, false);
  //requested_state = ODriveArduino::AXIS_STATE_IDLE;
  //odrive.run_state(3, requested_state, false);

  lcd1.setCursor(0, 0);
  lcd1.print("                    ");
  lcd1.setCursor(0, 1);
  lcd1.print("     EMERGENCY      ");
  lcd1.setCursor(0, 2);
  lcd1.print("                    ");
  lcd1.setCursor(0, 3);
  lcd1.print("                    ");
  
  lcd2.setCursor(0, 0);
  lcd2.print("                    ");
  lcd2.setCursor(0, 1);
  lcd2.print("        STOP        ");
  lcd2.setCursor(0, 2);
  lcd2.print("                    ");
  lcd2.setCursor(0, 3);
  lcd2.print("                    ");

  for (int lcd2BkltDim = 0; lcd2BkltDim >0; lcd2BkltDim--){  
    analogWrite(lcd2Backlight, lcd2BkltDim);
    delayMicroseconds(1000);
  }
  for (int lcd2BkltDim = 0; lcd2BkltDim <255; lcd2BkltDim++){  
    analogWrite(lcd2Backlight, lcd2BkltDim);
    delayMicroseconds(1000);
  }
  for (int lcd1BkltDim = 0; lcd1BkltDim >0; lcd1BkltDim--){  
    analogWrite(lcd1Backlight, lcd1BkltDim);
    delayMicroseconds(1000);
  }
  for (int lcd1BkltDim = 0; lcd1BkltDim <255; lcd1BkltDim++){  
    analogWrite(lcd1Backlight, lcd1BkltDim);
    delayMicroseconds(1000);
  }
  analogWrite(lcd2Backlight, 255);
  analogWrite(lcd1Backlight, 255);
  eStopPressed = true;
}


//_____________________________________________________________________Test Patterns______________________________________________________

void TestPatternCircles(void){
  long testSpeed = 300000;

  RunCircleMillmm (testSpeed,0,0,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,10,0,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,20,0,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,30,0,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,40,0,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,50,0,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,-10,0,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,-20,0,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,-30,0,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,-40,0,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,-50,0,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  testSpeed = 300000;
  RunCircleMillmm (testSpeed,0,10,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,10,10,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,20,10,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,30,10,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,40,10,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,50,10,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,-10,10,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,-20,10,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,-30,10,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,-40,10,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,-50,10,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  testSpeed = 300000;
  RunCircleMillmm (testSpeed,0,20,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,10,20,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,20,20,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,30,20,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,40,20,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,50,20,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,-10,20,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,-20,20,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,-30,20,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,-40,20,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,-50,20,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  testSpeed = 300000;
  RunCircleMillmm (testSpeed,0,30,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,10,30,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,20,30,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,30,30,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,40,30,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,50,30,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,-10,30,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,-20,30,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,-30,30,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,-40,30,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,-50,30,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  testSpeed = 300000;
  RunCircleMillmm (testSpeed,0,40,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,10,40,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,20,40,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,30,40,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,40,40,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,50,40,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,-10,40,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,-20,40,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,-30,40,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,-40,40,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,-50,40,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  testSpeed = 300000;
  RunCircleMillmm (testSpeed,0,50,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,10,50,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,20,50,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,30,50,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,40,50,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,50,50,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,-10,50,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,-20,50,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,-30,50,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,-40,50,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,-50,50,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  testSpeed = 300000;
  RunCircleMillmm (testSpeed,0,-10,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,10,-10,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,20,-10,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,30,-10,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,40,-10,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,50,-10,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,-10,-10,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,-20,-10,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,-30,-10,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,-40,-10,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,-50,-10,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  testSpeed = 300000;
  RunCircleMillmm (testSpeed,0,-20,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,10,-20,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,20,-20,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,30,-20,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,40,-20,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,50,-20,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,-10,-20,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,-20,-20,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,-30,-20,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,-40,-20,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,-50,-20,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  testSpeed = 300000;
  RunCircleMillmm (testSpeed,0,-30,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,10,-30,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,20,-30,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,30,-30,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,40,-30,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,50,-30,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,-10,-30,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,-20,-30,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,-30,-30,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,-40,-30,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,-50,-30,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  testSpeed = 300000;
  RunCircleMillmm (testSpeed,0,-40,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,10,-40,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,20,-40,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,30,-40,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,40,-40,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,50,-40,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,-10,-40,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,-20,-40,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,-30,-40,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,-40,-40,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,-50,-40,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  testSpeed = 300000;
  RunCircleMillmm (testSpeed,0,-50,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,10,-50,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,20,-50,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,30,-50,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,40,-50,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,50,-50,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,-10,-50,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,-20,-50,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,-30,-50,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,-40,-50,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
  RunCircleMillmm (testSpeed,-50,-50,5,0,false,1); // long cirSpeed, float xCentermm, float yCentermm, float diameter, float toolDiam, bool internal, float cycles
}




//-----------------------------------------------------------------Eye - Circles and Lines------------------------------------------------

void TestPatternEye(long testSpeed){
  
      LineMill(testSpeed,0,testSpeed,0,0,0);
      //delay (10000);      
      LineMill(testSpeed,50,testSpeed,0,0,0);
      LineMill(testSpeed,50,testSpeed,1,0,0);
      LineMill(testSpeed,1,testSpeed,0,0,0);
      LineMill(testSpeed,2,testSpeed,0,0,0);
      LineMill(testSpeed,50,testSpeed,2,0,0);
      LineMill(testSpeed,50,testSpeed,3,0,0);
      LineMill(testSpeed,3,testSpeed,0,0,0);
      LineMill(testSpeed,4,testSpeed,0,0,0);
      LineMill(testSpeed,50,testSpeed,4,0,0);
      LineMill(testSpeed,50,testSpeed,5,0,0);
      LineMill(testSpeed,5,testSpeed,0,0,0);
      LineMill(testSpeed,6,testSpeed,0,0,0);
      LineMill(testSpeed,50,testSpeed,6,0,0);
      LineMill(testSpeed,50,testSpeed,7,0,0);
      LineMill(testSpeed,7,testSpeed,0,0,0);
      LineMill(testSpeed,8,testSpeed,0,0,0);
      LineMill(testSpeed,50,testSpeed,8,0,0);
      LineMill(testSpeed,50,testSpeed,9,0,0);
      LineMill(testSpeed,9,testSpeed,0,0,0);
      LineMill(testSpeed,10,testSpeed,0,0,0);
      LineMill(testSpeed,50,testSpeed,10,0,0);
      LineMill(testSpeed,50,testSpeed,11,0,0);
      LineMill(testSpeed,11,testSpeed,0,0,0);
      LineMill(testSpeed,12,testSpeed,0,0,0);
      LineMill(testSpeed,50,testSpeed,12,0,0);
      LineMill(testSpeed,50,testSpeed,13,0,0);
      LineMill(testSpeed,13,testSpeed,0,0,0);
      LineMill(testSpeed,14,testSpeed,0,0,0);
      LineMill(testSpeed,50,testSpeed,14,0,0);
      LineMill(testSpeed,50,testSpeed,15,0,0);
      LineMill(testSpeed,15,testSpeed,0,0,0);
      LineMill(testSpeed,16,testSpeed,0,0,0);
      LineMill(testSpeed,50,testSpeed,16,0,0);
      LineMill(testSpeed,50,testSpeed,17,0,0);
      LineMill(testSpeed,17,testSpeed,0,0,0);
      LineMill(testSpeed,18,testSpeed,0,0,0);
      LineMill(testSpeed,50,testSpeed,18,0,0);
      LineMill(testSpeed,50,testSpeed,19,0,0);
      LineMill(testSpeed,19,testSpeed,0,0,0);
      LineMill(testSpeed,20,testSpeed,0,0,0);
      LineMill(testSpeed,50,testSpeed,20,0,0);
      LineMill(testSpeed,50,testSpeed,21,0,0);
      LineMill(testSpeed,21,testSpeed,0,0,0);
      LineMill(testSpeed,22,testSpeed,0,0,0);
      LineMill(testSpeed,50,testSpeed,22,0,0);
      LineMill(testSpeed,50,testSpeed,23,0,0);
      LineMill(testSpeed,23,testSpeed,0,0,0);
      LineMill(testSpeed,24,testSpeed,0,0,0);
      LineMill(testSpeed,50,testSpeed,24,0,0);
      LineMill(testSpeed,50,testSpeed,25,0,0);
      LineMill(testSpeed,25,testSpeed,0,0,0);
      LineMill(testSpeed,26,testSpeed,0,0,0);
      LineMill(testSpeed,50,testSpeed,26,0,0);
      LineMill(testSpeed,50,testSpeed,27,0,0);
      LineMill(testSpeed,27,testSpeed,0,0,0);
      LineMill(testSpeed,28,testSpeed,0,0,0);
      LineMill(testSpeed,50,testSpeed,28,0,0);
      LineMill(testSpeed,50,testSpeed,29,0,0);
      LineMill(testSpeed,29,testSpeed,0,0,0);
      LineMill(testSpeed,30,testSpeed,0,0,0);
      LineMill(testSpeed,50,testSpeed,30,0,0);
      LineMill(testSpeed,50,testSpeed,31,0,0);
      LineMill(testSpeed,31,testSpeed,0,0,0);
      LineMill(testSpeed,32,testSpeed,0,0,0);
      LineMill(testSpeed,50,testSpeed,32,0,0);
      LineMill(testSpeed,50,testSpeed,33,0,0);
      LineMill(testSpeed,33,testSpeed,0,0,0);
      LineMill(testSpeed,34,testSpeed,0,0,0);
      LineMill(testSpeed,50,testSpeed,34,0,0);
      LineMill(testSpeed,50,testSpeed,35,0,0);
      LineMill(testSpeed,35,testSpeed,0,0,0);
      LineMill(testSpeed,36,testSpeed,0,0,0);
      LineMill(testSpeed,50,testSpeed,36,0,0);
      LineMill(testSpeed,50,testSpeed,37,0,0);
      LineMill(testSpeed,37,testSpeed,0,0,0);
      LineMill(testSpeed,38,testSpeed,0,0,0);
      LineMill(testSpeed,50,testSpeed,38,0,0);
      LineMill(testSpeed,50,testSpeed,39,0,0);
      LineMill(testSpeed,39,testSpeed,0,0,0);
      LineMill(testSpeed,40,testSpeed,0,0,0);
      LineMill(testSpeed,50,testSpeed,40,0,0);
      LineMill(testSpeed,50,testSpeed,41,0,0);
      LineMill(testSpeed,41,testSpeed,0,0,0);
      LineMill(testSpeed,42,testSpeed,0,0,0);
      LineMill(testSpeed,50,testSpeed,42,0,0);
      LineMill(testSpeed,50,testSpeed,43,0,0);
      LineMill(testSpeed,43,testSpeed,0,0,0);
      LineMill(testSpeed,44,testSpeed,0,0,0);
      LineMill(testSpeed,50,testSpeed,44,0,0);
      LineMill(testSpeed,50,testSpeed,45,0,0);
      LineMill(testSpeed,45,testSpeed,0,0,0);
      LineMill(testSpeed,46,testSpeed,0,0,0);
      LineMill(testSpeed,50,testSpeed,46,0,0);
      LineMill(testSpeed,50,testSpeed,47,0,0);
      LineMill(testSpeed,47,testSpeed,0,0,0);
      LineMill(testSpeed,48,testSpeed,0,0,0);
      LineMill(testSpeed,50,testSpeed,48,0,0);
      LineMill(testSpeed,50,testSpeed,49,0,0);
      LineMill(testSpeed,49,testSpeed,0,0,0);
      LineMill(testSpeed,50,testSpeed,0,0,0);
      LineMill(testSpeed,50,testSpeed,50,0,0);

      //delay (20000);

      RunCircleMillmm (testSpeed,25,25,1,0,false,1);
      RunCircleMillmm (testSpeed,25,25,2,0,false,1);
      RunCircleMillmm (testSpeed,25,25,3,0,false,1);
      RunCircleMillmm (testSpeed,25,25,4,0,false,1);
      RunCircleMillmm (testSpeed,25,25,5,0,false,1);
      RunCircleMillmm (testSpeed,25,25,6,0,false,1);
      RunCircleMillmm (testSpeed,25,25,7,0,false,1);
      RunCircleMillmm (testSpeed,25,25,8,0,false,1);
      RunCircleMillmm (testSpeed,25,25,10,0,false,0.5);
      RunCircleMillmm (testSpeed,25,25,12,0,false,0.5);
      RunCircleMillmm (testSpeed,25,25,14,0,false,0.5);
      RunCircleMillmm (testSpeed,25,25,16,0,false,0.5);
      RunCircleMillmm (testSpeed,25,25,18,0,false,0.5);
      RunCircleMillmm (testSpeed,25,25,20,0,false,0.5);
      RunCircleMillmm (testSpeed,25,25,22,0,false,0.55);
      RunCircleMillmm (testSpeed,25,25,24,0,false,0.6);
      RunCircleMillmm (testSpeed,25,25,26,0,false,0.65);
      RunCircleMillmm (testSpeed,25,25,28,0,false,0.7);
      RunCircleMillmm (testSpeed,25,25,30,0,false,0.75);
      RunCircleMillmm (testSpeed,25,25,32,0,false,1);
      RunCircleMillmm (testSpeed,25,25,34,0,false,1);

      //delay(20000);

      LineMill(testSpeed,0,testSpeed,0,0,0);

      //delay (10000);
      
      LineMill(testSpeed,0,testSpeed,50,0,0);
      LineMill(testSpeed,1,testSpeed,50,0,0);
      LineMill(testSpeed,0,testSpeed,1,0,0);
      LineMill(testSpeed,0,testSpeed,2,0,0);
      LineMill(testSpeed,2,testSpeed,50,0,0);
      LineMill(testSpeed,3,testSpeed,50,0,0);
      LineMill(testSpeed,0,testSpeed,3,0,0);
      LineMill(testSpeed,0,testSpeed,4,0,0);
      LineMill(testSpeed,4,testSpeed,50,0,0);
      LineMill(testSpeed,5,testSpeed,50,0,0);
      LineMill(testSpeed,0,testSpeed,5,0,0);
      LineMill(testSpeed,0,testSpeed,6,0,0);
      LineMill(testSpeed,6,testSpeed,50,0,0);
      LineMill(testSpeed,7,testSpeed,50,0,0);
      LineMill(testSpeed,0,testSpeed,7,0,0);
      LineMill(testSpeed,0,testSpeed,8,0,0);
      LineMill(testSpeed,8,testSpeed,50,0,0);
      LineMill(testSpeed,9,testSpeed,50,0,0);
      LineMill(testSpeed,0,testSpeed,9,0,0);
      LineMill(testSpeed,0,testSpeed,10,0,0);
      LineMill(testSpeed,10,testSpeed,50,0,0);
      LineMill(testSpeed,11,testSpeed,50,0,0);
      LineMill(testSpeed,0,testSpeed,11,0,0);
      LineMill(testSpeed,0,testSpeed,12,0,0);
      LineMill(testSpeed,12,testSpeed,50,0,0);
      LineMill(testSpeed,13,testSpeed,50,0,0);
      LineMill(testSpeed,0,testSpeed,13,0,0);
      LineMill(testSpeed,0,testSpeed,14,0,0);
      LineMill(testSpeed,14,testSpeed,50,0,0);
      LineMill(testSpeed,15,testSpeed,50,0,0);
      LineMill(testSpeed,0,testSpeed,15,0,0);
      LineMill(testSpeed,0,testSpeed,16,0,0);
      LineMill(testSpeed,16,testSpeed,50,0,0);
      LineMill(testSpeed,17,testSpeed,50,0,0);
      LineMill(testSpeed,0,testSpeed,17,0,0);
      LineMill(testSpeed,0,testSpeed,18,0,0);
      LineMill(testSpeed,18,testSpeed,50,0,0);
      LineMill(testSpeed,19,testSpeed,50,0,0);
      LineMill(testSpeed,0,testSpeed,19,0,0);
      LineMill(testSpeed,0,testSpeed,20,0,0);
      LineMill(testSpeed,20,testSpeed,50,0,0);
      LineMill(testSpeed,21,testSpeed,50,0,0);
      LineMill(testSpeed,0,testSpeed,21,0,0);
      LineMill(testSpeed,0,testSpeed,22,0,0);
      LineMill(testSpeed,22,testSpeed,50,0,0);
      LineMill(testSpeed,23,testSpeed,50,0,0);
      LineMill(testSpeed,0,testSpeed,23,0,0);
      LineMill(testSpeed,0,testSpeed,24,0,0);
      LineMill(testSpeed,24,testSpeed,50,0,0);
      LineMill(testSpeed,25,testSpeed,50,0,0);
      LineMill(testSpeed,0,testSpeed,25,0,0);
      LineMill(testSpeed,0,testSpeed,26,0,0);
      LineMill(testSpeed,26,testSpeed,50,0,0);
      LineMill(testSpeed,27,testSpeed,50,0,0);
      LineMill(testSpeed,0,testSpeed,27,0,0);
      LineMill(testSpeed,0,testSpeed,28,0,0);
      LineMill(testSpeed,28,testSpeed,50,0,0);
      LineMill(testSpeed,29,testSpeed,50,0,0);
      LineMill(testSpeed,0,testSpeed,29,0,0);
      LineMill(testSpeed,0,testSpeed,30,0,0);
      LineMill(testSpeed,30,testSpeed,50,0,0);
      LineMill(testSpeed,31,testSpeed,50,0,0);
      LineMill(testSpeed,0,testSpeed,31,0,0);
      LineMill(testSpeed,0,testSpeed,32,0,0);
      LineMill(testSpeed,32,testSpeed,50,0,0);
      LineMill(testSpeed,33,testSpeed,50,0,0);
      LineMill(testSpeed,0,testSpeed,33,0,0);
      LineMill(testSpeed,0,testSpeed,34,0,0);
      LineMill(testSpeed,34,testSpeed,50,0,0);
      LineMill(testSpeed,35,testSpeed,50,0,0);
      LineMill(testSpeed,0,testSpeed,35,0,0);
      LineMill(testSpeed,0,testSpeed,36,0,0);
      LineMill(testSpeed,36,testSpeed,50,0,0);
      LineMill(testSpeed,37,testSpeed,50,0,0);
      LineMill(testSpeed,0,testSpeed,37,0,0);
      LineMill(testSpeed,0,testSpeed,38,0,0);
      LineMill(testSpeed,38,testSpeed,50,0,0);
      LineMill(testSpeed,39,testSpeed,50,0,0);
      LineMill(testSpeed,0,testSpeed,39,0,0);
      LineMill(testSpeed,0,testSpeed,40,0,0);
      LineMill(testSpeed,40,testSpeed,50,0,0);
      LineMill(testSpeed,41,testSpeed,50,0,0);
      LineMill(testSpeed,0,testSpeed,41,0,0);
      LineMill(testSpeed,0,testSpeed,42,0,0);
      LineMill(testSpeed,42,testSpeed,50,0,0);
      LineMill(testSpeed,43,testSpeed,50,0,0);
      LineMill(testSpeed,0,testSpeed,43,0,0);
      LineMill(testSpeed,0,testSpeed,44,0,0);
      LineMill(testSpeed,44,testSpeed,50,0,0);
      LineMill(testSpeed,45,testSpeed,50,0,0);
      LineMill(testSpeed,0,testSpeed,45,0,0);
      LineMill(testSpeed,0,testSpeed,46,0,0);
      LineMill(testSpeed,46,testSpeed,50,0,0);
      LineMill(testSpeed,47,testSpeed,50,0,0);
      LineMill(testSpeed,0,testSpeed,47,0,0);
      LineMill(testSpeed,0,testSpeed,48,0,0);
      LineMill(testSpeed,48,testSpeed,50,0,0);
      LineMill(testSpeed,49,testSpeed,50,0,0);
      LineMill(testSpeed,0,testSpeed,49,0,0);
      LineMill(testSpeed,0,testSpeed,50,0,0);
      LineMill(testSpeed,50,testSpeed,50,0,0);
}

void PerspexCut (long perspexSpeed){
      GlobalMove(300000,0,300000,0,0,0,false);
      delay(5000);
      LineMill(16384,-185,16384,0,0,0);
     // LineMill(40000,0,40000,10,0,0);
     // LineMill(1638,10,1638,10,0,0);
     // LineMill(16384,0,16384,0,0,0);
     // LineMill(perspexSpeed,0,perspexSpeed,0,0,0);
     // LineMill(perspexSpeed,0,perspexSpeed,0,0,0);
}
