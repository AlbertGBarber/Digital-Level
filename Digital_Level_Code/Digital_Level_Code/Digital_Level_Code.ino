//This code is placed under the MIT license
//Copyright (c) 2020 Albert Barber
//
//Permission is hereby granted, free of charge, to any person obtaining a copy
//of this software and associated documentation files (the "Software"), to deal
//in the Software without restriction, including without limitation the rights
//to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//copies of the Software, and to permit persons to whom the Software is
//furnished to do so, subject to the following conditions:
//
//The above copyright notice and this permission notice shall be included in
//all copies or substantial portions of the Software.
//
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
//THE SOFTWARE.

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <VoltageReference.h>

//=================================================================
#define DISPLAY_ADDR 0x3C //I2C address for the SSD1306 display
//=================================================================

//=================================================================
const boolean crossLaserEnable = true; //if not using a cross-line laser, set to false
//=================================================================

// ================================================================
// ===                   IMU OFFSETS                            ===
// ================================================================
//offsets for the Imu's gyro and accel, you'll need to supply your own
//by running the IMU_Zero sketch found under the MPU6050 examples
//make sure to read the instuctions in the example
#define X_GYRO_OFFSET  161
#define Y_GYRO_OFFSET  4
#define Z_GYRO_OFFSET  -14

#define X_ACCEL_OFFSET -1116
#define Y_ACCEL_OFFSET 75
#define Z_ACCEL_OFFSET 1663

// ================================================================
// ===                         MODES                            ===
// ================================================================
//Defines the order of modes, you can re-order modes as you wish, but you must keep the cross-line laser in last
//you can remove modes by setting the mode number to greater than the NUM_MODES ex: 999
//if you do this don't forget to decrease NUM_MODES to match the number of modes remaining
//and also change the order of the remaining modes (going from 0 to however remain)
#define XY_LEVEL         0  //IMU
#define ROLL_LEVEL       1  //IMU
#define PROTRACTOR       2  //IMU
#define LASER_POINTER    3  //Laser Diode
#define LASER_POINTER2   4  //Cross-line Laser Diode

uint8_t NUM_MODES =      5; //total number of active modes

volatile uint8_t mode = 0; //inital mode
// ================================================================
// ===                         PIN SETUP                        ===
// ================================================================

//Arduino Pro-Mini Pins
#define INTERRUPT_PIN_IMU 2

#define MODE_BUTTON_PIN  11
#define ZERO_BUTTON_PIN  10
#define LASER_PIN        12
#define LASER2_PIN       13

// ================================================================
// ===                     BAT SENSE SETUP                      ===
// ================================================================
//our range for lipo voltage is 4.2-3.4V, 
//after 3.4 the LiPo is 95% empty, so we should recharge
const unsigned long batteryUpdateTime = 5000; //how often we update the battery level in ms
unsigned long prevBatReadTime = 0; //the last time we read the battery in ms
uint8_t batteryLvl; //the battery percentage
#define MAX_BAT_VOLTAGE 4200 //max battery voltage
#define MIN_BAT_VOLTAGE 3400 //min battery voltage

//initialize the voltage reference library to read the Arduino's internal reference voltage
VoltageReference vRef;

// ================================================================
// ===                     SCREEN SETUP                         ===
// ================================================================
#define SCREEN_WIDTH      128
#define SCREEN_HEIGHT     64
#define YELLOW_BAR_HEIGHT 16
#define BASE_FONT_LENGTH  6
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1// Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ================================================================
// ===                  PROGRAM GLOBAL VARS                     ===
// ================================================================
boolean laserOn = false;
boolean zeroButtonToggle = false;
boolean zeroButtonRun = true;
boolean previousZeroButtonState = HIGH;
boolean previousModeButtonState = HIGH;
unsigned long currentTime = millis();
unsigned long lastZeroPress = millis();

//presets for drawing a cicle of maximum radius on the blue portion of the screen
const uint8_t circleYcenter =  SCREEN_HEIGHT - (SCREEN_HEIGHT - YELLOW_BAR_HEIGHT) / 2;
const uint8_t circleRadius = 23;

//program runtime vars
//Don't add any more, we're at max prog memory
uint8_t circleXcenter;
// ================================================================
// ===                         IMU SETUP                        ===
// ================================================================

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;

// MPU control/status vars
boolean imuOn = true;   // set true if we've attached the IMU interrupt (the imu setup connects the imu initally)
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float yprOffsets[] = {0, 0, 0};    // [yaw, pitch, roll]  offsets (in rad) for yaw/pitch/roll (for zeroing)

//interrupt routine
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

// ================================================================
// ================================================================
// ================================================================
// ===                   SETUP FUNCTION                         ===
// ================================================================
// ================================================================
// ================================================================

void setup() {
  //Serial.begin(115200);
  
  //if we're not using the cross-line laser, we can just decrease the number of modes by 1, and it will be skipped
  //(as long as it's the last mode)
  if (!crossLaserEnable) {
    NUM_MODES--;
  }
  // ================================================================
  // ===                  DISPLAY CONFIG                          ===
  // ================================================================
  //note debug messages after this are displayed on the display
  if (!display.begin(SSD1306_SWITCHCAPVCC, DISPLAY_ADDR)) { // Address 0x3C for 128x64
    //Serial.println(F("SSD1306 allocation failed"));
    for (;;); // Don't proceed, loop forever
  }

  // Clear the buffer, set the default text size, and color
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);

  // ================================================================
  // ===                  BATTERY CONFIG                          ===
  // ================================================================
  //start reading the internal voltage reference, and take an inital reading
  vRef.begin();
  batteryLvl = getBatteryLevel(vRef.readVcc(), MIN_BAT_VOLTAGE, MAX_BAT_VOLTAGE);

  // ================================================================
  // ===                  BUTTON PINS CONFIG                      ===
  // ================================================================

  pinMode(MODE_BUTTON_PIN, INPUT);
  pinMode(ZERO_BUTTON_PIN, INPUT);

  //config outputs
  pinMode(LASER_PIN, OUTPUT); //Laser
  pinMode(LASER2_PIN, OUTPUT); //cross-line laser

  // ================================================================
  // ===                      IMU CONFIG                          ===
  // ================================================================
  // initialize device
  mpu.initialize();
  pinMode(INTERRUPT_PIN_IMU, INPUT);

  // load and configure the DMP
  devStatus = mpu.dmpInitialize();

  // supply your own gyro / accel offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(X_GYRO_OFFSET);
  mpu.setYGyroOffset(Y_GYRO_OFFSET);
  mpu.setZGyroOffset(Z_GYRO_OFFSET);

  mpu.setXAccelOffset(X_ACCEL_OFFSET);
  mpu.setYAccelOffset(Y_ACCEL_OFFSET);
  mpu.setZAccelOffset(Z_ACCEL_OFFSET);

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN_IMU), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    // Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    display.clearDisplay();
    display.setCursor(16, 10);
    display.print("DMP Initialization failed (code)");
    display.print(devStatus);
    display.display();
    while (1);
  }
  //  //start up splash screen
  display.clearDisplay();
  drawHeader("By AGB");
  display.setCursor(28, 16);
  display.setTextSize(2);
  display.print("Digital");
  display.setCursor(35, 33);
  display.print("Level");
  display.display();
  delay(3000);
}

// ================================================================
// ================================================================
// ================================================================
// ===                     MAIN PROGRAM                         ===
// ================================================================
// ================================================================
// ================================================================

//Basic Function Outline
//each mode is contained in its own while loop
//until the mode is changed we continually loop through the current mode's loop
//for every mode a few things are common:
//we read the active sensor (or let interrupts for the sensor happen)
//we act on any button flags (zeroing, etc)
//we clear the display and redraw it (including the header)
//(we could overwrite only the changed parts of the display, but that is far more complicated, and for small display's it's quick to redraw the whole thing)
//we check any for any button presses at the end of the loop
//(this must come at the end of the loop, because if the mode changes we want to catch it at the while mode check, without doing anything in the loop)
//The zero button has two state boolean flags, a "run" and a "toggle"
//the "toggle" changes whenever the button is pressed
//the "run" is set false whenever the button is pressed and must be cleared by the current mode
//Both flags are reset to defaults (true for "run" and false for "toggle") when a mode changes
void loop() {

  //a typical level display using the IMU, like a bubble level
  while (mode == ROLL_LEVEL) {
    driveIMU(true); //turn on IMU
    checkDMP();
    display.clearDisplay();
    drawHeader("Roll Level");
    //We want to orient the level with the screen vertically so that the display can match the phyical roll
    //This means the IMU is rotated from its usual orientation (screen facing up)
    //so the axis of roll is different from what the library assumes,
    //and we need to caculate it ourselves
    ypr[2] = atan2(gravity.y , gravity.x );
    //an alternative calc, might be more accurate, but only works for angles < 80 deg
    //ypr[2] =  atan2(gravity.y , sqrt(gravity.x * gravity.x + gravity.z * gravity.z));

    //if the zero button is pressed, we set a new offset (zero point)
    if (!zeroButtonRun) {
      yprOffsets[2] = ypr[2];
      zeroButtonRun = true;
    }
    //subtrace the offset and convert to degrees
    float roll = (ypr[2] - yprOffsets[2]) * 180 / M_PI;
    //draw a crosshair circle, and a line across it representing the angle using the unit circle
    float lineXOffset = circleRadius * cos(-ypr[2]);
    float lineYOffset = circleRadius * sin(-ypr[2]);
    circleXcenter = circleRadius;
    drawCrosshairCircle(circleXcenter);
    display.drawLine(circleXcenter - lineXOffset, circleYcenter - lineYOffset , circleXcenter + lineXOffset, circleYcenter + lineYOffset, WHITE);
    display.setTextSize(2);
    display.setCursor(52, 30);
    //roll is negative to better match the line on the crosshair circle
    //we only display to one decimal place to avoid overflowing to the next line
    display.print(-roll, 1);
    display.display();
    readButtons();
  }

  //An x-y level using the IMU, like a circular bubble level
  //also normalizes angles when the multi-tool is turned upside down,
  //keeping them between -90, 90
  while ( mode == XY_LEVEL) {
    driveIMU(true); //turn on IMU
    checkDMP();
    display.clearDisplay();
    drawHeader("X-Y Level");
    //if the zero button is hit we'll store the current angles
    //and use them as a new zero point
    if (!zeroButtonRun) {
      yprOffsets[2] = ypr[2];
      yprOffsets[1] = ypr[1];
      zeroButtonRun = true;
    }
    //convert the angles and offsets to degrees
    float xAngleOff = yprOffsets[2] * 180 / M_PI;
    float yAngleOff = yprOffsets[1] * 180 / M_PI;
    float xAngle = ypr[2] * 180 / M_PI;
    float yAngle = ypr[1] * 180 / M_PI;

    //if we're holding the sensor upside down (ie screen facing the ground)
    //the angles will be from +-180-270
    //we shift them back to +-90 for readability
    //this also includes shifting any offsets if needed
    if (xAngle > 90 || xAngle < -90) {
      xAngle = (180 - abs(xAngle)) * getSign(xAngle);
    }

    if (yAngle > 90 || yAngle < -90) {
      yAngle = (180 - abs(yAngle)) * getSign(yAngle);
    }

    if (xAngleOff > 90 || xAngleOff < -90) {
      xAngleOff = (180 - abs(xAngleOff)) * getSign(xAngleOff);
    }

    if (yAngleOff > 90 || yAngleOff < -90) {
      yAngleOff = (180 - abs(yAngleOff)) * getSign(yAngleOff);
    }
    //subtract the offsets from the measured angle
    xAngle -= xAngleOff;
    yAngle -= yAngleOff;
    yAngle *= -1; //- to account for sensor orientation
    circleXcenter = circleRadius;
    //display the angles and a crosshair circle
    //angles are displayed with one decimal to make them fit on one line
    display.setCursor(45, 57);
    display.print("Zx2 -> Laser");
    display.setTextSize(2);
    display.setCursor(52, 20);
    display.print("X");
    display.print(xAngle, 1); //only display to one decimal place to avoid overflowing to the next line
    display.setCursor(52, 40);
    display.print("Y");
    display.print(-yAngle, 1);
    //draw a dot inside the crosshair circle according to the angles
    //(like a bubble level)
    display.fillCircle(circleXcenter + circleRadius  * sin(ypr[2]) , circleYcenter + circleRadius * sin(ypr[1]), 2, WHITE);
    drawCrosshairCircle(circleXcenter);
    display.display();
    readButtons();
  }

  //Presents yaw reading (z-axis rotation) from the IMU,
  //(z-axis is normal to the multi-tool's screen)
  //making the tool act like a compass/protractor
  //Unfortunatly the yaw drifts constantly (it's cannot be normalized by the acceletometer)
  //so you need to re-zero often
  while ( mode == PROTRACTOR ) {
    driveIMU(true); //turn on IMU
    checkDMP();
    yield();
    if (!zeroButtonRun) {
      yprOffsets[0] = ypr[0];
      zeroButtonRun = true;
    }
    ypr[0] -= yprOffsets[0];
    float yaw = ypr[0] * 180 / M_PI;
    //x and y co-ordinates for the angle line end/start points
    float lineXOffset = circleRadius * cos(ypr[0]);
    float lineYOffset = circleRadius * sin(ypr[0]);
    circleXcenter = circleRadius;
    display.clearDisplay();
    drawHeader("Protractor");
    display.setTextSize(2);
    display.setCursor(52, 30);
    display.print(yaw, 1); //print the yaw value with one decimal place to avoid text overflow
    //draws a line from -lineXOffset, --lineYOffset to +lineXOffset, +lineYOffset through the cross-hair's circle center
    display.drawLine(circleXcenter - lineXOffset, circleYcenter - lineYOffset , circleXcenter + lineXOffset, circleYcenter + lineYOffset, WHITE);
    drawCrosshairCircle(circleXcenter);
    display.display();
    readButtons();
  }

  //Laser pointer, just turns on/off the laser
  //both the extra and zero buttons will toggle the laser
  while (mode == LASER_POINTER || mode == LASER_POINTER2) {
    display.clearDisplay();
    if (mode == LASER_POINTER) {
      drawHeader("Laser Pointer");
    } else {
      drawHeader("Laser Cross");
    }
    display.setTextSize(2);
    display.setCursor(10, 30);
    //print the state of the laser
    if (laserOn) {
      display.print(F("Laser On"));
    } else {
      display.print(F("Laser Off"));
    }
    display.display();
    //if either of the extra or zero button's have been pressed
    //we need to turn the laser on or off
    //we then set the state of both buttons  b/c we don't know which was pressed
    if (!zeroButtonRun) {
      driveLaser(!laserOn);
      zeroButtonRun = true;
    }
    readButtons();
  }
}

//read the mode and zero buttons
//if the buttons have been pressed set flags
//The zero button has two state boolean flags, a "run" and a "toggle"
//the "toggle" changes whenever the button is pressed
//the "run" is set false whenever the button is pressed and must be cleared by the current mode
//Both flags are reset to defaults (true for "run" and false for "toggle") when a mode changes
void readButtons(void) {

  currentTime = millis();
  
  //if the mode pin is low (pressed) and it was not previously low (ie it's not being held down)
  //advance the mode counter
  //otherwise, the button is not pushed, set the previous state to high (not pressed)
  if (digitalRead(MODE_BUTTON_PIN) == LOW && previousModeButtonState != LOW) {
    previousModeButtonState = LOW; //set the previous state to low while the button is being held
    //if the mode pin is high, we need to increment the mode, wrapping is needed
    //and reset the system for the next mode
    resetSystemState();
    mode = (mode + 1) % NUM_MODES;
  } else if ( digitalRead(MODE_BUTTON_PIN) == HIGH) {
    previousModeButtonState = HIGH;
  }

  //if the zero pin is low (pressed) and it was not previously low (ie it's not being held down)
  //set zero toggle and run flags
  //otherwise, the button is not pushed, set the previous state to high (not pressed)
  if (digitalRead(ZERO_BUTTON_PIN) == LOW && previousZeroButtonState != LOW) {
    //double tap to drive the laser cross (only works in a level mode)
    //checks if the zero button has been pressed in the last x milliseconds
    //if it has, drive the cross-line laser
    if (currentTime - lastZeroPress < 400) {
      driveLaser(!laserOn);
    }
    lastZeroPress = millis();
    previousZeroButtonState = LOW;
    zeroButtonRun = false;
    zeroButtonToggle = !zeroButtonToggle;
  } else if ( digitalRead(ZERO_BUTTON_PIN) == HIGH) {
    previousZeroButtonState = HIGH;
  }
}

//resets button variables and turns off/resets all sensors
//sets a clean state for the next mode and tries to save some power by turning off unused sensors
void resetSystemState() {
  display.clearDisplay();
  zeroButtonToggle = false;
  zeroButtonRun = true;
  driveLaser(false);
  driveIMU(false);
}

//fills in the header on the screen (the yellow bar) with the current mode name and the battery charge level
//because the battery level can fluctuate with current draw / noise, we only measure it at fixed intervals
//this prevents it from changing too often on the display, confusing the user
void drawHeader(String modeName) {
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.print(modeName);
  currentTime = millis();
  //update the battery level after batteryUpdateTime ms
  if (currentTime - prevBatReadTime > batteryUpdateTime) {
    batteryLvl = getBatteryLevel(vRef.readVcc(), MIN_BAT_VOLTAGE, MAX_BAT_VOLTAGE);
    prevBatReadTime = millis();
  }
  display.setCursor(100, 0);
  display.print(batteryLvl);
  display.print(char(37)); //prints the % symbol
}

//draws a maximum sized cicle centered horizontally at x pixel
//with a vertical and horizontal line running through the center
void drawCrosshairCircle(uint16_t x) {
  display.drawFastVLine(x, YELLOW_BAR_HEIGHT + 1, circleRadius * 2, WHITE);
  display.drawFastHLine(x - circleRadius, circleYcenter, circleRadius * 2, WHITE);
  display.drawCircle(x, circleYcenter, circleRadius, WHITE);
}

//turn the lasers on or off according to the state input
void driveLaser(boolean state) {
  if (mode == LASER_POINTER2 || mode == XY_LEVEL || mode == ROLL_LEVEL) { //the cross-laser can be toggled on during the roll modes
    digitalWrite(LASER2_PIN, state);
  } else {
    digitalWrite(LASER_PIN, state);
  }
  laserOn = state;
}

//if the imu isn't on and state is true, turn it on (attach its interrupt)
//if the imu is on and the state is false, turn it off (detach its interrupt)
//we don't need to keep attaching and detach the interrupt if it's already been done
void driveIMU(boolean state) {
  if (!imuOn && state) {
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN_IMU), dmpDataReady, RISING);
    imuOn = true;
  } else if (imuOn && !state) {
    detachInterrupt(INTERRUPT_PIN_IMU);
    imuOn = false;
  }
}


//checks if the imu is ready to be read
//we cannot read the imu in the interrupt function b/c i2c is disabled during interrupts
void checkDMP(void) {
  if (mpuInterrupt) {
    GetDMP();
  }
}

//reads the fifo packet from the imu and clears the buffer
//if the fifo count is too big or too small we won't read, and instead clear the buffer
void GetDMP() {
  //static unsigned long LastGoodPacketTime;
  mpuInterrupt = false;
  fifoCount = mpu.getFIFOCount();
  if ((!fifoCount) || (fifoCount % packetSize)) { // we have failed Reset and wait till next time!
    // digitalWrite(LED_PIN, LOW); // lets turn off the blinking light so we can see we are failing.
    mpu.resetFIFO();// clear the buffer and start over
  } else {
    while (fifoCount  >= packetSize) { // Get the packets until we have the latest!
      mpu.getFIFOBytes(fifoBuffer, packetSize); // lets do the magic and get the data
      fifoCount -= packetSize;
    }
    //LastGoodPacketTime = millis();
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    //digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Blink the Light
  }
}

//copied from Roberto Lo Giacco Battery Sense library
//the LiPo's remaining power is not linearly related to its voltage
//so we use a best fit line to approximate the remaining power percentage
uint8_t getBatteryLevel(uint16_t voltage, uint16_t minVoltage, uint16_t maxVoltage) {
  // slow
  // uint8_t result = 110 - (110 / (1 + pow(1.468 * (voltage - minVoltage)/(maxVoltage - minVoltage), 6)));

  // steep
  // uint8_t result = 102 - (102 / (1 + pow(1.621 * (voltage - minVoltage)/(maxVoltage - minVoltage), 8.1)));

  // normal
  uint8_t result = 105 - (105 / (1 + pow(1.724 * (voltage - minVoltage) / (maxVoltage - minVoltage), 5.5)));
  return result >= 100 ? 100 : result;
}

//returns the sign of the input number (either -1 or 1)
int getSign(double num) {
  if (num >= 0) {
    return 1;
  } else {
    return -1;
  }
}
