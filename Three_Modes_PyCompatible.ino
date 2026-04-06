//////////////////////////////////////////////////////////////////////////////////////////////////////
// MIE438 FINAL PROJECT - ZEN GARDEN SAND ART TABLE                                                 
//                                                                                                  
// PURPOSE:                         
// Controls a 2-motor rail gantry capable of:
// 1) Coordinate-based motion, moves requested via (x,y) coordinate location
// 2) Manual joystick motion with continuous directional control
// 3) Clock mode, autonomously draws HH:MM in the sand and redraws every 5 minutes
//
// SYSTEM OVERVIEW:
// - Mechanical arrangement uses kinematic transform calculations so cartesian motion is translated to motor stepping
// - Two stepper motors driving a belt gantry
//
// MAIN FUNCTIONS/FEATURES:
// - Serial command interface for external Python control
// - Homing sequence using limit switches to calibrate and reset carriage location
// - Motion interruption and emergency stops with limit switches
// - Position tracking with cartesian coordinates
// - Manual joystick mode for free movement
// - Clock mode using hardware RTC with periodic 5-minute redraw
//////////////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////////////////////////
// MODE SELECT
// Three operating modes:
// MODE_JOYSTICK   uses real time manual control with an analog joystick input.
// MODE_COORDINATE receives target XY coordinates over serial and performs automated movements.
// MODE_CLOCK      autonomously reads hardware RTC and redraws the current time every 5 minutes.
//
// Default startup mode is coordinate since the art table is designed to be primarily autonomous.
//////////////////////////////////////////////////////////////////////////////////////////////////////
#include "RTC.h"

#define MODE_JOYSTICK   1
#define MODE_COORDINATE 2
#define MODE_CLOCK      3

int currentMode = MODE_COORDINATE; //default coordinate startup mode

//////////////////////////////////////////////////////////////////////////////////////////////////////
// ============================ COORDINATE MODE VARIABLE =============================================
//
// Variables defining hardware interface for stepper motors and limit switches, and motion scaling constants.
//////////////////////////////////////////////////////////////////////////////////////////////////////

#define STEP_PIN1 3
#define DIR_PIN1 4
#define STEP_PIN2 6
#define DIR_PIN2 7

// HOME_X / HOME_Y correspond to bottom left corner, machine origin side
// MAX_X / MAX_Y correspond to top right corner, farthest travel side
#define HOME_X 10
#define HOME_Y 11
#define MAX_X 12
#define MAX_Y 9   

const int stepDelay = 1000;         // stepDelay and pulseWidth for overall motor speed
const int pulseWidth = 500;         
const int PIXEL_STEP_UNIT = 25;     // number of steps corresponding to a single grid unit
const int MAX_STEP = 600;           // maximum number of motor steps from one side to another
const int GRID_MAX = 24;            // maximum number of grid units from one side to another
const int backoffStepsY = 25;       // number of steps to back off of limit switch and end interrupt
const int backoffStepsX = 30;

int currentX = 0;                   // cartesian position estimate based on grid
int currentY = 0;
int currentM1 = 0;                  // motor position estimate based on steps
int currentM2 = 0;

bool moveInterrupted = false;       // true if motion was interrupted by switch event or stop from external communication
bool positionKnown = true;          // false if switch event invalidates position estimate
bool stopRequested = false;         // external software stop request flag
bool useLimitSwitch = true;         // allows disabling limit checks for joystick mode

//////////////////////////////////////////////////////////////////////////////////////////////////////
// ============================ JOYSTICK MODE VARIABLES =============================================
//////////////////////////////////////////////////////////////////////////////////////////////////////

const int xPin = A0;
const int yPin = A1;
const int buttonPin = 2;

const int deadZone = 150;

String currentDirection = "Neutral";
String lastDirection = "Neutral";

int stepCounter = 0;

//////////////////////////////////////////////////////////////////////////////////////////////////////
// ============================ CLOCK MODE VARIABLES ================================================
//////////////////////////////////////////////////////////////////////////////////////////////////////

volatile bool clockAlarmFired = false;  // set by RTC alarm ISR every minute
int lastDrawnMinute = -1;               // tracks last drawn minute; -1 forces draw on first activation

// Clock layout: HH:MM centred across the 24x24 grid (4 digits x 4 cols + 1 colon x 2 cols = 18 cols used, 3-col margin each side)
#define CLOCK_ROW      9    // top row of all digit cells
#define DIGIT1_COL     3    // H tens
#define DIGIT2_COL     7    // H units
#define COLON_COL     11    // colon separator
#define DIGIT3_COL    13    // M tens
#define DIGIT4_COL    17    // M units

// 7-segment digit paths defined as relative movements between points (starting at the bottom left corner of each digit cell)
struct Point {
  int dx;
  int dy;
};

// Each digit is defined as a series of line segments between points
// Each point is a relative movement from the previous point
const Point DIGIT_0[] = { {0,0},{3,0}, {3,0},{3,5}, {0,5},{3,5}, {0,0},{0,5} };
const Point DIGIT_1[] = { {3,0},{3,5} };
const Point DIGIT_2[] = { {0,0},{3,0}, {3,0},{3,3}, {0,3},{3,3}, {0,3},{0,5}, {0,5},{3,5} };
const Point DIGIT_3[] = { {0,0},{3,0}, {3,0},{3,5}, {0,3},{3,3}, {0,5},{3,5} };
const Point DIGIT_4[] = { {0,0},{0,3}, {0,3},{3,3}, {3,0},{3,5} };
const Point DIGIT_5[] = { {0,0},{3,0}, {0,0},{0,3}, {0,3},{3,3}, {3,3},{3,5}, {0,5},{3,5} };
const Point DIGIT_6[] = { {0,0},{3,0}, {0,0},{0,5}, {0,3},{3,3}, {3,3},{3,5}, {0,5},{3,5} };
const Point DIGIT_7[] = { {0,0},{3,0}, {3,0},{3,5} };
const Point DIGIT_8[] = { {0,0},{3,0}, {3,0},{3,5}, {0,3},{3,3}, {0,5},{3,5}, {0,0},{0,5} };
const Point DIGIT_9[] = { {0,0},{3,0}, {3,0},{3,5}, {0,3},{3,3}, {0,0},{0,3}, {0,5},{3,5} };

// Pre-calculate lengths of each digit path for drawDigit function
const int DIGIT_0_LEN = sizeof(DIGIT_0) / sizeof(DIGIT_0[0]);
const int DIGIT_1_LEN = sizeof(DIGIT_1) / sizeof(DIGIT_1[0]);
const int DIGIT_2_LEN = sizeof(DIGIT_2) / sizeof(DIGIT_2[0]);
const int DIGIT_3_LEN = sizeof(DIGIT_3) / sizeof(DIGIT_3[0]);
const int DIGIT_4_LEN = sizeof(DIGIT_4) / sizeof(DIGIT_4[0]);
const int DIGIT_5_LEN = sizeof(DIGIT_5) / sizeof(DIGIT_5[0]);
const int DIGIT_6_LEN = sizeof(DIGIT_6) / sizeof(DIGIT_6[0]);
const int DIGIT_7_LEN = sizeof(DIGIT_7) / sizeof(DIGIT_7[0]);
const int DIGIT_8_LEN = sizeof(DIGIT_8) / sizeof(DIGIT_8[0]);
const int DIGIT_9_LEN = sizeof(DIGIT_9) / sizeof(DIGIT_9[0]);

//////////////////////////////////////////////////////////////////////////////////////////////////////
// ============================ SHARED LOW LEVEL FUNCTIONS ===========================================
// These are reusable low-level motion functions used by both coordinate and joystick modes.
//////////////////////////////////////////////////////////////////////////////////////////////////////

void pulseStep(int stepPin) {       // generates one step pulse on selected stepper motor
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(pulseWidth);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(stepDelay);
}

void backoffX(bool away) {          // moves gantry a short distance away from x-axis limit switch after trigger
  if (away) {
    digitalWrite(DIR_PIN1, HIGH);   // HOME_X pressed, move +X
    digitalWrite(DIR_PIN2, HIGH);
  }
  else {
    digitalWrite(DIR_PIN1, LOW);    // MAX_X pressed, move -X
    digitalWrite(DIR_PIN2, LOW);
  }

  // automatic backoff step for when limit switch activated
  for (int i = 0; i < backoffStepsX; i++) {
    digitalWrite(STEP_PIN1, HIGH);
    digitalWrite(STEP_PIN2,HIGH);
    delayMicroseconds(pulseWidth);
    digitalWrite(STEP_PIN1, LOW);
    digitalWrite(STEP_PIN2, LOW);
    delayMicroseconds(stepDelay);
  }
}

void backoffY(bool away) {          // moves gantry a short distance away from y-axis limit switch after trigger
  if (away) {
    digitalWrite(DIR_PIN1, LOW);    // HOME_Y pressed, move +Y
    digitalWrite(DIR_PIN2, HIGH);
  }
  else {
    digitalWrite(DIR_PIN1, HIGH);   // MAX_Y pressed, move -Y
    digitalWrite(DIR_PIN2, LOW);
  }

  for (int i = 0; i < backoffStepsY; i++) {
    digitalWrite(STEP_PIN1, HIGH);
    digitalWrite(STEP_PIN2,HIGH);
    delayMicroseconds(pulseWidth);
    digitalWrite(STEP_PIN1, LOW);
    digitalWrite(STEP_PIN2, LOW);
    delayMicroseconds(stepDelay);
  }
}

bool limitCheck() {                   // checks all limit switch readings and handles any switch event
  if (!useLimitSwitch) return false;
  
  if (digitalRead(HOME_X) == LOW) {
    Serial.println("LIMIT,HOMEX");
    moveInterrupted = true;
    backoffX(true);
    positionKnown = false;
    return true;
  }

  if (digitalRead(MAX_X) == LOW) {
    Serial.println("LIMIT,MAXX");
    moveInterrupted = true;
    backoffX(false);
    positionKnown = false;
    return true;
  }

  if (digitalRead(HOME_Y) == LOW) {
    Serial.println("LIMIT,HOMEY");
    moveInterrupted = true;
    backoffY(true);
    positionKnown = false;
    return true;
  }

  if(digitalRead(MAX_Y) == LOW) {
    Serial.println("LIMIT,MAXY");
    moveInterrupted = true;
    backoffY(false);
    positionKnown = false;
    return true;
  }
  return false;
}
 
void stepMotor1(bool dir) {           // steps motor 1 by one step if safe to do so
  if (limitCheck() || stopRequested) return;

  digitalWrite(DIR_PIN1, dir ? HIGH:LOW);
  pulseStep(STEP_PIN1);

  currentM1 += (dir ? 1:-1);

  if (limitCheck() || stopRequested) return;
}

void stepMotor2(bool dir) {           // steps motor 2 by one step if safe to do so
  if (limitCheck() || stopRequested) return;

  digitalWrite(DIR_PIN2, dir ? HIGH:LOW);
  pulseStep(STEP_PIN2);

  currentM2 += (dir ? 1:-1);

  if (limitCheck() || stopRequested) return;
}

void stepBoth(bool dir1, bool dir2) {  // steps both motors simultaneously by one step if safe to do so
  if (limitCheck() || stopRequested) return;

  digitalWrite(DIR_PIN1, dir1 ? HIGH:LOW);
  digitalWrite(DIR_PIN2, dir2 ? HIGH:LOW);

  digitalWrite(STEP_PIN1, HIGH);
  digitalWrite(STEP_PIN2, HIGH);
  delayMicroseconds(pulseWidth);
  digitalWrite(STEP_PIN1, LOW);
  digitalWrite(STEP_PIN2, LOW);  
  delayMicroseconds(stepDelay);

  currentM1 += (dir1 ? 1:-1);
  currentM2 += (dir2 ? 1:-1);

  if (limitCheck() || stopRequested) return;
}

void homeSystem() {                       // performs homing to establish a known reference location
  Serial.println("INFO,HOMING");

  while (digitalRead(HOME_X) == HIGH) {   // moves in -X direction until limit switch is hit
    if (stopRequested) return;
    
    digitalWrite(DIR_PIN1, LOW);
    digitalWrite(DIR_PIN2, LOW);

    digitalWrite(STEP_PIN1, HIGH);
    digitalWrite(STEP_PIN2, HIGH);
    delayMicroseconds(pulseWidth);
    digitalWrite(STEP_PIN1, LOW);
    digitalWrite(STEP_PIN2, LOW);  
    delayMicroseconds(stepDelay);
  }
  delay(300);
  backoffX(true);

  while(digitalRead(HOME_Y) == HIGH) {    // moves in -Y direction until limit switch is hit
    if (stopRequested) return;
    digitalWrite(DIR_PIN1, HIGH);
    digitalWrite(DIR_PIN2, LOW);

    digitalWrite(STEP_PIN1, HIGH);
    digitalWrite(STEP_PIN2, HIGH);
    delayMicroseconds(pulseWidth);
    digitalWrite(STEP_PIN1, LOW);
    digitalWrite(STEP_PIN2, LOW);  
    delayMicroseconds(stepDelay);
  }
  delay(300);
  backoffY(true);

  currentX = 0;
  currentY = 0;
  currentM1 = 0;
  currentM2 = 0;
  moveInterrupted = false;
  positionKnown = true;

  Serial.println("OK,HOME");
  Serial.println("POS,0,0");
}

bool motorMoveSafe(long M1Steps, long M2Steps) {     // predicts motor movement based on steps
  long targetM1 = currentM1 + M1Steps;
  long targetM2 = currentM2 + M2Steps;

  if (targetM1 < 0 || targetM1 > MAX_STEP) return false;
  if (targetM2 < 0 || targetM2 > MAX_STEP) return false;

  return true;
}

void moveMotors(long M1Steps, long M2Steps) {        // coordinates motion function, desired relative movement of motors
  moveInterrupted = false;
  stopRequested = false;

  long absM1 = abs(M1Steps);
  long absM2 = abs(M2Steps);

  bool dir1 = (M1Steps >= 0);
  bool dir2 = (M2Steps >= 0);

  long maxSteps = max(absM1, absM2);

  long acc1 = 0;   // accumulator of motor movements
  long acc2 = 0;

  for (long i = 0; i < maxSteps; i++) {
    if (stopRequested) {
      moveInterrupted = true;
      Serial.println("ERR,STOPPED");
      return;
    }

    bool moveM1 = false;    // should this motor move this loop
    bool moveM2 = false;

    acc1 += absM1;
    acc2 += absM2;

    if (acc1 >= maxSteps) {
      moveM1 = true;
      acc1 -= maxSteps;
    }

    if (acc2 >= maxSteps) {
      moveM2 = true;
      acc2 -= maxSteps;
    }

    if (moveM1 && moveM2) {
      stepBoth(dir1, dir2);
    }
    else if (moveM1) {
      stepMotor1(dir1);
    }
    else if (moveM2) {
      stepMotor2(dir2);
    }
    if (moveInterrupted) return;
  }
}

void moveBy(int xPixels, int yPixels) {              // converts between pixels and motor steps
  int targetX = currentX + xPixels;
  int targetY = currentY + yPixels;

  if (targetX < 0 || targetX > GRID_MAX || targetY < 0 || targetY > GRID_MAX) {
    Serial.println("ERR,OUT_OF_BOUNDS");
    return;
  }
  
  // convert motor movements into pixels (kinematic transformation from Cartesian XY motion to 2 motors displacements)
  long m1Units = xPixels - yPixels;
  long m2Units = xPixels + yPixels;

  long m1Steps = m1Units * PIXEL_STEP_UNIT;
  long m2Steps = m2Units * PIXEL_STEP_UNIT;

  int prevM1 = currentM1;   // save current motor positions before movement
  int prevM2 = currentM2;

  moveMotors(m1Steps, m2Steps);

  // updating XY and motor positions
  if (!moveInterrupted && currentM1 == prevM1 + m1Steps && currentM2 == prevM2 + m2Steps) {
    currentX = targetX;
    currentY = targetY;

    Serial.print("OK,MOVE,");
    Serial.print(currentX);
    Serial.print(",");
    Serial.print(currentY);
    Serial.println("");

    Serial.print("POS,");
    Serial.print(currentX);
    Serial.print(",");
    Serial.println(currentY); 
  }
  else {
    Serial.println("ERR,MOVE_INTERRUPTED");
    Serial.println("ERR,POSITION_UNKNOWN");
  }
}

void moveTo(int targetX, int targetY) {              // absolute motion command to target coordinate
  if(!positionKnown) {
    Serial.println("ERR,POSITION_UNKNOWN");
    return;
  }
  
  if (targetX < 0 || targetX > GRID_MAX || targetY < 0 || targetY > GRID_MAX) {
    Serial.println("ERR,OUT_OF_BOUNDS");
    return;
  }
  
  int dx = targetX - currentX;
  int dy = targetY - currentY;

  moveBy(dx,dy);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
// ============================ JOYSTICK MODE FUNCTIONS =============================================
//////////////////////////////////////////////////////////////////////////////////////////////////////

void joystickMode() {
  const int stepDelay = 1000;         // stepDelay for overall motor speed  (if joystick mode needed to be controlled slower or faster)
  useLimitSwitch = false;             // disable limit switch in joystick mode (manual control)
  
  // read joystick analog value
  int xValue = analogRead(xPin);
  int yValue = analogRead(yPin);

  // shift joystick values so center = 0
  int xCentered = xValue - 512;
  int yCentered = yValue - 512;

  // DEADZONE/HYSTERESIS (Prevents noise near center from causing movement)
  if (abs(xCentered) < deadZone && abs(yCentered) < deadZone) {
    currentDirection = "Neutral";
  } 
  else {
    // Angle Calculation 
    float angle = atan2(yCentered, xCentered) * 180.0 / PI;     // atan2 gives angle of joystick vector (in degrees)
    if (angle < 0) angle += 360;                                // Convert from [-180,180] → [0,360]

    //  DIRECTION MAPPING (DISCRETIZATION)
    //  Convert continuous angle into discrete motion zones
    // right
    if (angle >= 348.75 || angle < 22.5) currentDirection = "Right";
    // upper right quadrant (30,45,60 degrees)
    else if (angle < 37.5) currentDirection = "UpRight30";
    else if (angle < 52.5) currentDirection = "UpRight45";
    else if (angle < 67.5) currentDirection = "UpRight60";
    // up
    else if (angle < 112.5) currentDirection = "Up";
    // upper left quadrant
    else if (angle < 127.5) currentDirection = "UpLeft60";
    else if (angle < 142.5) currentDirection = "UpLeft45";
    else if (angle < 157.5) currentDirection = "UpLeft30";
    // left
    else if (angle < 202.5) currentDirection = "Left";
    // lower left quadrant
    else if (angle < 217.5) currentDirection = "DownLeft30";
    else if (angle < 232.5) currentDirection = "DownLeft45";
    else if (angle < 247.5) currentDirection = "DownLeft60";
    // down
    else if (angle < 297.5) currentDirection = "Down";
    // lower right quadrant
    else if (angle < 307.5) currentDirection = "DownRight60";
    else if (angle < 322.5) currentDirection = "DownRight45";
    else currentDirection = "DownRight30";
  }

  // PRINT ONLY WHEN CHANGED
  // Reduces serial spam and improves readability
  if (currentDirection != lastDirection) {
    Serial.println(currentDirection);
    lastDirection = currentDirection;
  }
  // do nothing if joystick is at neutral position
  if (currentDirection == "Neutral") return;

  //////////////////////////////////////////////////////
  // USE SHARED MOTOR FUNCTIONS (NO DUPLICATION)
  //////////////////////////////////////////////////////
  // STRAIGHT
  if (currentDirection == "Right") stepBoth(true, true);
  else if (currentDirection == "Left") stepBoth(false, false);
  else if (currentDirection == "Up") stepBoth(false, true);
  else if (currentDirection == "Down") stepBoth(true, false);

  // 45° Diagonal 
  // Only one motor moves --> perfect diagonal
  else if (currentDirection == "UpRight45") stepMotor2(true);
  else if (currentDirection == "UpLeft45") stepMotor1(false);
  else if (currentDirection == "DownRight45") stepMotor1(true);
  else if (currentDirection == "DownLeft45") stepMotor2(false);

  // 30°
  // One motor moves every step, the other every 2 steps
  // Creates asymmetric motion (shallower angle)
  else if (currentDirection == "UpRight30") {
    stepMotor2(true);
    if (stepCounter % 2 == 0) stepMotor1(true);
  }
  else if (currentDirection == "UpLeft30") {
    stepMotor2(false);
    if (stepCounter % 2 == 0) stepMotor1(false);
  }
  else if (currentDirection == "DownLeft30") {
    stepMotor1(false);
    if (stepCounter % 2 == 0) stepMotor2(true);
  }
  else if (currentDirection == "DownRight30") {
    stepMotor1(true);
    if (stepCounter % 2 == 0) stepMotor2(false);
  }

  // 60°
  // Opposite ratio of 30° --> steeper diagonal
  else if (currentDirection == "UpRight60") {
    stepMotor1(true);
    if (stepCounter % 2 == 0) stepMotor2(true);
  }
  else if (currentDirection == "UpLeft60") {
    stepMotor1(false);
    if (stepCounter % 2 == 0) stepMotor2(false);
  }
  else if (currentDirection == "DownLeft60") {
    stepMotor1(false);
    if (stepCounter % 2 == 0) stepMotor2(true);
  }
  else if (currentDirection == "DownRight60") {
    stepMotor1(true);
    if (stepCounter % 2 == 0) stepMotor2(false);
  }

  // used to control step ratio for angled motion
  stepCounter++;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
// ============================ CLOCK MODE FUNCTIONS ================================================
//////////////////////////////////////////////////////////////////////////////////////////////////////

void clockAlarmISR() {              // RTC alarm ISR, set flag only
  clockAlarmFired = true;
}

void eraseCanvas() {                // wipes sand with a full boustrophedon raster scan
  Serial.println("INFO,ERASING");
  useLimitSwitch = true;

  for (int row = 0; row <= GRID_MAX; row++) {
    if (stopRequested || moveInterrupted) return;

    // even rows go left to right, odd rows go right to left
    if (row % 2 == 0) {           
      moveTo(0, row);
      moveTo(GRID_MAX, row);
    }
    else {
      moveTo(GRID_MAX, row);
      moveTo(0, row);
    }
  }
}

void drawDigit(int digit, int originX, int originY) {   // traces one digit at grid position (originX, originY)
  // input validation for digit
  if (digit < 0 || digit > 9) {
    Serial.println("ERR,BAD_DIGIT");
    return;
  }

  // select digit path and length based on input digit using pointer
  const Point* path;
  int pathLen;

  // select path array for the requested digit
  if      (digit == 0) { path = DIGIT_0; pathLen = DIGIT_0_LEN; }
  else if (digit == 1) { path = DIGIT_1; pathLen = DIGIT_1_LEN; }
  else if (digit == 2) { path = DIGIT_2; pathLen = DIGIT_2_LEN; }
  else if (digit == 3) { path = DIGIT_3; pathLen = DIGIT_3_LEN; }
  else if (digit == 4) { path = DIGIT_4; pathLen = DIGIT_4_LEN; }
  else if (digit == 5) { path = DIGIT_5; pathLen = DIGIT_5_LEN; }
  else if (digit == 6) { path = DIGIT_6; pathLen = DIGIT_6_LEN; }
  else if (digit == 7) { path = DIGIT_7; pathLen = DIGIT_7_LEN; }
  else if (digit == 8) { path = DIGIT_8; pathLen = DIGIT_8_LEN; }
  else                 { path = DIGIT_9; pathLen = DIGIT_9_LEN; }

  // iterate through path points and move to each coordinate in sequence to trace out the digit
  for (int i = 0; i < pathLen; i++) {
    if (stopRequested || moveInterrupted) return;

    int tx = constrain(originX + path[i].dx, 0, GRID_MAX);
    int ty = constrain(originY + path[i].dy, 0, GRID_MAX);
    moveTo(tx, ty);
  }
}

void drawClockFace(int hour, int minute) {    // erases sand and draws HH:MM in 24-hour format
  Serial.println("INFO,DRAWING_CLOCK");
  useLimitSwitch = true;

  eraseCanvas();                              // force clean canvas for clock drawing
  if (stopRequested || moveInterrupted) {
    Serial.println("ERR,CLOCK_INTERRUPTED");
    return;
  }

  // decompose time into individual digits using integer division
  int d1 = hour   / 10;       // H tens  (0, 1, or 2)
  int d2 = hour   % 10;       // H units (0-9)
  int d3 = minute / 10;       // M tens  (0-5)
  int d4 = minute % 10;       // M units (0-9)

  drawDigit(d1, DIGIT1_COL, CLOCK_ROW);         // draw digit one
  if (stopRequested || moveInterrupted) return; // check for stop or interrupt

  drawDigit(d2, DIGIT2_COL, CLOCK_ROW);         // draw digit two
  if (stopRequested || moveInterrupted) return; // check for stop or interrupt

  // colon: two small squares so each dot leaves a visible mark in the sand
  moveTo(COLON_COL,     CLOCK_ROW + 1);         
  moveTo(COLON_COL + 1, CLOCK_ROW + 1);
  moveTo(COLON_COL + 1, CLOCK_ROW + 2);
  moveTo(COLON_COL,     CLOCK_ROW + 2);         // upper dot
  if (stopRequested || moveInterrupted) return;

  moveTo(COLON_COL,     CLOCK_ROW + 3);
  moveTo(COLON_COL + 1, CLOCK_ROW + 3);
  moveTo(COLON_COL + 1, CLOCK_ROW + 4);
  moveTo(COLON_COL,     CLOCK_ROW + 4);         // lower dot
  if (stopRequested || moveInterrupted) return;

  drawDigit(d3, DIGIT3_COL, CLOCK_ROW);         // draw digit three
  if (stopRequested || moveInterrupted) return; // check for stop or interrupt

  drawDigit(d4, DIGIT4_COL, CLOCK_ROW);         // draw digit four

  Serial.println("OK,CLOCK_DRAWN");
  Serial.print("POS,");
  Serial.print(currentX);
  Serial.print(",");
  Serial.println(currentY);
}

void clockMode() {                  // called from loop() when in clock mode; checks 5-min interval and redraws
  if (!clockAlarmFired) return;     // only proceed if RTC alarm has fired, indicating a minute has passed

  __disable_irq();                  // disable interrupts to safely access volatile flag
  clockAlarmFired = false;          // reset flag immediately to avoid missing next alarm while processing
  __enable_irq();                   // re-enable interrupts to allow next alarm to trigger

  RTCTime now;                      
  RTC.getTime(now);                     // read current time from RTC
  int currentMinute = now.getMinutes(); // get current minute for timing logic

  
  int elapsed = (currentMinute - lastDrawnMinute + 60) % 60;      // calculate elapsed time since last draw in minutes
                                                                  // modulo handles the wrap from minute 59 back to 0

  // redraw clock if it's the first time (lastDrawnMinute == -1) or if at least 5 minutes have elapsed since the last draw
  if (lastDrawnMinute == -1 || elapsed >= 5) { 
    lastDrawnMinute = currentMinute;
    drawClockFace(now.getHour(), currentMinute);          // draw the clock face with the current hour and minute        
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
// ============================ MAIN ================================================================
//
// Implements external serial interface.
//////////////////////////////////////////////////////////////////////////////////////////////////////

void sendStatus() {                     // sends current machine position and mode to host
  Serial.print("STATUS,");
  Serial.print(positionKnown ? "KNOWN" : "UNKNOWN");
  Serial.print(",");
  Serial.print(currentX);
  Serial.print(",");
  Serial.print(currentY);
  Serial.print(",");
  if      (currentMode == MODE_JOYSTICK)   Serial.println("JOYSTICK");
  else if (currentMode == MODE_CLOCK)      Serial.println("CLOCK");
  else                                     Serial.println("COORDINATE");
}

void setMode(int mode) {                // switches between joystick, coordinate, and clock control modes
  currentMode = mode;
  stopRequested = false;

  if (currentMode == MODE_JOYSTICK) {
    Serial.println("OK,MODE,JOYSTICK");
  }
  else if (currentMode == MODE_CLOCK) {
    lastDrawnMinute = -1;               // force immediate draw on first clock activation
    Serial.println("OK,MODE,CLOCK");
  }
  else {
    Serial.println("OK,MODE,COORDINATE");
  }
}

void processCommand(String input) {     // parses incoming serial commands from python controller
  input.trim();
  input.toUpperCase();

  if (input == "HOME") {
    homeSystem();
    return;
  }

  if (input == "GETPOS") {
    Serial.print("POS,");
    Serial.print(currentX);
    Serial.print(",");
    Serial.println(currentY);
    return;
  }

  if (input == "STATUS") {
    sendStatus();
    return;
  }

  if (input == "STOP") {
    stopRequested = true;
    Serial.println("OK,STOP");
    return;
  }

  if (input == "SETMODE,JOYSTICK") {
    setMode(MODE_JOYSTICK);
    return;
  }

  if (input == "SETMODE,COORDINATE") {
    setMode(MODE_COORDINATE);
    return;
  }

  if (input == "SETMODE,CLOCK") {
    setMode(MODE_CLOCK);
    return;
  }

  if (input == "ERASE") {
    eraseCanvas();
    if (!stopRequested && !moveInterrupted) {
      Serial.println("OK,ERASE");
      Serial.print("POS,");
      Serial.print(currentX);
      Serial.print(",");
      Serial.println(currentY);
    }
    return;
  }

  // SETTIME,HH,MM,SS -- sets the RTC time from the Python controller
  if (input.startsWith("SETTIME,")) {
    int c1 = input.indexOf(',');
    int c2 = input.indexOf(',', c1 + 1);
    int c3 = input.indexOf(',', c2 + 1);
    if (c2 > c1 && c3 > c2) {
      int h = input.substring(c1 + 1, c2).toInt();
      int m = input.substring(c2 + 1, c3).toInt();
      int s = input.substring(c3 + 1).toInt();
      RTCTime t(1, Month::JANUARY, 2025, h, m, s, DayOfWeek::WEDNESDAY, SaveLight::SAVING_TIME_ACTIVE);
      RTC.setTime(t);
      Serial.println("OK,TIME_SET");
    }
    else {
      Serial.println("ERR,BAD_SETTIME");
    }
    return;
  }

  if (input.startsWith("MOVE,")) {
    if (currentMode != MODE_COORDINATE) {
      Serial.println("ERR,WRONG_MODE");
      return;
    }
    int comma1 = input.indexOf(',');
    int comma2 = input.indexOf(',', comma1 + 1);

    if (comma2 > comma1) {
      int x = input.substring(comma1 + 1, comma2).toInt();
      int y = input.substring(comma2 + 1).toInt();
      moveTo(x,y);
      return;
    }
  }
  Serial.println("ERR,BAD_COMMAND");
}

void setup() {                          // hardware initialization and startup
  pinMode(STEP_PIN1, OUTPUT);
  pinMode(DIR_PIN1, OUTPUT);
  pinMode(STEP_PIN2, OUTPUT);
  pinMode(DIR_PIN2, OUTPUT);

  pinMode(HOME_X, INPUT_PULLUP);
  pinMode(HOME_Y, INPUT_PULLUP);
  pinMode(MAX_X, INPUT_PULLUP);
  pinMode(MAX_Y, INPUT_PULLUP);

  pinMode(buttonPin, INPUT_PULLUP);
  
  Serial.begin(115200);
  delay(2000);

  // RTC setup -- set a default time; Python controller can override with SETTIME command
  RTC.begin();
  RTCTime startTime(1, Month::JANUARY, 2025, 0, 0, 0, DayOfWeek::WEDNESDAY, SaveLight::SAVING_TIME_ACTIVE);
  RTC.setTime(startTime);

  // RTC alarm fires every minute at :00 seconds
  // ISR sets clockAlarmFired flag; clockMode() in loop() checks the 5-minute interval
  RTCTime alarmTime;
  alarmTime.setSecond(0);
  AlarmMatch matchRule;
  matchRule.addMatchSecond();
  RTC.setAlarmCallback(clockAlarmISR, alarmTime, matchRule);

  Serial.println("READY");
  Serial.println("MODE,COORDINATE");
}

void loop() {                           // main control loop, checks for incoming serial commands or joystick/clock mode
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    processCommand(input);
  }
  
  if (currentMode == MODE_JOYSTICK) {
    joystickMode();
  }

  if (currentMode == MODE_CLOCK) {
    clockMode();
  }
}
