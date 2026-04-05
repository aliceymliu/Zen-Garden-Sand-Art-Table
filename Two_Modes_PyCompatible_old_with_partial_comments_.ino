//////////////////////////////////////////////////////////////////////////////////////////////////////
// MIE438 FINAL PROJECT - ZEN GARDEN SAND ART TABLE                                                 
//                                                                                                  
// PURPOSE:                         
// Controls a 2-motor rail gantry capable of:
// 1) Coordinate-based motion, moves requested via (x,y) coordinate location
// 2) Manual joystick motion with continuous directional control
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
//////////////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////////////////////////
// MODE SELECT
// Two operating modes:
// MODE_JOYSTICK uses real time manual control with an analog joystick input.
// MODE_COORDINATE receives target XY coordinates over serial and performs automated movements.
//
// Default startup mode in coordinates since the art table is designed to be primarily autonomous.
//////////////////////////////////////////////////////////////////////////////////////////////////////
#define MODE_JOYSTICK 1
#define MODE_COORDINATE 2

int currentMode = MODE_COORDINATE; //default coordinate startup mode

//////////////////////////////////////////////////////////////////////////////////////////////////////
// ============================ COORDINATE MODE VARIABLE =============================================
//
//Variables defining hardware interface for stepper motors and limit switches, and motion scaling constants.
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
// ============================ SHARED LOW LEVEL FUNCTIONS ===========================================
//
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
    digitalWrite(DIR_PIN1, HIGH); // HOME_X pressed, move +X
    digitalWrite(DIR_PIN2, HIGH);
  }
  else {
    digitalWrite(DIR_PIN1, LOW); // MAX_X pressed, move -X
    digitalWrite(DIR_PIN2, LOW);
  }

  for (int i = 0; i < backoffStepsX; i++) {
    digitalWrite(STEP_PIN1, HIGH);
    digitalWrite(STEP_PIN2,HIGH);
    delayMicroseconds(pulseWidth);
    digitalWrite(STEP_PIN1, LOW);
    digitalWrite(STEP_PIN2, LOW);
    delayMicroseconds(stepDelay);
  }
}

void backoffY(bool away) {          // moves gantry a short distance away from x-axis limit switch after trigger
  if (away) {
    digitalWrite(DIR_PIN1, LOW); // HOME_Y pressed, move +Y
    digitalWrite(DIR_PIN2, HIGH);
  }
  else {
    digitalWrite(DIR_PIN1, HIGH); // MAX_Y pressed, move -Y
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

bool limitCheck() {                 // checks all limit switch readings and handles any switch event
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
 
void stepMotor1(bool dir) {         // steps motor 1 by one step if safe to do so
  if (limitCheck() || stopRequested) return;

  digitalWrite(DIR_PIN1, dir ? HIGH:LOW);
  pulseStep(STEP_PIN1);

  currentM1 += (dir ? 1:-1);

  if (limitCheck() || stopRequested) return;
}

void stepMotor2(bool dir) {         // steps motor 2 by one step if safe to do so
  if (limitCheck() || stopRequested) return;

  digitalWrite(DIR_PIN2, dir ? HIGH:LOW);
  pulseStep(STEP_PIN2);

  currentM2 += (dir ? 1:-1);

  if (limitCheck() || stopRequested) return;
}

void stepBoth(bool dir1, bool dir2) {     // steps both motors simultaneously by one step if safe to do so
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

void homeSystem() {                 // performs homing to establish a known reference location
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
  
  // convert motor movements into pixels
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
  const int stepDelay = 1000;         /// For joystick smoother version we are using 600
  useLimitSwitch = false;
  int xValue = analogRead(xPin);
  int yValue = analogRead(yPin);

  int xCentered = xValue - 512;
  int yCentered = yValue - 512;

  // DEADZONE
  if (abs(xCentered) < deadZone && abs(yCentered) < deadZone) {
    currentDirection = "Neutral";
  } 
  else {
    float angle = atan2(yCentered, xCentered) * 180.0 / PI;
    if (angle < 0) angle += 360;

    if (angle >= 348.75 || angle < 22.5) currentDirection = "Right";
    else if (angle < 37.5) currentDirection = "UpRight30";
    else if (angle < 52.5) currentDirection = "UpRight45";
    else if (angle < 67.5) currentDirection = "UpRight60";
    else if (angle < 112.5) currentDirection = "Up";
    else if (angle < 127.5) currentDirection = "UpLeft60";
    else if (angle < 142.5) currentDirection = "UpLeft45";
    else if (angle < 157.5) currentDirection = "UpLeft30";
    else if (angle < 202.5) currentDirection = "Left";
    else if (angle < 217.5) currentDirection = "DownLeft30";
    else if (angle < 232.5) currentDirection = "DownLeft45";
    else if (angle < 247.5) currentDirection = "DownLeft60";
    else if (angle < 297.5) currentDirection = "Down";
    else if (angle < 307.5) currentDirection = "DownRight60";
    else if (angle < 322.5) currentDirection = "DownRight45";
    else currentDirection = "DownRight30";
  }

  // PRINT ONLY WHEN CHANGED
  if (currentDirection != lastDirection) {
    Serial.println(currentDirection);
    lastDirection = currentDirection;
  }

  if (currentDirection == "Neutral") return;

  //////////////////////////////////////////////////////
  // USE SHARED MOTOR FUNCTIONS (NO DUPLICATION)
  //////////////////////////////////////////////////////

  // STRAIGHT
  if (currentDirection == "Right") stepBoth(true, true);
  else if (currentDirection == "Left") stepBoth(false, false);
  else if (currentDirection == "Up") stepBoth(false, true);
  else if (currentDirection == "Down") stepBoth(true, false);

  // 45°
  else if (currentDirection == "UpRight45") stepMotor2(true);
  else if (currentDirection == "UpLeft45") stepMotor1(false);
  else if (currentDirection == "DownRight45") stepMotor1(true);
  else if (currentDirection == "DownLeft45") stepMotor2(false);

  // 30°
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

  stepCounter++;
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
  Serial.println(currentMode == MODE_JOYSTICK ? "JOYSTICK" : "COORDINATE");
}

void setMode (int mode) {               // switches between joystick and coordinate control modes
  currentMode = mode;

  if (currentMode == MODE_JOYSTICK) {
    Serial.println("OK,MODE,JOYSTICK");
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

  Serial.println("READY");
  Serial.println("MODE,COORDINATE");
}

void loop() {                           // main control loop, checks for incoming serial commands or joystick manual control
  if(Serial.available()) {
    String input = Serial.readStringUntil('\n');
    processCommand(input);
  }
  
  if (currentMode == MODE_JOYSTICK) {
    joystickMode();
  }
}

