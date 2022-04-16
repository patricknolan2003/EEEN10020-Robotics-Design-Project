#include <LCD16x2.h> //allows LCD to be used
#include <Wire.h>
#include <math.h>
#include <NineAxesMotion.h>
#include <EnableInterrupt.h>

LCD16x2 lcd;

NineAxesMotion mySensor;

int E1 = 5; //Port for M1 Speed Control
int E2 = 6; //Port for M2 Speed Control

int M1 = 4; //Port for M1 Direction Control
int M2 = 7; //Port for M2 Direction Control

int StraightSpeedM1 = 200;
int StraightSpeedM2 = 200;

volatile double x = 0;
volatile double y = 0;
volatile int loopcount = 0;
const int OpticalSensor = A0;
const int interruptPin = 2;

volatile int CycleCount = 0; // use volatile for shared variables
volatile int MatchLength = 180; // 180 seconds is 3 minutes

void TurnRight() {

  analogWrite(E1, 175); //Set M1 speed
  digitalWrite(M1, LOW); //Set M1 direction
  analogWrite(E2, 175); //Set M2 speed
  digitalWrite(M2, LOW); //Set M2 direction
  return;

}
void TurnLeft() {

  analogWrite(E1, 175); //Set M1 speed
  digitalWrite(M1, HIGH); //Set M1 direction
  analogWrite(E2, 175); //Set M2 speed
  digitalWrite(M2, HIGH); //Set M2 direction
  return;

}
double WhatHeadingIsClockwiseOfCurrent(double currentheading, double degrees1) {
  while (degrees1 > 0) {
    if (currentheading >= 360) {
      currentheading = currentheading - 360;
    }
    currentheading = currentheading + 1;
    degrees1 = degrees1 - 1;
    if (currentheading >= 360) {
      currentheading = currentheading - 360;
    }

  }
  return currentheading;

}
double WhatHeadingIsAntiClockwiseOfCurrent(double currentheading1, double degrees2) {
  while (degrees2 > 0) {

    if (currentheading1 <= 0) {
      currentheading1 = currentheading1 + 360;
    }
    currentheading1 = currentheading1 - 1;
    degrees2 = degrees2 - 1;

    if (currentheading1 < 0) {
      currentheading1 = currentheading1 + 360;
    }

  }
  return currentheading1;

}

void TurnClockwise(double degrees3) {
  double LiveHeading1 = mySensor.readEulerHeading();
  double TargetHeadingClockwise = WhatHeadingIsClockwiseOfCurrent(LiveHeading1, degrees3);
  while (((TargetHeadingClockwise + 1) < LiveHeading1) || ((TargetHeadingClockwise - 1) > LiveHeading1)) {
    LiveHeading1 = mySensor.readEulerHeading();
    TurnRight();
  }
}
void TurnAntiClockwise(double degrees4) {
  double LiveHeading2 = mySensor.readEulerHeading();
  double TargetHeadingAntiClockwise = WhatHeadingIsAntiClockwiseOfCurrent(LiveHeading2, degrees4);
  while (((TargetHeadingAntiClockwise + 1) < LiveHeading2) || ((TargetHeadingAntiClockwise - 1) > LiveHeading2)) {
    LiveHeading2 = mySensor.readEulerHeading();
    TurnLeft();
  }
}

double GetHeadingError(double TargetHeading, double CurrentHeading) {

  double ClockwiseDirection = CurrentHeading - TargetHeading;
  double AntiClockwiseDirection = 360 - (CurrentHeading + TargetHeading);
  double error = 0;

  if (abs(ClockwiseDirection) < abs(AntiClockwiseDirection)) {
    error = ClockwiseDirection;
  } else {
    error = AntiClockwiseDirection;
  }

  return error;
}

void GoForward(double time1) {

  double CurrentTime = millis();
  double headingOriginal = mySensor.readEulerHeading();

  double LiveHeading;
  int RightMotorSpeed;
  int LeftMotorSpeed;
  double AngleError;
  double Response;

  while (millis() < (CurrentTime + time1)) {
    LiveHeading = mySensor.readEulerHeading();
    AngleError = GetHeadingError(headingOriginal, LiveHeading);

    Response = AngleError * 70;

    RightMotorSpeed = StraightSpeedM2 + Response;
    LeftMotorSpeed = StraightSpeedM1 - Response;

    if (RightMotorSpeed < 0) {
      RightMotorSpeed = 0;
    }
    if (LeftMotorSpeed < 0) {
      LeftMotorSpeed = 0;
    }

    if (RightMotorSpeed > 254) {
      RightMotorSpeed = 254;
    }
    if (LeftMotorSpeed > 254) {
      LeftMotorSpeed = 254;
    }

    analogWrite(E1, RightMotorSpeed); //Set M1 speed
    digitalWrite(M1, HIGH); //Set M1 direction
    analogWrite(E2, LeftMotorSpeed); //E2 is overpowered
    digitalWrite(M2, LOW); //Set M2 direction

  }
  return;

}

void DistanceTravelled() // interrupt function for rotations
{
  double OpticalValue = analogRead(A0);
  double HeadingAngle4Coordinates = mySensor.readEulerHeading();

  if (OpticalValue <= 300) {
    x = x + (15 * cos(HeadingAngle4Coordinates)); //check that this is the distance the bot drives in one rotation
    y = y + (15 * sin(HeadingAngle4Coordinates));
  }
}

void arm_and_start() {

  int buttons = 0;

  float timeleft;

  long countdown_int;

  long fudge = 0;
  bool mode = 0;
  long timeflipped = 0;

  Wire.begin();

  lcd.lcdClear();
  lcd.lcdSetBlacklight(200);

  lcd.lcdGoToXY(1, 2);
  lcd.lcdWrite("v");

  lcd.lcdGoToXY(1, 1);
  lcd.lcdWrite("SYNC ");

  lcd.lcdGoToXY(12, 1);
  lcd.lcdWrite(" SYNC");

  lcd.lcdGoToXY(4, 2);
  lcd.lcdWrite("[--set--]");

  buttons = lcd.readButtons(); //Sample the state of the large white buttons

  do {
    buttons = lcd.readButtons(); //Sample the state of the large white buttons

    if (buttons == 14) //leftmost button depressed
    {
      fudge = fudge + 450;
    }

    buttons = lcd.readButtons(); //Sample the state of the large white buttons

    if (buttons == 9 && (millis() - timeflipped) > 250) //both middle buttons depressed
    {
      mode = !mode;
      timeflipped = millis();

      if (!mode) //syncing safe mode
      {
        lcd.lcdSetBlacklight(100);

        lcd.lcdGoToXY(1, 1);
        lcd.lcdWrite("SYNC ");

        lcd.lcdGoToXY(12, 1);
        lcd.lcdWrite(" SYNC");

        lcd.lcdGoToXY(4, 2);
        lcd.lcdWrite("[--set--]");

      } else if (mode) //armed mode
      {
        lcd.lcdSetBlacklight(400);
        lcd.lcdGoToXY(1, 1);
        lcd.lcdWrite("ARMED");

        lcd.lcdGoToXY(12, 1);
        lcd.lcdWrite("ARMED");

        lcd.lcdGoToXY(4, 2);
        lcd.lcdWrite("[-disarm-]");
      }

    }

    countdown_int = (millis() + fudge) % 10000;

    countdown_int = 9998 - countdown_int;

    lcd.lcdGoToXY(7, 1);
    lcd.lcdWrite((float) countdown_int / 1000, 2);

  }
  while (!(mode == 1 && countdown_int <= 750));

  lcd.lcdClear();
  lcd.lcdSetBlacklight(200);
  lcd.lcdGoToXY(6, 2);
  lcd.lcdWrite("Begin!");

  enableInterrupt(11, CheckCycle, RISING); //Attach an interrupt to pin 11
  tone(11, 31); //Output a 31 Hz square wave to pin 11 to trigger this interrupt
}

void CheckCycle(void) {
  CycleCount = CycleCount + 1; //Check how many times we've been here

  if (CycleCount == 31 * MatchLength) {
    //turn off motors
    analogWrite(E1, 0); //Turn off M1
    analogWrite(E2, 0); //Turn off M2

    while (1) //loop forever to shut down Arduino
    {}
  }
}

void setup() //This code is executed once
{
  int buttons;
  float positions[8][2];

  //Peripheral Initialization
  Serial.begin(9600); //Initialize the Serial Port to view information on the Serial Monitor
  I2C.begin(); //Initialize I2C communication to the let the library communicate with the sensor.
  Wire.begin();

  //Sensor Initialization
  mySensor.initSensor(); //The I2C Address can be changed here inside this function in the library
  mySensor.setOperationMode(OPERATION_MODE_NDOF); //NDOF = 9 Degrees of Freedom Sensor (Other operatin modes are available)
  mySensor.setUpdateMode(AUTO);

  pinMode(E1, OUTPUT); //Set up motor pins
  pinMode(E2, OUTPUT);
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  pinMode(OpticalSensor, OUTPUT);
  pinMode(A0, INPUT_PULLUP);

  lcd.lcdClear();
  arm_and_start();

}

void loop() {

  while (loopcount == 0) {
    GoForward(3000);
    TurnClockwise(90);
    GoForward(6500);
    TurnAntiClockwise(90);
    GoForward(14500);
    TurnClockwise(90);
    GoForward(7000); //table length
    TurnClockwise(90);
    GoForward(3500);
    TurnClockwise(90);
    GoForward(7000);
    TurnAntiClockwise(90);
    GoForward(11000);
    TurnClockwise(90);
    GoForward(6500);
    TurnAntiClockwise(90);
    GoForward(3000);

    loopcount++;
  }

}