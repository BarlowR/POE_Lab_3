// ADAFRUIT SETUP
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

Adafruit_DCMotor *RightMotor = AFMS.getMotor(1);
Adafruit_DCMotor *LeftMotor = AFMS.getMotor(2);

// SENSOR PINS
const byte RIGHT_SENSOR = A0;
const byte LEFT_SENSOR = A1;

// PID VARIABLES
int set_point;
int current_value;
int error = 0;
int prev_error = 0;

// PID CONSTANTS
float k_p;
float k_i;
float k_d;
float PID;

// ROBOT STATES
bool track_end = false;

void setup() {
  Serial.begin(9600);
  AFMS.begin();

// BEGIN RUNNING BOTH MOTORS
  set_motor_speeds(150, 150);
}

// calculates PID and error
void loop() {
  
// stop running if at track end
  track_end = check_sensors();
  while (track_end) {
  }

  PID = (int) calculate_PID(error);
  prev_error = error;
  error = adjust_motors(PID);
}

void set_motor_speeds(int right_speed, int left_speed) {
  RightMotor->setSpeed(right_speed);
  RightMotor->run(FORWARD);
  RightMotor->run(RELEASE);

  LeftMotor->setSpeed(left_speed);
  LeftMotor->run(FORWARD);
  LeftMotor->run(RELEASE);   
}

bool check_sensors() {
// says whether robot is at track end or not
  return track_end
}

// calculate and return the PID
float calculate_PID(error) {
  
  p = error;
  i = i + error;
  d = error - prev_error

  return k_p*p + k_i*i + k_d*d;
}

// returns error and changes motor speed according to PID
float adjust_motors(PID) {
  
  // add PID to one motor and subtract from the other
  set_motor_speeds(current_value + PID, current_value - PID);

  // return the error
  return set_point - current_value;
}

