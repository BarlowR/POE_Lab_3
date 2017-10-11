// ADAFRUIT SETUP
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <EEPROM.h>

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

Adafruit_DCMotor *RightMotor = AFMS.getMotor(1);
Adafruit_DCMotor *LeftMotor = AFMS.getMotor(2);

// SENSOR PINS
const byte RIGHT_SENSOR_PIN = A0;
const byte LEFT_SENSOR_PIN = A1;

// PID VARIABLES
int set_point = 0;
int current_value = 0;
int error = 0;
int prev_error = 0;
int threshold = 0;

// PID CONSTANTS
float k_p;
float k_i;
float k_d;
int PID;

// ROBOT STATES
int datacount = 0;
int right_speed = 150;
int left_speed = 150;
bool right_sensor = 0;
bool left_sensor = 0;


void setup() {
  Serial.begin(9600);
  AFMS.begin();
  
  pinMode(RIGHT_SENSOR_PIN, INPUT);
  pinMode(LEFT_SENSOR_PIN, INPUT);
  
  
// RETRIEVE THRESHOLD CALCULATION FROM EEPROM
  threshold = EEPROM.get(0, threshold);

// BEGIN RUNNING BOTH MOTORS
  set_motor_speeds();
}

// MAIN LOOP
void loop() {

// INCREASE COUNT FOR MATLAB USE
  ++datacount;
  
// CHECK FOR LEFT, RIGHT, MIDDLE, OR END OF TRACK
  error = check_sensors();

// STOP IF CHECK_SENSORS RETURNS 2 (END OF TRACK)
  while (error == 2) {
    error = check_sensors();
  }

// CALCULATE PID, ADJUST MOTORS, AND REPORT DATA TO MATLAB
  calculate_PID();
  adjust_motors();
  send_to_matlab();

// CURRENT ERROR BECOMES PREVIOUS ERROR FOR DERIVATIVE CALC
  prev_error = error;
}

// GIVEN CURRENT MOTOR SPEED VALUES, ADJUST MOTORS
void set_motor_speeds() {
  RightMotor->setSpeed(right_speed);
  RightMotor->run(FORWARD);
  RightMotor->run(RELEASE);

  LeftMotor->setSpeed(left_speed);
  LeftMotor->run(FORWARD);
  LeftMotor->run(RELEASE);   
}

// RETURNS THE FOLLOWING: RIGHT OF LINE = -1, LEFT OF LINE = 1, ON LINE = 0, END OF TRACK = 2
int check_sensors() {
  left_sensor = analogRead(LEFT_SENSOR_PIN) > threshold;
  right_sensor = analogRead(RIGHT_SENSOR_PIN) > threshold;

// IF BOTH SENSORS ARE ABOVE THRESHOLD, BOTH ARE ON BLACK LINE (SIGNIFIES END OF TRACK)
  if (left_sensor && right_sensor) {
    return 2;
  }
  
  return left_sensor - right_sensor;
}

// CALCULATE PID VALUE TO ADD AND SUBTRACT TO MOTORS
float calculate_PID() {
  
  int p = error;
  int i = i + error;
  int d = error - prev_error;

  PID = k_p*p + k_i*i + k_d*d;
}

// CHANGE MOTORS BY PID
float adjust_motors() {
  right_speed = right_speed + PID;
  left_speed = left_speed - PID;
}

// SEND ALL DATA TO MATLAB
void send_to_matlab() {
  Serial.println(String(datacount) + String(",") + String(k_p) + String(",") + String(k_i) + String(",") + String(k_d) + String(",") + String(left_sensor) + String(",") + String(right_sensor) + String(",") + String(left_speed) + String(",") + String(right_speed));  // send average reading to matlab
}
