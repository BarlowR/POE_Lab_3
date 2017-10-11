// ADAFRUIT SETUP
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <EEPROM.h>

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

Adafruit_DCMotor *RightMotor = AFMS.getMotor(2);
Adafruit_DCMotor *LeftMotor = AFMS.getMotor(1);

// SENSOR PINS
const byte RIGHT_SENSOR_PIN = A1;
const byte LEFT_SENSOR_PIN = A0;

// PID VARIABLES
int set_point = 0;
int current_value = 0;
int error = 0;
int prev_error = 0;
int threshold = 0;
const int numOfData = 100;
int pastData[numOfData]; 


// PID CONSTANTS
int k_p = EEPROM.read(1);
float k_i = EEPROM.read(2);
int k_d = EEPROM.read(3);
int PID;

// ROBOT STATES
int datacount = 0;
int right_speed;
int left_speed;
int speed_const = 40;
bool right_sensor = 0;
bool left_sensor = 0;


void setup() {
  Serial.begin(9600);
  AFMS.begin();

  pinMode(RIGHT_SENSOR_PIN, INPUT);
  pinMode(LEFT_SENSOR_PIN, INPUT);


  // RETRIEVE THRESHOLD CALCULATION FROM EEPROM
  threshold = 4 * EEPROM.read(0);

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
    readPID();
    right_speed = 0;
    left_speed = 0;
    set_motor_speeds();
  }

  // CALCULATE PID, ADJUST MOTORS, AND REPORT DATA TO MATLAB
  calculate_PID();
  adjust_motors();
  set_motor_speeds();
  //send_to_matlab();

  // CURRENT ERROR BECOMES PREVIOUS ERROR FOR DERIVATIVE CALC
  prev_error = error;

  readPID();
}

// GIVEN CURRENT MOTOR SPEED VALUES, ADJUST MOTORS
void set_motor_speeds() {
  if (right_speed > 0){
    RightMotor->setSpeed(right_speed);
     RightMotor->run(BACKWARD);
  }
  else {
    RightMotor->setSpeed(abs(right_speed));
    RightMotor->run(FORWARD);
  }
  

  if (left_speed > 0){
    LeftMotor->setSpeed(left_speed);
    LeftMotor->run(FORWARD);
  }
  else{
    LeftMotor->setSpeed(abs(left_speed));
    LeftMotor->run(BACKWARD);
  }

  if (millis()%1000 < 5){
    Serial.print(k_p + (int)(1/k_i) + k_d);
    Serial.print(right_speed);
    Serial.print(left_speed);
    Serial.println(".");
  }

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

  pastData[datacount%numOfData] = error;
  
  int p = error;

  int i = 0; 
  for (int j = 0; j <numOfData; j++){
    i += pastData[j];
  }

  float d = error - prev_error;

  PID = k_p * p + k_i * i + k_d * d;
}





// CHANGE MOTORS BY PID
float adjust_motors() {
  right_speed = speed_const + PID;
  left_speed = speed_const - PID;
}

// SEND ALL DATA TO MATLAB
void send_to_matlab() {
  //Serial.println(String(datacount) + String(",") + String(k_p) + String(",") + String(k_i) + String(",") + String(k_d) + String(",") + String(left_sensor) + String(",") + String(right_sensor) + String(",") + String(left_speed) + String(",") + String(right_speed));  // send average reading to matlab
}

void readPID() {
  String PIDvals;
  while (Serial.available()) {
    PIDvals = Serial.readStringUntil(".");
    if (PIDvals.length() > 10){
      k_p = PIDvals.substring(0, 3).toInt();
      EEPROM.write(1, (byte)k_p);
      k_i = 1.0/PIDvals.substring(4, 7).toInt();
      EEPROM.write(2, (byte)k_i);
      k_d = PIDvals.substring(8, 11).toInt();
      EEPROM.write(3, (byte)k_d);
    }
  }
}

