// Sensor Calibration data is stored in EEPROM. This program determines what ADC value is the threshold between the floor and the black tape. 

// Reset the Arduino and place one sensor over the tape and one over the floor. The LED will turn off to indicate data collection is about to begin, then flash at ~20Hz to indicate data collection. Finally, the LED will turn solid to show that data collection is done. 

#include <EEPROM.h>



byte sensor_value; 

const int dataLength = 400;

int sensor_data[2][dataLength];
int dataIndex = 0; 

const byte STATUS_LED = 13;

unsigned long initialTime;

byte address = 0;

boolean done = false; 


void setup() {

 //turn on LED
 pinMode(STATUS_LED, OUTPUT);
 digitalWrite(STATUS_LED, HIGH);

 Serial.begin(9600);

 initialTime = millis();
}

void loop() {
  // Wait 3 seconds
  if (millis()-initialTime < 3000){
    Serial.println("Waiting");
  }
  // Turn off the status LED and wait 2 seconds
  else if (millis()-initialTime < 5000){
     digitalWrite(STATUS_LED, LOW);
     Serial.println("Waiting 2");

  }
  //Collect data
  else if(!done){
      done = getData();
  }
  //Turn LED on to indicate finish
  else {
    digitalWrite(STATUS_LED, HIGH);
  }
}




bool getData(){
  if (dataIndex < dataLength){
    // flash LED at 20Hz to show data collection
    if (millis()%200 < 100)  digitalWrite(STATUS_LED, HIGH);
    else  digitalWrite(STATUS_LED, LOW);

    //collect data
    sensor_data[0][dataIndex] = analogRead(A0);
    sensor_data[1][dataIndex] = analogRead(A1);
    dataIndex++;
    return false;
    delay(20);

  }
  else {

    // take the average of all of the data, then avaeage the two results and divide it by 4 to save in EEPROM
    long sum = 0;
    for (int i = 0; i < dataLength; i ++){
      sum += sensor_data[0][i];
      
    }
    int sensor1 = sum/dataLength;

    
    sum = 0;
    for (int i = 0; i < dataLength; i ++){
      sum += sensor_data[1][i];
    }
    int sensor2 = sum/dataLength;    

    sensor_value = (sensor1+sensor2)/8;
    EEPROM.write(address, sensor_value);
    Serial.print(sensor_value);
    return true;
  }
}


