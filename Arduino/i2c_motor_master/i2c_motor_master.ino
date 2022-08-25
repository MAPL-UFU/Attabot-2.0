//Codigo I2C Mestre para o arduino due

#include "Wire.h"

const int pin_SENSOR = 12;

int sensorState = 0;
int prestate = 0;

#define slaveAdress 0x08


void setup() {
  Wire.begin();
  pinMode(pin_SENSOR, INPUT);

}

void loop() {
  sensorState = digitalRead(pin_SENSOR);

  if (sensorState == HIGH && prestate == 0) {

    Wire.beginTransmission(slaveAdress);
    Wire.write(sensorState); 
    Wire.endTransmission();

    prestate = 1;

    delay(500);

  } else if (sensorState == LOW) {

    Wire.beginTransmission(slaveAdress);
    Wire.write(sensorState); 
    Wire.endTransmission();

    prestate = 0;

  }

} 
