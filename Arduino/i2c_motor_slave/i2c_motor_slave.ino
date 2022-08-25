//Codigo I2C Escravo para o arduino nano

#include "Wire.h"


#define mL 8
#define mR 9
#define pin_LED 3
#define myAdress 0x08

void setup() {
  Wire.begin(myAdress);
  Wire.onReceive(receiveEvent);

  pinMode(mL, OUTPUT);
  pinMode(mR, OUTPUT);
  pinMode(pin_LED, OUTPUT);

  Serial.begin(9600);
}

void loop() {


}

void receiveEvent(int howMany) {
  // verifica se existem dados para serem lidos no barramento I2C
  if (Wire.available()) {
    // le o byte recebido
    char received = Wire.read();

    // se o byte recebido for igual a 0, apaga o LED
    if (received == 0) {
     digitalWrite(mL, LOW); 
     digitalWrite(mR, LOW);
     digitalWrite(pin_LED, LOW); 
    }

    // se o byte recebido for igual a 1 acende o LED
    if (received == 1) {
      digitalWrite(mL, LOW);
      digitalWrite(mR, HIGH); 
      digitalWrite(pin_LED, HIGH);
    }
    Serial.println(received);
  }
}
