#include <Arduino.h>
HardwareSerial Serial1(PA3, PA2);

void setup() {
  Serial1.begin(57600);
  pinMode(PC13, OUTPUT);
  
  
}

void loop() {
  Serial1.println("DFU is Working!");
  digitalWrite( PC13 , HIGH);
  delay(5000);
  digitalWrite( PC13 , LOW);
  delay(5000);


}