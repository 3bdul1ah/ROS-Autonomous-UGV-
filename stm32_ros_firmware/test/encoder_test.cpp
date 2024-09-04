// #include <Servo.h>
// HardwareSerial Serial1(PA3, PA2);

// #define LEFT_MOTOR_PWM_PIN 6
// #define RIGHT_MOTOR_PWM_PIN 7
// #define LEFT_ENCODER_A_PIN 2
// #define LEFT_ENCODER_B_PIN 3
// #define RIGHT_ENCODER_A_PIN 4
// #define RIGHT_ENCODER_B_PIN 5

// #define MOTOR_FORWARD_PWM 2000
// #define MOTOR_BACKWARD_PWM 1000
// #define MOTOR_STOP_PWM 1500

// Servo leftMotor;
// Servo rightMotor;

// STM32encoder leftEncoder(TIM3, LEFT_ENCODER_A_PIN, LEFT_ENCODER_B_PIN);
// STM32encoder rightEncoder(TIM4, RIGHT_ENCODER_A_PIN, RIGHT_ENCODER_B_PIN);

// void setup() {
//     Serial.begin(57600);
//     leftMotor.attach(LEFT_MOTOR_PWM_PIN);
//     rightMotor.attach(RIGHT_MOTOR_PWM_PIN);
//     leftEncoder.dynamic(1, 10, true);
//     rightEncoder.dynamic(1, 10, true);
//     leftMotor.writeMicroseconds(MOTOR_STOP_PWM);
//     rightMotor.writeMicroseconds(MOTOR_STOP_PWM);
// }

// void loop() {
//     leftMotor.writeMicroseconds(MOTOR_FORWARD_PWM);
//     rightMotor.writeMicroseconds(MOTOR_FORWARD_PWM);
//     delay(2000);

//     Serial.print("Left Encoder: ");
//     Serial.print(leftEncoder.pos());
//     Serial.print("\tRight Encoder: ");
//     Serial.println(rightEncoder.pos());

//     leftMotor.writeMicroseconds(MOTOR_STOP_PWM);
//     rightMotor.writeMicroseconds(MOTOR_STOP_PWM);
//     delay(2000);

//     leftMotor.writeMicroseconds(MOTOR_BACKWARD_PWM);
//     rightMotor.writeMicroseconds(MOTOR_BACKWARD_PWM);
//     delay(2000);

//     Serial.print("Left Encoder: ");
//     Serial.print(leftEncoder.pos());
//     Serial.print("\tRight Encoder: ");
//     Serial.println(rightEncoder.pos());

//     leftMotor.writeMicroseconds(MOTOR_STOP_PWM);
//     rightMotor.writeMicroseconds(MOTOR_STOP_PWM);
//     delay(2000);
// }
    ///////////////////////
#include <Servo.h>
#include <Arduino.h>
#include "FastInterruptEncoder.h"

HardwareSerial Serial1(PA3, PA2);

#define ENCODER_READ_DELAY    300

#define RIGHT_MOTOR_PWM_PIN PB9
Servo rightMotor;


Encoder enc(PB6, PB7 /* or HALFQUAD or FULLQUAD */, FULLQUAD /* Noise and Debounce Filter (default 0) */); // - Example for STM32, check datasheet for possible Timers for Encoder mode. TIM_CHANNEL_1 and TIM_CHANNEL_2 only
//Encoder enc(25, 26, SINGLE, 250);  - Example for ESP32

unsigned long encodertimer = 0;

void setup() {
  Serial1.begin(115200);

  rightMotor.attach(RIGHT_MOTOR_PWM_PIN);

  
  if (enc.init()) {
    Serial1.println("Encoder Initialization OK");
  } else {
    Serial.println("Encoder Initialization Failed");
    while(1);
  }

}

void loop() {

  rightMotor.writeMicroseconds(2000); 

  enc.loop();                     //better to use with Timer Interrupt
                  
  if ((unsigned long)ENCODER_READ_DELAY < (unsigned long)(millis() - encodertimer)) {
    Serial1.println(enc.getTicks());
    encodertimer = millis();  
  }
}
