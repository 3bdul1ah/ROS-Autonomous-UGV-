// // #include <Servo.h>
// // #include <Arduino.h>

// // HardwareSerial Serial1(PA3, PA2);

// // #define LEFT_MOTOR_PWM_PIN PB10
// // #define RIGHT_MOTOR_PWM_PIN PB9

// // Servo leftMotor;
// // Servo rightMotor;

// // void setup() {
// //     Serial1.begin(115200);

// //     leftMotor.attach(LEFT_MOTOR_PWM_PIN);
// //     rightMotor.attach(RIGHT_MOTOR_PWM_PIN);
// // }

// // void loop() {
// //   rightMotor.write(180); 
// //   Serial1.print("FW\n");

// // }

// ////

// #include <Servo.h>
// #include <Arduino.h>
// #include "FastInterruptEncoder.h"

// HardwareSerial Serial1(PA3, PA2);

// #define ENCODER_READ_DELAY    300

// #define RIGHT_MOTOR_PWM_PIN PB9
// Servo rightMotor;


// Encoder enc(PB6, PB7 /* or HALFQUAD or FULLQUAD */, FULLQUAD /* Noise and Debounce Filter (default 0) */); // - Example for STM32, check datasheet for possible Timers for Encoder mode. TIM_CHANNEL_1 and TIM_CHANNEL_2 only
// //Encoder enc(25, 26, SINGLE, 250);  - Example for ESP32

// unsigned long encodertimer = 0;

// void setup() {
//   Serial1.begin(115200);

//   rightMotor.attach(RIGHT_MOTOR_PWM_PIN);

  
//   if (enc.init()) {
//     Serial1.println("Encoder Initialization OK");
//   } else {
//     Serial.println("Encoder Initialization Failed");
//     while(1);
//   }

// }

// void loop() {

//   rightMotor.writeMicroseconds(1499); 

//   enc.loop();                     //better to use with Timer Interrupt
                  
//   if ((unsigned long)ENCODER_READ_DELAY < (unsigned long)(millis() - encodertimer)) {
//     Serial1.println(enc.getTicks());
//     encodertimer = millis();  
//   }
// }
