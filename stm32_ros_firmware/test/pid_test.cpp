// #include <Servo.h>
// #include <STM32encoder.h>
// #include <PID_v1.h>
// HardwareSerial Serial1(PA3, PA2);

// #define LEFT_MOTOR_PWM_PIN 6
// #define RIGHT_MOTOR_PWM_PIN 7
// #define LEFT_ENCODER_A_PIN 2
// #define LEFT_ENCODER_B_PIN 3
// #define RIGHT_ENCODER_A_PIN 4
// #define RIGHT_ENCODER_B_PIN 5

// #define MOTOR_STOP_PWM 1500

// #define KP 1.0
// #define KI 0.1
// #define KD 0.1

// Servo leftMotor;
// Servo rightMotor;

// STM32encoder leftEncoder(TIM3, LEFT_ENCODER_A_PIN, LEFT_ENCODER_B_PIN);
// STM32encoder rightEncoder(TIM4, RIGHT_ENCODER_A_PIN, RIGHT_ENCODER_B_PIN);

// double leftSpeedSetpoint = 0, rightSpeedSetpoint = 0;
// double leftSpeed = 0, rightSpeed = 0;
// double leftMotorPWM = MOTOR_STOP_PWM, rightMotorPWM = MOTOR_STOP_PWM;

// PID leftPID(&leftSpeed, &leftMotorPWM, &leftSpeedSetpoint, KP, KI, KD, DIRECT);
// PID rightPID(&rightSpeed, &rightMotorPWM, &rightSpeedSetpoint, KP, KI, KD, DIRECT);

// void setup() {
//     Serial.begin(57600);
//     leftMotor.attach(LEFT_MOTOR_PWM_PIN);
//     rightMotor.attach(RIGHT_MOTOR_PWM_PIN);
//     leftEncoder.dynamic(1, 10, true);
//     rightEncoder.dynamic(1, 10, true);
//     leftPID.SetMode(AUTOMATIC);
//     rightPID.SetMode(AUTOMATIC);
// }

// void loop() {
//     long leftPosition = leftEncoder.pos();
//     long rightPosition = rightEncoder.pos();

//     static long prevLeftPosition = 0, prevRightPosition = 0;
//     static unsigned long prevTime = millis();
//     unsigned long currentTime = millis();
//     double dt = (currentTime - prevTime) / 1000.0;

//     leftSpeed = (leftPosition - prevLeftPosition) / dt;
//     rightSpeed = (rightPosition - prevRightPosition) / dt;

//     prevLeftPosition = leftPosition;
//     prevRightPosition = rightPosition;
//     prevTime = currentTime;

//     leftPID.Compute();
//     rightPID.Compute();

//     leftMotor.writeMicroseconds(constrain(leftMotorPWM, 1000, 2000));
//     rightMotor.writeMicroseconds(constrain(rightMotorPWM, 1000, 2000));

//     Serial.print("Left Speed: ");
//     Serial.print(leftSpeed);
//     Serial.print("\tLeft PWM: ");
//     Serial.println(leftMotorPWM);

//     Serial.print("Right Speed: ");
//     Serial.print(rightSpeed);
//     Serial.print("\tRight PWM: ");
//     Serial.println(rightMotorPWM);

//     delay(100); 
// }


#include <Servo.h>
#include <Arduino.h>
#include "FastInterruptEncoder.h"

HardwareSerial Serial1(PA3, PA2);

#define LEFT_ENCODER_READ_DELAY   300
#define RIGHT_ENCODER_READ_DELAY  300

#define LEFT_MOTOR_PWM_PIN        PA5
#define RIGHT_MOTOR_PWM_PIN       PB9

Servo Left_Motor;
Servo Right_Motor;

Encoder Left_Encoder(PA6, PA7, SINGLE, 250);       
Encoder Right_Encoder(PB6, PB7, SINGLE, 250);

unsigned long LeftEncoderTimer = 0;
unsigned long RightEncoderTimer = 0;

void setup() {
  Serial1.begin(115200);

  Left_Motor.attach(LEFT_MOTOR_PWM_PIN);
  Right_Motor.attach(RIGHT_MOTOR_PWM_PIN);

  Left_Motor.writeMicroseconds(1500);
  Right_Motor.writeMicroseconds(1500); 

  if (Left_Encoder.init()) {
    Serial1.println("Left Encoder Initialization OK");
  } else {
    Serial1.println("Left Encoder Initialization Failed");
    while(1);
  }

  if (Right_Encoder.init()) {
    Serial1.println("Right Encoder Initialization OK");
  } else {
    Serial1.println("Right Encoder Initialization Failed");
    while(1);
  }
}

void loop() {
  Left_Motor.writeMicroseconds(1500);
  Right_Motor.writeMicroseconds(1500); 

  Left_Encoder.loop();
  if (millis() - LeftEncoderTimer > LEFT_ENCODER_READ_DELAY) {
    Serial1.print("Left Encoder Ticks: ");
    Serial1.println(Left_Encoder.getTicks());
    LeftEncoderTimer = millis();  
  }

  Right_Encoder.loop();
  if (millis() - RightEncoderTimer > RIGHT_ENCODER_READ_DELAY) {
    Serial1.print("Right Encoder Ticks: ");
    Serial1.println(Right_Encoder.getTicks());
    RightEncoderTimer = millis();  
  }
}
