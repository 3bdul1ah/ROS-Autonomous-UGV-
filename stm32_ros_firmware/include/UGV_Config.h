#ifndef UGV_CONFIG_H
#define UGV_CONFIG_H

#include <Arduino.h>
#include <Servo.h>
#include "FastInterruptEncoder.h"

#define UGV_LEFT_MOTOR_PWM_PIN        PA6
#define UGV_RIGHT_MOTOR_PWM_PIN       PB9

#define UGV_BAUD_RATE                57600

#define UGV_ENCODER_READ_DELAY   300

#define UGV_TIM1_CH1_PIN    PA8
#define UGV_TIM1_CH2_PIN    PA9

#define UGV_TIM2_CH1_PIN    PA0 // CHA L
#define UGV_TIM2_CH2_PIN    PA1 // CHB L

#define UGV_TIM3_CH1_PIN    PA6
#define UGV_TIM3_CH2_PIN    PA7

#define UGV_TIM4_CH1_PIN    PB6 // CHB R
#define UGV_TIM4_CH2_PIN    PB7 // CHA R

#define UGV_TIM5_CH1_PIN    PA0
#define UGV_TIM5_CH2_PIN    PA1

#define UGV_WHEEL_BASE 1.2
#define UGV_WHEEL_RADIUS 0.257
#define UGV_MAX_VELOCITY 0.4306073

extern Servo UGV_Left_Motor;
extern Servo UGV_Right_Motor;

extern Encoder UGV_Left_Encoder;
extern Encoder UGV_Right_Encoder;

extern unsigned long UGV_Encoder_Timer;

extern long UGV_Left_Encoder_Ticks;
extern long UGV_Right_Encoder_Ticks;

void UGV_initialize_Serial();
void UGV_Configure_Motors();
void UGV_Configure_Encoders();
void UGV_Read_Encoders();

#endif // UGV_CONFIG_H
