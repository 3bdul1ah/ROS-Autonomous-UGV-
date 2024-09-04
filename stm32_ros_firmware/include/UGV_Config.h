#include <Arduino.h>
#include <Servo.h>
#include "FastInterruptEncoder.h"


#define LEFT_MOTOR_PWM_PIN        PA6
#define RIGHT_MOTOR_PWM_PIN       PB9

/* - - - - - - - - - - - - - - - - - - - - -
 Encoder NAME(X,Y,Z,T)                      |
 > X: (CH1(TIMX)                            |
 > Y: CH2(TIMX)                             |
 > Z: TYPE: SINGLE or HALFQUAD or FULLQUAD  |
 > T: Noise and Debounce Filter: Default 0  |
 - - - - - - - - - - - - - - - - - - - - - */ 

#define LEFT_ENCODER_READ_DELAY   300
#define RIGHT_ENCODER_READ_DELAY  300

#define TIM1_CH1_PIN    PA8 
#define TIM1_CH2_PIN    PA9 

#define TIM2_CH1_PIN    PA0 // CHA L
#define TIM2_CH2_PIN    PA1 // CHB L

#define TIM3_CH1_PIN    PA6
#define TIM3_CH2_PIN    PA7

#define TIM4_CH1_PIN    PB6 // CHB R
#define TIM4_CH2_PIN    PB7 // CHA R

#define TIM5_CH1_PIN    PA0
#define TIM5_CH2_PIN    PA1

#define TIM8_CH1_PIN    PC6
#define TIM8_CH2_PIN    PC7

#define TIM9_CH1_PIN    PA2
#define TIM9_CH2_PIN    PA3

#define WHEEL_BASE 1.2
#define WHEEL_RADIUS 0.257
#define MAX_VELOCITY 0.4306073

#ifndef UGV_CONFIG_H
#define UGV_CONFIG_H    

extern Servo Left_Motor;     
extern Servo Right_Motor;

extern Encoder Left_Encoder;
extern Encoder Right_Encoder;

extern unsigned long Encoder_Timer;

extern long Left_Encoder_Ticks;
extern long Right_Encoder_Ticks;

void initialize_Serial();
void Configure_Motors();
void Configure_Encoders();
void Read_Left_Encoder();
void Read_Right_Encoder();

#endif // UGV_CONFIG_H
