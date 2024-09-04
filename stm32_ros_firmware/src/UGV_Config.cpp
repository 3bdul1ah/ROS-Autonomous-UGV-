#include "UGV_Config.h"

Servo Left_Motor;
Servo Right_Motor;

Encoder Left_Encoder(TIM2_CH1_PIN, TIM2_CH2_PIN, SINGLE, 250);       
Encoder Right_Encoder(TIM4_CH1_PIN, TIM4_CH1_PIN, SINGLE, 250);

unsigned long Left_Encoder_Timer = 0;
unsigned long Right_Encoder_Timer = 0;

long Left_Encoder_Ticks = 0;
long Right_Encoder_Ticks = 0;


void initialize_Serial() {
    Serial1.begin(115200);
    // while (!Serial1) {
    // }
    // Serial1.println("Serial1 Initialized");
}

void Configure_Motors() {
    Left_Motor.attach(LEFT_MOTOR_PWM_PIN);
    Right_Motor.attach(RIGHT_MOTOR_PWM_PIN);
}


void Configure_Encoders() {
    if (Left_Encoder.init()) {
        Serial1.println("Left Encoder Initialization OK");
    } else {
        Serial1.println("Left Encoder Initialization Failed");
    }
    
    if (Right_Encoder.init()) {
        Serial1.println("Right Encoder Initialization OK");
    } else {
        Serial1.println("Right Encoder Initialization Failed");
    }
}


void Read_Left_Encoder() {
    Left_Encoder.loop();
    if ((unsigned long)LEFT_ENCODER_READ_DELAY < (unsigned long)(millis() - Left_Encoder_Timer)) {
        Left_Encoder_Ticks = Left_Encoder.getTicks();
        Serial1.println(Left_Encoder_Ticks);
        Left_Encoder_Timer = millis();  
    }
}

void Read_Right_Encoder() {
    Right_Encoder.loop();
    if ((unsigned long)RIGHT_ENCODER_READ_DELAY < (unsigned long)(millis() - Right_Encoder_Timer)) {
    Right_Encoder_Ticks = Right_Encoder.getTicks();
    Serial1.println(Right_Encoder_Ticks);
    Right_Encoder_Timer = millis();  
  }
}