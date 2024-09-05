#include "UGV_Config.h"

Servo Left_Motor;
Servo Right_Motor;

Encoder Left_Encoder(TIM2_CH1_PIN, TIM2_CH2_PIN, SINGLE, 0);
Encoder Right_Encoder(TIM4_CH1_PIN, TIM4_CH2_PIN, SINGLE, 0);

unsigned long Encoder_Timer = 0;

long Left_Encoder_Ticks = 0;
long Right_Encoder_Ticks = 0;

void initialize_Serial() {
    Serial1.begin(BAUD_RATE);
    // while (!Serial1) {
    // }
    // Serial1.println("Serial1 Initialized");
}

void Configure_Motors() {
    Left_Motor.attach(LEFT_MOTOR_PWM_PIN);
    Right_Motor.attach(RIGHT_MOTOR_PWM_PIN);
}

void Configure_Encoders() {
    if (Left_Encoder.init(0)) {
        // Serial1.println("Left Encoder Initialization OK");
    } else {
        // Serial1.println("Left Encoder Initialization Failed");
    }

    if (Right_Encoder.init(1)) {
        // Serial1.println("Right Encoder Initialization OK");
    } else {
        // Serial1.println("Right Encoder Initialization Failed");
    }
}

void Read_Encoders() {
    Left_Encoder.loop();
    Right_Encoder.loop();

    if (millis() - Encoder_Timer >= ENCODER_READ_DELAY) {
        Left_Encoder_Ticks = Left_Encoder.getTicks();
        Right_Encoder_Ticks = Right_Encoder.getTicks();
        // Serial1.print(Left_Encoder_Ticks);
        // Serial1.print('\t');
        // Serial1.println(Right_Encoder_Ticks);
        Encoder_Timer = millis();
    }
}


