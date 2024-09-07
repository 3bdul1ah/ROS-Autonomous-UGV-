#include "UGV_Config.h"

Servo UGV_Left_Motor;
Servo UGV_Right_Motor;

Encoder UGV_Left_Encoder(UGV_TIM2_CH1_PIN, UGV_TIM2_CH2_PIN, SINGLE, 0);
Encoder UGV_Right_Encoder(UGV_TIM4_CH1_PIN, UGV_TIM4_CH2_PIN, SINGLE, 0);

unsigned long UGV_Encoder_Timer = 0;

long UGV_Left_Encoder_Ticks = 0;
long UGV_Right_Encoder_Ticks = 0;

void UGV_initialize_Serial() {
    Serial1.begin(UGV_BAUD_RATE);
    // while (!Serial1) {
    // }
    // Serial1.println("Serial1 Initialized");
}

void UGV_Configure_Motors() {
    UGV_Left_Motor.attach(UGV_LEFT_MOTOR_PWM_PIN);
    UGV_Right_Motor.attach(UGV_RIGHT_MOTOR_PWM_PIN);
}

void UGV_Configure_Encoders() {
    if (UGV_Left_Encoder.init(0)) {
        // Serial1.println("Left Encoder Initialization OK");
    } else {
        // Serial1.println("Left Encoder Initialization Failed");
    }

    if (UGV_Right_Encoder.init(1)) {
        // Serial1.println("Right Encoder Initialization OK");
    } else {
        // Serial1.println("Right Encoder Initialization Failed");
    }
}

void UGV_Read_Encoders() {
    UGV_Left_Encoder.loop();
    UGV_Right_Encoder.loop();

    if (millis() - UGV_Encoder_Timer >= UGV_ENCODER_READ_DELAY) {
        UGV_Left_Encoder_Ticks = UGV_Left_Encoder.getTicks();
        UGV_Right_Encoder_Ticks = UGV_Right_Encoder.getTicks();
        // Serial1.print(UGV_Left_Encoder_Ticks);
        // Serial1.print('\t');
        // Serial1.println(UGV_Right_Encoder_Ticks);
        UGV_Encoder_Timer = millis();
    }
}
