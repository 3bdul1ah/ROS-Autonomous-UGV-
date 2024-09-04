#include "UGV_Config.h"
HardwareSerial Serial1(PA3, PA2);


void setup() {
    initialize_Serial();    
    Configure_Motors();     
    Configure_Encoders();     
  
}

void loop() {
    Read_Right_Encoder();
    Right_Motor.writeMicroseconds(1500);
    Serial1.println(Right_Encoder_Ticks);

    Serial1.println('\n');

    Read_Left_Encoder();
    Left_Motor.writeMicroseconds(1500);
    Serial1.println(Left_Encoder_Ticks);
}
