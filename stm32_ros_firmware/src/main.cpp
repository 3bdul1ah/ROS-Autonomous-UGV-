#include "UGV_Config.h"
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point32.h>

HardwareSerial Serial1(PA3, PA2);

float UGV_Linear_Velocity = 0.0;
float UGV_Angular_Velocity = 0.0;
float UGV_Right_Wheel_Velocity, UGV_Left_Wheel_Velocity;

const int UGV_MOTOR_STOP = 1500;
const int UGV_MOTOR_MAX_FORWARD = 2200;
const int UGV_MOTOR_MAX_BACKWARD = 800;

ros::NodeHandle UGV_ROS_Serial;

geometry_msgs::Point32 UGV_Encoder_Data_Message;
ros::Publisher UGV_Encoder_Data_Publisher("/encoder", &UGV_Encoder_Data_Message);

void UGV_cmd_vel_Callback(const geometry_msgs::Twist& msg) {
  UGV_Linear_Velocity = msg.linear.x;
  UGV_Angular_Velocity = msg.angular.z;

  UGV_Left_Wheel_Velocity  = (2 * UGV_Linear_Velocity - UGV_Angular_Velocity * UGV_WHEEL_BASE) /(2 * UGV_WHEEL_RADIUS);
  UGV_Right_Wheel_Velocity = (2 * UGV_Linear_Velocity + UGV_Angular_Velocity * UGV_WHEEL_BASE) /(2 * UGV_WHEEL_RADIUS);

  if (UGV_Left_Wheel_Velocity < 0) {
    UGV_Left_Motor.writeMicroseconds(map(abs(UGV_Left_Wheel_Velocity), 0, UGV_MAX_VELOCITY, UGV_MOTOR_MAX_BACKWARD, UGV_MOTOR_STOP));
  }
  else if (UGV_Left_Wheel_Velocity > 0) {
    UGV_Left_Motor.writeMicroseconds(map(UGV_Left_Wheel_Velocity, 0, UGV_MAX_VELOCITY, UGV_MOTOR_MAX_FORWARD, UGV_MOTOR_STOP));
  }
  else {
    UGV_Left_Motor.writeMicroseconds(UGV_MOTOR_STOP);
  }

  if (UGV_Right_Wheel_Velocity < 0) {
    UGV_Right_Motor.writeMicroseconds(map(abs(UGV_Right_Wheel_Velocity), 0, UGV_MAX_VELOCITY, UGV_MOTOR_MAX_BACKWARD, UGV_MOTOR_STOP));
  }
  else if (UGV_Right_Wheel_Velocity > 0) {
    UGV_Right_Motor.writeMicroseconds(map(UGV_Right_Wheel_Velocity, 0, UGV_MAX_VELOCITY, UGV_MOTOR_MAX_FORWARD, UGV_MOTOR_STOP));
  }
  else {
    UGV_Right_Motor.writeMicroseconds(UGV_MOTOR_STOP);
  }
}

ros::Subscriber<geometry_msgs::Twist> UGV_cmd_Vel_Subscriber("/cmd_vel", &UGV_cmd_vel_Callback);

void setup() {
  UGV_initialize_Serial();
  UGV_Configure_Motors();
  UGV_Configure_Encoders();

  UGV_ROS_Serial.initNode();
  UGV_ROS_Serial.getHardware()->setBaud(UGV_BAUD_RATE);

  UGV_ROS_Serial.advertise(UGV_Encoder_Data_Publisher);
  UGV_ROS_Serial.subscribe(UGV_cmd_Vel_Subscriber);
}

void loop() {
  UGV_Read_Encoders();
  UGV_Encoder_Data_Message.x = UGV_Left_Encoder_Ticks;
  UGV_Encoder_Data_Message.y = UGV_Right_Encoder_Ticks;

  UGV_Encoder_Data_Publisher.publish(&UGV_Encoder_Data_Message);

  UGV_ROS_Serial.spinOnce();
}
