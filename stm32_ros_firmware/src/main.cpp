#include "UGV_Config.h"
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point32.h>

HardwareSerial Serial1(PA3, PA2);


float Linear_Velocity = 0.0;
float Angular_Velocity = 0.0;
float Right_Wheel_Velocity, Left_Wheel_Velocity;

const int MOTOR_STOP = 1500;
const int MOTOR_MAX_FORWARD = 2200;
const int MOTOR_MAX_BACKWARD = 800;

ros::NodeHandle ROS_Serial;

geometry_msgs::Point32 Encoder_Data_Message;
ros::Publisher Encoder_Data_Publisher("/encoder", &Encoder_Data_Message);

void cmd_vel_Callback(const geometry_msgs::Twist& msg){
  Linear_Velocity = msg.linear.x;
  Angular_Velocity = msg.angular.z;

  Left_Wheel_Velocity  = (2 * Linear_Velocity - Angular_Velocity * WHEEL_BASE) /(2 * WHEEL_RADIUS);
  Right_Wheel_Velocity = (2 * Linear_Velocity + Angular_Velocity * WHEEL_BASE) /(2 * WHEEL_RADIUS);

  if (Left_Wheel_Velocity < 0) {
    Left_Motor.writeMicroseconds(map(abs(Left_Wheel_Velocity), 0, MAX_VELOCITY, MOTOR_MAX_BACKWARD, MOTOR_STOP));
  }

  else if (Left_Wheel_Velocity > 0){
    Left_Motor.writeMicroseconds(map(Left_Wheel_Velocity, 0, MAX_VELOCITY, MOTOR_MAX_FORWARD, MOTOR_STOP));
  }

  else {
    Left_Motor.writeMicroseconds(MOTOR_STOP);
  }

  if (Right_Wheel_Velocity < 0) {
    Right_Motor.writeMicroseconds(map(abs(Right_Wheel_Velocity), 0, MAX_VELOCITY, MOTOR_MAX_BACKWARD, MOTOR_STOP));
  }

  else if (Right_Wheel_Velocity > 0){
    Right_Motor.writeMicroseconds(map(Right_Wheel_Velocity, 0, MAX_VELOCITY, MOTOR_MAX_FORWARD, MOTOR_STOP));
  }

  else {
    Right_Motor.writeMicroseconds(MOTOR_STOP);
  }

}

ros::Subscriber<geometry_msgs::Twist> cmd_Vel_Subscriber("/cmd_vel", &cmd_vel_Callback);

void setup() {
  initialize_Serial();
  Configure_Motors();
  Configure_Encoders();

  ROS_Serial.initNode();
  ROS_Serial.getHardware()->setBaud(BAUD_RATE);

  ROS_Serial.advertise(Encoder_Data_Publisher);
  ROS_Serial.subscribe(cmd_Vel_Subscriber);

}

void loop() {

  Read_Encoders();
  Encoder_Data_Message.x = Left_Encoder_Ticks;
  Encoder_Data_Message.y = Right_Encoder_Ticks;

  Encoder_Data_Publisher.publish(&Encoder_Data_Message);

  ROS_Serial.spinOnce();
}
