#include <Arduino.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <AFMotor.h>

ros::NodeHandle  nh;
AF_DCMotor motor1(3);
AF_DCMotor motor2(4);

void messageCb(const geometry_msgs::Twist& msg)
{
  int forward;
  int turn;
  digitalWrite(13, HIGH-digitalRead(13));
  
  forward = msg.linear.x;
  turn = msg.angular.z;
  if(forward == 1){
    Serial.println("MOVING FORWARD"); 
    motor1.run(FORWARD);
    motor2.run(FORWARD);
    delay(10);
  } else {
    if(turn == 1){
      Serial.println("TURN LEFT");
      motor1.run(FORWARD);
      motor2.run(BACKWARD);
      delay(10);
    }
    if(turn == -1){
      Serial.println("TURN RIGHT");
      motor1.run(BACKWARD);
      motor2.run(FORWARD);
      delay(10);
    }
  }
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", messageCb);

void setup(){
  Serial.begin(9600);
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
  motor1.setSpeed(200);
  motor1.run(RELEASE);
  motor2.setSpeed(200);
  motor2.run(RELEASE);
}

void loop(){
  nh.spinOnce();
  delay(1);
}
