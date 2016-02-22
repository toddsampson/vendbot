#include <Arduino.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle  nh;

void messageCb(const geometry_msgs::Twist& msg)
{
  digitalWrite(13, HIGH-digitalRead(13));
  Serial.println("hi!----------------------");
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", messageCb);

void setup(){
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
}

void loop(){
  Serial.println("loop");
  nh.spinOnce();
  delay(200);
}
