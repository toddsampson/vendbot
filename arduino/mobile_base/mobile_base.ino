#include <Arduino.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle  nh;

void messageCb(const geometry_msgs::Twist& msg)
{
  int forward;
  int turn;
  digitalWrite(13, HIGH-digitalRead(13));
  
  forward = msg.linear.x;
  turn = msg.angular.z;
  if(forward == 1){
    Serial.println("MOVING FORWARD");    
  } else {
    if(turn == 1){
      Serial.println("TURN LEFT");
    }
    if(turn == -1){
      Serial.println("TURN RIGHT");
    }
  }
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", messageCb);

void setup(){
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
}

void loop(){
  nh.spinOnce();
  delay(1);
}
