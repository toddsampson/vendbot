#include <Arduino.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <AFMotor.h>

ros::NodeHandle  nh;
AF_DCMotor motor1(3);
AF_DCMotor motor2(4);
int turnSpeed = 125;
int moveSpeed = 175;
int runningFor = 0;
unsigned long lastActionTime = 0;

void messageCb(const geometry_msgs::Twist& msg)
{
  int movement;
  int turn;
  unsigned long thisActionTime = millis();
  if(runningFor == 0 || runningFor > 55){
    movement = msg.linear.x;
    turn = msg.angular.z;
    if(movement == 1 || movement == -1 || turn == 1 || turn == -1){
      digitalWrite(13, HIGH);
      lastActionTime = thisActionTime;
      runningFor = 1;
      if(movement == 1 || movement == -1){
        if(movement == 1){
          Serial.println("MOVING FORWARD"); 
          motor1.run(FORWARD);
          motor1.setSpeed(moveSpeed);
          motor2.run(FORWARD);
          motor2.setSpeed(moveSpeed);
        }
        if(movement == -1){
          Serial.println("MOVING BACKWARD"); 
          motor1.run(BACKWARD);
          motor1.setSpeed(moveSpeed);
          motor2.run(BACKWARD);
          motor2.setSpeed(moveSpeed);          
        }
      } else {
        if(turn == 1){
          Serial.println("TURN LEFT");
          motor1.run(BACKWARD);
          motor1.setSpeed(turnSpeed);
          motor2.run(FORWARD);
          motor2.setSpeed(turnSpeed);
        }
        if(turn == -1){
          Serial.println("TURN RIGHT");
          motor1.run(FORWARD);
          motor1.setSpeed(turnSpeed);
          motor2.run(BACKWARD);
          motor2.setSpeed(turnSpeed);
        }
      }
      delay(200);
      if(lastActionTime == thisActionTime){
        Serial.println("THIS ACTION RAN IT OUT");
        runningFor = 0;
      }
    }
  }
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", messageCb);

void setup(){
  Serial.begin(9600);
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
  motor1.setSpeed(turnSpeed);
  motor1.run(RELEASE);
  motor2.setSpeed(turnSpeed);
  motor2.run(RELEASE);
}

void loop(){
  if(runningFor == 0){
    motor1.run(RELEASE);
    motor2.run(RELEASE);
    digitalWrite(13, LOW);
  } else {
    runningFor++;
  }
  nh.spinOnce();
  delay(1);
}
