#include <Arduino.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <AFMotor.h>

ros::NodeHandle  nh;
AF_DCMotor motorLeft(3); //left wheel
AF_DCMotor motorRight(1); //right wheel
int turnSpeed = 125;
int moveSpeed = 195;
int runningFor = 0;
int leftHeading = 0; //1 forward, 2 backward
int rightHeading = 0; //1 forward, 2 backward
unsigned long lastActionTime = 0;
std_msgs::String debug_msg;
ros::Publisher Debug ("debug_bot", &debug_msg);
#define LEFT digitalPinToInterrupt(20)
#define RIGHT digitalPinToInterrupt(21)

long coder[2] = {
  0,0};
int lastSpeed[2] = {
  0,0};  

void messageCb(const geometry_msgs::Twist& msg)
{
  float movement;
  float turn;
  String testString;
  unsigned long thisActionTime;
  if(runningFor == 0){
    movement = msg.linear.x;
    turn = msg.angular.z;
    if(movement > 0.1 || movement < -0.1 || turn > 0.1 || turn < -0.1){
      thisActionTime = millis();
      lastActionTime = thisActionTime;
      debug_msg.data = "NEW ACTION STARTING";
      Debug.publish(&debug_msg);
      runningFor = 1;
      if(movement > 0.1 || movement < -0.1){
        if(movement > 0.1){
          debug_msg.data = "MOVING FORWARD";
          Debug.publish(&debug_msg);
          leftHeading = 1;
          rightHeading = 1;
          motorRight.run(FORWARD);
          motorRight.setSpeed(moveSpeed);
          motorLeft.run(FORWARD);
          motorLeft.setSpeed(moveSpeed);
        }
        if(movement < -0.1){
          debug_msg.data = "MOVING BACKWARD";
          Debug.publish(&debug_msg);
          leftHeading = 2;
          rightHeading = 2;
          motorLeft.run(BACKWARD);
          motorLeft.setSpeed(moveSpeed);
          motorRight.run(BACKWARD);
          motorRight.setSpeed(moveSpeed);          
        }
      } else {
        if(turn > 0.1){
          debug_msg.data = "TURN LEFT";
          Debug.publish(&debug_msg);
          leftHeading = 2;
          rightHeading = 1;
          motorLeft.run(BACKWARD);
          motorLeft.setSpeed(turnSpeed);
          motorRight.run(FORWARD);
          motorRight.setSpeed(turnSpeed);
        }
        if(turn < -0.1){
          debug_msg.data = "TURN RIGHT";
          Debug.publish(&debug_msg);
          leftHeading = 1;
          rightHeading = 2;
          motorLeft.run(FORWARD);
          motorLeft.setSpeed(turnSpeed);
          motorRight.run(BACKWARD);
          motorRight.setSpeed(turnSpeed);
        }
      }
      delay(200);
      debug_msg.data = "AFTER DELAY";
      Debug.publish(&debug_msg); 
      debug_msg.data = "last action time";
      //debug_msg.data += lastActionTime;
      Debug.publish(&debug_msg);
      debug_msg.data = "this action time";
      //debug_msg.data += thisActionTime;
      Debug.publish(&debug_msg);
      if(lastActionTime == thisActionTime){
        debug_msg.data = "THIS ACTION RAN IT OUT";
        Debug.publish(&debug_msg);
      }
      motorLeft.run(RELEASE);
      motorRight.run(RELEASE);
      runningFor = 0;
      leftHeading = 0;
      rightHeading = 0;
    }
  } else {
    if(runningFor == 1){
      debug_msg.data = "RUNNING == 1 - MESSAGE RECEIVED AND SKIPPED BECAUSE WE ARE ALREADY RUNNING";
      Debug.publish(&debug_msg);
    } else {
      debug_msg.data = "RUNNING != 1 - MESSAGE RECEIVED AND SKIPPED BECAUSE WE ARE ALREADY RUNNING";
      Debug.publish(&debug_msg);           
    }
  }
}

void LwheelSpeed()
{
  Serial.println(lastSpeed[0]);
  if(leftHeading == 1){
        debug_msg.data = "recording left wheel forward action";
        Debug.publish(&debug_msg);  
    coder[0] ++;
  } else {
    if(leftHeading == 2){
        debug_msg.data = "recording left wheel backward action";
        Debug.publish(&debug_msg);  
      coder[0] --;  //count the left wheel encoder interrupts
    }
  }
}

void RwheelSpeed()
{
  Serial.println(lastSpeed[1]);
  if(rightHeading == 1){
        debug_msg.data = "recording right wheel forward action";
        Debug.publish(&debug_msg);  
    coder[1] ++;
  } else {
    if(rightHeading == 2){
        debug_msg.data = "recording right wheel backward action";
        Debug.publish(&debug_msg);  
      coder[1] --;  //count the left wheel encoder interrupts
    }
  }
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", messageCb);

void setup(){
  Serial.begin(57600);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(Debug);
  attachInterrupt(LEFT, LwheelSpeed, CHANGE);
  attachInterrupt(RIGHT, RwheelSpeed, CHANGE);
  motorLeft.setSpeed(turnSpeed);
  motorLeft.run(RELEASE);
  motorRight.setSpeed(turnSpeed);
  motorRight.run(RELEASE);
}

void loop(){
  static unsigned long timer = 0;
  if(runningFor == 0){

  } else {
    runningFor++;
  }
  nh.spinOnce();

  if(millis() - timer > 100){
    Serial.print("Coder value: ");
    Serial.print(coder[0]);
    Serial.print("[Left Wheel] ");
    Serial.print(coder[1]);
    Serial.println("[Right Wheel]");

    lastSpeed[0] = coder[0];   //record the latest speed value
    lastSpeed[1] = coder[1];
    coder[0] = 0;                 //clear the data buffer
    coder[1] = 0;
    timer = millis();
  }  
  delay(1);
}
