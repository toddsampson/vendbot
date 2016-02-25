#include <Arduino.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <AFMotor.h>

ros::NodeHandle  nh;
AF_DCMotor motorLeft(3); //left wheel
AF_DCMotor motorRight(1); //right wheel
float currX = 0.0;
float currZ = 0.0;
float goalX = 0.0;
float goalZ = 0.0;
int running = 0;
int turnSpeed = 125;
int moveSpeed = 195;
int leftHeading = 0; //1 forward, 2 backward
int rightHeading = 0; //1 forward, 2 backward
int forwardBlocked = 0; //0 unblocked, 1 blocked
unsigned long lastMssgTime = 0;
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
  goalX = msg.linear.x;
  goalZ = msg.angular.z;
  lastMssgTime = millis();
}

void moveForward()
{
  debug_msg.data = "MOVING FORWARD";
  Debug.publish(&debug_msg);
  running = 1;
  leftHeading = 1;
  rightHeading = 1;
  motorRight.run(FORWARD);
  motorRight.setSpeed(moveSpeed);
  motorLeft.run(FORWARD);
  motorLeft.setSpeed(moveSpeed);
}

void moveBackward()
{
  debug_msg.data = "MOVING BACKWARD";
  Debug.publish(&debug_msg);
  running = 1;
  leftHeading = 2;
  rightHeading = 2;
  motorLeft.run(BACKWARD);
  motorLeft.setSpeed(moveSpeed);
  motorRight.run(BACKWARD);
  motorRight.setSpeed(moveSpeed);
}

void turnLeft()
{
  debug_msg.data = "TURN LEFT";
  Debug.publish(&debug_msg);
  running = 1;
  leftHeading = 2;
  rightHeading = 1;
  motorLeft.run(BACKWARD);
  motorLeft.setSpeed(turnSpeed);
  motorRight.run(FORWARD);
  motorRight.setSpeed(turnSpeed);
}

void turnRight()
{
  debug_msg.data = "TURN RIGHT";
  Debug.publish(&debug_msg);
  running = 1;
  leftHeading = 1;
  rightHeading = 2;
  motorLeft.run(FORWARD);
  motorLeft.setSpeed(turnSpeed);
  motorRight.run(BACKWARD);
  motorRight.setSpeed(turnSpeed);
}

void stopMovement()
{
  debug_msg.data = "MOVEMENT STOPPED";
  Debug.publish(&debug_msg);
  motorLeft.run(RELEASE);
  motorRight.run(RELEASE);
  running = 0;
  leftHeading = 0;
  rightHeading = 0;
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
  static unsigned long encTimer = 0;
  nh.spinOnce();
  
  if(goalX != currX || goalZ != currZ){
    currX = goalX;  // later we will slowly ramp curr up towards goal
    currZ = goalZ;  // and use an accel method to determine speed to set
    if(currX > 0.1 || currX < -0.1 || currZ > 0.1 || currZ < -0.1){
      debug_msg.data = "NEW ACTION STARTING";
      Debug.publish(&debug_msg);
      if(currX > 0.1){
        if(forwardBlocked == 0){
          moveForward();
        }
      } else if(currX < -0.1){
        moveBackward();
      } else if(currZ > 0.1){
        turnLeft();
      } else if(currZ < -0.1){
        turnRight();
      }
    } else {
      stopMovement();
    }
  }

  if(running == 1 && (millis() - lastMssgTime > 250)){
    stopMovement();
  } 

  if(millis() - encTimer > 100){
    Serial.print("Coder value: ");
    Serial.print(coder[0]);
    Serial.print("[Left Wheel] ");
    Serial.print(coder[1]);
    Serial.println("[Right Wheel]");

    lastSpeed[0] = coder[0];   //record the latest speed value
    lastSpeed[1] = coder[1];
    coder[0] = 0;                 //clear the data buffer
    coder[1] = 0;
    encTimer = millis();
  }  
  delay(1);
}
