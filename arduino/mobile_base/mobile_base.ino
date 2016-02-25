#include <Arduino.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <AFMotor.h>
#include <NewPing.h>
#include <SharpIR.h>


#define irl A6
#define irc A7
#define irr A8
#define model 20150
#define MAX_DISTANCE 200

NewPing sonar_left(24, 24, MAX_DISTANCE);
NewPing sonar_right(25, 25, MAX_DISTANCE);

SharpIR ir_left(irl, 25, 93, model);
SharpIR ir_center(irc, 25, 93, model);
SharpIR ir_right(irr, 25, 93, model);

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
geometry_msgs::Twist odom_msg;
ros::Publisher Pub ("ard_odom", &odom_msg);
geometry_msgs::Twist sensor_msg;
ros::Publisher Sensorpub ("sensor_debug", &sensor_msg);
#define LEFT digitalPinToInterrupt(20)
#define RIGHT digitalPinToInterrupt(21)
int odomInterval = 100;
float wheelDiameter = 6.56; // In cm
int wheelSeparation = 26; // In cm
int encoderTicks = 20; // Per rotation
double vel_lx = 0; // odom linear x velocity
double vel_az = 0; // odom angular z velocity

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
  currX = 0;
  currZ = 0;
  goalX = 0;
  goalZ = 0;
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
  nh.advertise(Pub);
  nh.advertise(Sensorpub);
  pinMode (irl, INPUT);
  pinMode (irc, INPUT);
  pinMode (irr, INPUT);
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
  unsigned int sLeft = sonar_left.ping();
  unsigned int sRight = sonar_right.ping();
  int dLeft=ir_left.distance();
  int dCenter=ir_center.distance();
  int dRight=ir_right.distance();

    sensor_msg.linear.x = dLeft;
    sensor_msg.linear.y = dCenter;
    sensor_msg.linear.z = dRight;
    sensor_msg.angular.x = sLeft;
    sensor_msg.angular.z = sRight;
    Sensorpub.publish(&sensor_msg);
  
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

  if(millis() - encTimer > odomInterval){
    lastSpeed[0] = coder[0];   //record the latest speed value
    lastSpeed[1] = coder[1];

    if((leftHeading == 1 && rightHeading == 1) || (leftHeading == 2 && rightHeading == 2)) {
      // forward or backwards
      vel_lx = ((((lastSpeed[0] + lastSpeed[1]) / 2.0) * 3.14 * wheelDiameter) / encoderTicks) * (10.0 / odomInterval); // 10 = 1000 second * .01 cm to m
      vel_az = 0;
    } else if(leftHeading == 2 && rightHeading == 1) {
      // left turn
      vel_lx = 0;
      vel_az = ((((((lastSpeed[0] - lastSpeed[1]) / 2.0) * 3.14 * wheelDiameter) / encoderTicks) / (wheelSeparation / 2.0)) * (1000.0 / odomInterval));
    } else if(leftHeading == 1 && rightHeading == 2) {
      // right turn
      vel_lx = 0;
      vel_az = ((((((lastSpeed[0] - lastSpeed[1]) / 2.0) * 3.14 * wheelDiameter) / encoderTicks) / (wheelSeparation / 2.0)) * (1000.0 / odomInterval));
    } else {
      vel_lx = 0;
      vel_az = 0;
    }
    
    Serial.print("Odom Linear X: ");
    Serial.println(vel_lx);
    Serial.print("Odom Angular Z: ");
    Serial.println(vel_az);

    odom_msg.linear.x = vel_lx;
    odom_msg.angular.z = vel_az;
    Pub.publish(&odom_msg);

    coder[0] = 0;                 //clear the data buffer
    coder[1] = 0;
    encTimer = millis();
  }
  delay(1);
}

// TODO: Move to constants instead of vars where possible
// TODO: Serious refactor
// TODO: Pre-compute values so they don't need to be recomputed every time
// TODO: Think about moving forward and backwards from 0, 1, 2 to 0, 1, -1
// TODO: Switch time from odomInterval to actual time passed for velocity calculations
// TODO: Don't report odom when any values are out of range

