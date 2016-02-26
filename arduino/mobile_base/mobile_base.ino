#include <Arduino.h>
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Range.h>
#include <AFMotor.h>
#include <NewPing.h>
#include <SharpIR.h>

#define irl A6
#define irc A7
#define irr A8
#define model 20150
#define MAX_DISTANCE 200
#define SONAR_PERSONAL_SPACE 1000
#define IR_CENTER_PERSONAL_SPACE 35
#define IR_SIDE_PERSONAL_SPACE 25
#define LEFT digitalPinToInterrupt(20)
#define RIGHT digitalPinToInterrupt(21)

NewPing sonar_left(24, 24, MAX_DISTANCE);
NewPing sonar_right(25, 25, MAX_DISTANCE);
SharpIR ir_left(irl, 25, 93, model);
SharpIR ir_center(irc, 25, 93, model);
SharpIR ir_right(irr, 25, 93, model);
AF_DCMotor motorLeft(3); //left wheel
AF_DCMotor motorRight(1); //right wheel

float currX = 0.0;
float currZ = 0.0;
float goalX = 0.0;
float goalZ = 0.0;
boolean running = false;
int turnSpeed = 125;
int moveSpeed = 180;
int moveSpeedMax = 225;
int currSpeed = 0;
int leftHeading = 0; //1 forward, 2 backward
int rightHeading = 0; //1 forward, 2 backward
int forwardBlocked = 0; //0 unblocked, 1 blocked
unsigned long lastMssgTime = 0;
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
unsigned long range_timer;
char irl_frameid[] = "/ir_left";
char irc_frameid[] = "/ir_center";
char irr_frameid[] = "/ir_right";

ros::NodeHandle  nh;
std_msgs::String debug_msg;
ros::Publisher Debug ("debug_bot", &debug_msg);
geometry_msgs::Twist odom_msg;
ros::Publisher Pub ("ard_odom", &odom_msg);
geometry_msgs::Twist sensor_msg;
ros::Publisher Sensorpub ("sensor_debug", &sensor_msg);
sensor_msgs::Range irl_range_msg;
ros::Publisher irl_pub_range( "ir_left", &irl_range_msg);
sensor_msgs::Range irc_range_msg;
ros::Publisher irc_pub_range( "ir_center", &irc_range_msg);
sensor_msgs::Range irr_range_msg;
ros::Publisher irr_pub_range( "ir_right", &irr_range_msg);

void messageCb(const geometry_msgs::Twist& msg)
{
  goalX = msg.linear.x;
  goalZ = msg.angular.z;
  lastMssgTime = millis();
}

void nextSpeed(int maxSpeed)
{
  if(currSpeed < maxSpeed){
    currSpeed += 5;
    motorRight.setSpeed(currSpeed);
    motorLeft.setSpeed(currSpeed);
  }
}

void moveForward()
{
  debug_msg.data = "MOVING FORWARD";
  Debug.publish(&debug_msg);
  running = true;
  leftHeading = 1;
  rightHeading = 1;
  currSpeed = moveSpeed;
  motorRight.run(FORWARD);
  motorRight.setSpeed(moveSpeed);
  motorLeft.run(FORWARD);
  motorLeft.setSpeed(moveSpeed);
}

void moveBackward()
{
  debug_msg.data = "MOVING BACKWARD";
  Debug.publish(&debug_msg);
  running = true;
  leftHeading = 2;
  rightHeading = 2;
  currSpeed = moveSpeed;
  motorLeft.run(BACKWARD);
  motorLeft.setSpeed(moveSpeed);
  motorRight.run(BACKWARD);
  motorRight.setSpeed(moveSpeed);
}

void turnLeft()
{
  debug_msg.data = "TURN LEFT";
  Debug.publish(&debug_msg);
  running = true;
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
  running = true;
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
  running = false;
  currX = 0;
  currZ = 0;
  goalX = 0;
  goalZ = 0;
  currSpeed = 0;
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

boolean sonarBlocked(int val)
{
  if(val > 0 && val < SONAR_PERSONAL_SPACE){
    return true;
  }
  return false;
}

boolean irSideBlocked(int val)
{
  if(val > 0 && val < IR_SIDE_PERSONAL_SPACE){
    return true;
  }
  return false;
}

boolean irCenterBlocked(int val)
{
  if(val > 0 && val < IR_CENTER_PERSONAL_SPACE){
    return true;
  }
  return false;
}

boolean sensorBlocked(int sLeft, int sRight, int dLeft, int dCenter, int dRight)
{
  if(sonarBlocked(sLeft) || sonarBlocked(sRight) || irSideBlocked(dLeft) || irCenterBlocked(dCenter) || irSideBlocked(dRight)){
    return true;
  }
  return false;
}

void debugSensors(int dLeft, int dCenter, int dRight, int sLeft, int sRight)
{
  sensor_msg.linear.x = dLeft;
  sensor_msg.linear.y = dCenter;
  sensor_msg.linear.z = dRight;
  sensor_msg.angular.x = sLeft;
  sensor_msg.angular.y = forwardBlocked;
  sensor_msg.angular.z = sRight;
  Sensorpub.publish(&sensor_msg);
}

void checkSensors()
{
  unsigned int sLeft = sonar_left.ping();
  unsigned int sRight = sonar_right.ping();
  int dLeft=ir_left.distance(); 
  int dCenter=ir_center.distance();
  int dRight=ir_right.distance();

  if(sensorBlocked(sLeft, sRight, dLeft, dCenter, dRight)){
    forwardBlocked = 1;
    debug_msg.data = "BLOCKING FORWARD MOTION";
    Debug.publish(&debug_msg);
  } else {
    forwardBlocked = 0;
    debug_msg.data = "FORWARD MOTION UNBLOCKED";
    Debug.publish(&debug_msg); 
  }

  if(goalX > 0.1 && (sensorBlocked(sLeft, sRight, dLeft, dCenter, dRight))){
    goalX = 0;
    goalZ = 0;
    debug_msg.data = "SHOULD STOP FORWARD MOTION by setting goal velocities to 0";
    Debug.publish(&debug_msg); 
  }

  if ( (millis()-range_timer) > 50){
    irl_range_msg.range = dLeft;
    irl_range_msg.header.stamp = nh.now();
    irl_pub_range.publish(&irl_range_msg);
    irc_range_msg.range = dCenter;
    irc_range_msg.header.stamp = nh.now();
    irc_pub_range.publish(&irc_range_msg);   
    irr_range_msg.range = dRight;
    irr_range_msg.header.stamp = nh.now();
    irr_pub_range.publish(&irr_range_msg);    
    range_timer =  millis();
  }
  
  //debugSensors(dLeft, dCenter, dRight, sLeft, sRight);
}

void controlMotors()
{
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
//  if(movingForward() || movingBackward()) {
//    nextSpeed(moveSpeedMax);
//    debugSensors(currSpeed, 0, 0, 0, 0);
//  }
  if(running == true && (millis() - lastMssgTime > 250)){
    stopMovement();
  }  
}

boolean movingForward()
{
  if(leftHeading == 1 && rightHeading == 1){
    return true;
  }
  return false;
}

boolean movingBackward()
{
  if(leftHeading == 2 && rightHeading == 2){
    return true;
  }
  return false;
}

boolean turningLeft()
{
  if(leftHeading == 2 && rightHeading == 1){
    return true;
  }
  return false;
}

boolean turningRight()
{
  if(leftHeading == 1 && rightHeading == 2){
    return true;
  }
  return false;
}

void debugOdom(int vel_lx, int vel_az)
{
  sensor_msg.linear.x = vel_lx;
  sensor_msg.linear.y = lastSpeed[0];
  sensor_msg.linear.z = lastSpeed[1];
  sensor_msg.angular.x = leftHeading;
  sensor_msg.angular.y = rightHeading;
  sensor_msg.angular.z = vel_az;
  Sensorpub.publish(&sensor_msg);
}

void publishOdom(int vel_lx, int vel_az)
{
  odom_msg.linear.x = vel_lx;
  odom_msg.angular.z = vel_az;
  Pub.publish(&odom_msg);
}

void handleOdometry()
{
  lastSpeed[0] = coder[0];   //record the latest speed value
  lastSpeed[1] = coder[1];

  if(movingForward() || movingBackward()) {
    // forward or backwards
    vel_lx = ((((lastSpeed[0] + lastSpeed[1]) / 2.0) * 3.14 * wheelDiameter) / encoderTicks) * (10.0 / odomInterval); // 10 = 1000 second * .01 cm to m
    vel_az = 0;
  } else if(turningLeft()) {
    // left turn
    vel_lx = 0;
    vel_az = ((((((lastSpeed[0] - lastSpeed[1]) / 2.0) * 3.14 * wheelDiameter) / encoderTicks) / (wheelSeparation / 2.0)) * (1000.0 / odomInterval));
  } else if(turningRight()) {
    // right turn
    vel_lx = 0;
    vel_az = ((((((lastSpeed[0] - lastSpeed[1]) / 2.0) * 3.14 * wheelDiameter) / encoderTicks) / (wheelSeparation / 2.0)) * (1000.0 / odomInterval));
  } else {
    vel_lx = 0;
    vel_az = 0;
  }

  //debugOdom(vel_lx, vel_az);
  publishOdom(vel_lx, vel_az);
  coder[0] = 0;   //clear the data buffer
  coder[1] = 0;
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", messageCb);

void setupSensorMsgs()
{
  irl_range_msg.radiation_type = sensor_msgs::Range::INFRARED;
  irl_range_msg.header.frame_id =  irl_frameid;
  irl_range_msg.field_of_view = 1;
  irl_range_msg.min_range = 0.03;
  irl_range_msg.max_range = 0.4;
  irc_range_msg.radiation_type = sensor_msgs::Range::INFRARED;
  irc_range_msg.header.frame_id =  irc_frameid;
  irc_range_msg.field_of_view = 1;
  irc_range_msg.min_range = 0.03;
  irc_range_msg.max_range = 0.4;
  irr_range_msg.radiation_type = sensor_msgs::Range::INFRARED;
  irr_range_msg.header.frame_id =  irr_frameid;
  irr_range_msg.field_of_view = 1;
  irr_range_msg.min_range = 0.03;
  irr_range_msg.max_range = 0.4; 
}

void setupRosTopics()
{
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(Debug);
  nh.advertise(Pub);
  nh.advertise(Sensorpub);
  nh.advertise(irl_pub_range);
  nh.advertise(irc_pub_range);
  nh.advertise(irr_pub_range);  
}

void setup(){
  Serial.begin(57600);
  setupRosTopics();
  pinMode (irl, INPUT);
  //pinMode (irc, INPUT);
  pinMode (irr, INPUT);
  attachInterrupt(LEFT, LwheelSpeed, CHANGE);
  attachInterrupt(RIGHT, RwheelSpeed, CHANGE);
  motorLeft.setSpeed(turnSpeed);
  motorLeft.run(RELEASE);
  motorRight.setSpeed(turnSpeed);
  motorRight.run(RELEASE);
  setupSensorMsgs();
}

void loop(){
  static unsigned long encTimer = 0;
 
  nh.spinOnce();

  checkSensors();
  
  controlMotors();
  
  if(millis() - encTimer > odomInterval){
    handleOdometry();
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

