#include <Arduino.h>
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Range.h>
#include <tf/transform_broadcaster.h>
//#include <nav_msgs/Odometry.h>  
#include <AFMotor.h>
#include <NewPing.h>
#include <SharpIR.h>

#define irl A6
#define irc A7
#define irr A8
#define model 20150
#define MAX_DISTANCE 200
#define SONAR_PERSONAL_SPACE 1050
#define IR_CENTER_PERSONAL_SPACE 40
#define IR_SIDE_PERSONAL_SPACE 30
#define odomInterval 100
#define sensInterval 125
#define turnSpeedMin 145
#define turnSpeedMax 180
#define moveSpeedMin 175
#define moveSpeedMax 225
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
int currSpeed = 0;
int leftHeading = 0; //1 forward, 2 backward
int rightHeading = 0; //1 forward, 2 backward
int forwardBlocked = 0; //0 unblocked, 1 blocked
unsigned long lastMssgTime = 0;
float wheelDiameter = 6.56; // In cm
int wheelSeparation = 26; // In cm
int encoderTicks = 20; // Per rotation

long coder[2] = {
  0,0};
int lastSpeed[2] = {
  0,0};


ros::NodeHandle  nh;
std_msgs::String debug_msg;
ros::Publisher Debug ("debug_bot", &debug_msg);
//nav_msgs::Odometry odom_msg;
geometry_msgs::Twist odom_msg;
ros::Publisher Pub ("ard_odom", &odom_msg);
geometry_msgs::Twist sensor_msg;
ros::Publisher Sensorpub ("sensor_debug", &sensor_msg);
sensor_msgs::Range ir_range_msg;
ros::Publisher irl_pub( "ir_left_depth_frame", &ir_range_msg);
ros::Publisher irc_pub( "ir_center_depth_frame", &ir_range_msg);
ros::Publisher irr_pub( "ir_right_depth_frame", &ir_range_msg);
sensor_msgs::Range sonar_range_msg;
ros::Publisher sl_pub( "sonar_left_depth_frame", &sonar_range_msg);
ros::Publisher sr_pub( "sonar_right_depth_frame", &sonar_range_msg);
geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;
char base_link[] = "/base_link";
char odom[] = "/odom";

void messageCb(const geometry_msgs::Twist& msg)
{
  goalX = msg.linear.x;
  goalZ = msg.angular.z;
  lastMssgTime = millis();
  debug_msg.data = "RUNNING MSSG CALLBACK";
  Debug.publish(&debug_msg);
}

int nextSpeed(int minSpeed, int maxSpeed)
{
  if(currSpeed == 0){
    return minSpeed;
  } else if(currSpeed < maxSpeed){
    return currSpeed + 1;
  }
  return currSpeed;
}

int speedBump()
{
  int currLeftCnt = abs(coder[0]);
  int currRightCnt = abs(coder[1]);
  if(currRightCnt == 0){
    if(currLeftCnt > 1){
      return -50;
    } else if (currLeftCnt == 1) {
      return -40;
    }
  }
  if(currLeftCnt == 0){
    if(currRightCnt > 1){
      return 50;
    } else if (currRightCnt == 1) {
      return 40;
    }
  }
  float encCntRatio = currLeftCnt / currRightCnt;
  if(encCntRatio < 0.99){
    if(encCntRatio < 0.5){
      return 50;
    }
    return 40;
  } else if(encCntRatio > 1.01){
    if(encCntRatio > 1.5){
      return -50;
    }
    return -40;
  }
  return 0;
}

int getLeftSpeed(int correction)
{
  int newSpeed = currSpeed + correction;
  if(newSpeed > moveSpeedMax){
    return moveSpeedMax;
  } else if(newSpeed < moveSpeedMin){
    return moveSpeedMin;
  }
  return newSpeed;
}

int getRightSpeed(int correction)
{
  int newSpeed = currSpeed - correction;
  if(newSpeed > moveSpeedMax){
    return moveSpeedMax;
  } else if(newSpeed < moveSpeedMin){
    return moveSpeedMin;
  }
  return newSpeed;
}

void moveForward()
{
  debug_msg.data = "MOVING FORWARD";
  Debug.publish(&debug_msg);
  running = true;
  leftHeading = 1;
  rightHeading = 1;
  currSpeed = nextSpeed(moveSpeedMin, moveSpeedMax);
  int correction = speedBump();
  int leftSpeed = getLeftSpeed(correction);
  int rightSpeed = getRightSpeed(correction);
  motorLeft.run(FORWARD);
  motorLeft.setSpeed(leftSpeed);
  motorRight.run(FORWARD);
  motorRight.setSpeed(rightSpeed);  
}

void moveBackward()
{
  debug_msg.data = "MOVING BACKWARD";
  Debug.publish(&debug_msg);
  running = true;
  leftHeading = 2;
  rightHeading = 2;
  currSpeed = nextSpeed(moveSpeedMin, moveSpeedMax);
  int correction = speedBump();
  int leftSpeed = getLeftSpeed(correction);
  int rightSpeed = getRightSpeed(correction);
  //debugSensors(currSpeed, correction, 0, leftSpeed, rightSpeed);
  motorLeft.run(BACKWARD);
  motorLeft.setSpeed(currSpeed);
  motorRight.run(BACKWARD);
  motorRight.setSpeed(currSpeed);
}

void turnLeft()
{
  debug_msg.data = "TURN LEFT";
  Debug.publish(&debug_msg);
  running = true;
  leftHeading = 2;
  rightHeading = 1;
  currSpeed = nextSpeed(turnSpeedMin, turnSpeedMax);
  motorLeft.run(BACKWARD);
  motorLeft.setSpeed(currSpeed);
  motorRight.run(FORWARD);
  motorRight.setSpeed(currSpeed);
}

void turnRight()
{
  debug_msg.data = "TURN RIGHT";
  Debug.publish(&debug_msg);
  running = true;
  leftHeading = 1;
  rightHeading = 2;
  currSpeed = nextSpeed(turnSpeedMin, turnSpeedMax);
  motorLeft.run(FORWARD);
  motorLeft.setSpeed(currSpeed);
  motorRight.run(BACKWARD);
  motorRight.setSpeed(currSpeed);
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

void publishIR(float dLeft, float dCenter, float dRight)
{
  char irl_frameid[] = "/ir_left_depth_frame";
  char irc_frameid[] = "/ir_center_depth_frame";
  char irr_frameid[] = "/ir_right_depth_frame";

  ir_range_msg.header.frame_id =  irl_frameid;   
  ir_range_msg.range = dLeft / 100;
  ir_range_msg.header.stamp = nh.now();
  irl_pub.publish(&ir_range_msg);

  ir_range_msg.header.frame_id =  irc_frameid;
  ir_range_msg.range = dCenter / 100;
  ir_range_msg.header.stamp = nh.now();
  irc_pub.publish(&ir_range_msg);

  ir_range_msg.header.frame_id =  irr_frameid;
  ir_range_msg.range = dRight / 100;
  ir_range_msg.header.stamp = nh.now();
  irr_pub.publish(&ir_range_msg);  
}

void publishSonar(float sLeft, float sRight)
{
  char sl_frameid[] = "/sonar_left_depth_frame";
  char sr_frameid[] = "/sonar_right_depth_frame";

  sLeft = (sLeft / US_ROUNDTRIP_CM) / 100;
  sonar_range_msg.header.frame_id =  sl_frameid;
  sonar_range_msg.range = sLeft;
  sonar_range_msg.header.stamp = nh.now();
  sl_pub.publish(&sonar_range_msg);
    
  sRight = (sRight / US_ROUNDTRIP_CM) / 100;
  sonar_range_msg.header.frame_id =  sr_frameid;
  sonar_range_msg.range = sRight;
  sonar_range_msg.header.stamp = nh.now();
  sr_pub.publish(&sonar_range_msg);
}

void checkSensors()
{
  float sLeft = sonar_left.ping();
  float sRight = sonar_right.ping();
  float dLeft=ir_left.distance(); 
  float dCenter=ir_center.distance();
  float dRight=ir_right.distance();

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

  publishIR(dLeft, dCenter, dRight);

  publishSonar(sLeft, sRight);
  
  //debugSensors(dLeft, dCenter, dRight, sLeft, sRight);
}

void controlMotors()
{
  if(goalX != currX || goalZ != currZ || goalX > 0.1 || goalX < -0.1 || goalZ > 0.1 || goalZ < -0.1){
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
  if(running == true && (millis() - lastMssgTime > 300)){
    debug_msg.data = "STOPPING MOVEMENT DUE TO LASTMSSGTIME TIMEOUT";
    Debug.publish(&debug_msg);    
    stopMovement();
  }  
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
  //odom_msg.header.stamp = nh.now();
  //odom_msg.header.frame_id = "odom";
  //odom_msg.child_frame_id = "base_link";
  //odom_msg.twist.twist.linear.x = vel_lx;
  //odom_msg.twist.twist.angular.z = vel_az;
  Pub.publish(&odom_msg);

  t.header.frame_id = odom;
  t.child_frame_id = base_link;
  t.transform.translation.x = vel_lx; 
  t.transform.rotation.x = 0.0;
  t.transform.rotation.y = 0.0; 
  t.transform.rotation.z = vel_az; 
  t.transform.rotation.w = 0.0;  
  t.header.stamp = nh.now();
  broadcaster.sendTransform(t);  
}

void handleOdometry()
{
  double vel_lx = 0; // odom linear x velocity
  double vel_az = 0; // odom angular z velocity  
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

  debugOdom(vel_lx, vel_az);
  publishOdom(vel_lx, vel_az);
  coder[0] = 0;   //clear the data buffer
  coder[1] = 0;
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", messageCb);

void setupSensorMsgs()
{
  ir_range_msg.radiation_type = sensor_msgs::Range::INFRARED;
  ir_range_msg.field_of_view = 0.01;
  ir_range_msg.min_range = 0.1;
  ir_range_msg.max_range = 0.8;

  sonar_range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  sonar_range_msg.field_of_view = 0.7;
  sonar_range_msg.min_range = 0.02;
  sonar_range_msg.max_range = 3; 
}

void setupRosTopics()
{
  nh.initNode();
  broadcaster.init(nh);
  nh.subscribe(sub);
  nh.advertise(Debug);
  nh.advertise(Pub);
  nh.advertise(Sensorpub);
  nh.advertise(irl_pub);
  nh.advertise(irc_pub);
  nh.advertise(irr_pub);
  nh.advertise(sl_pub);
  nh.advertise(sr_pub);  
}

void setup(){
  Serial.begin(57600);
  setupRosTopics();
  pinMode (irl, INPUT);
  pinMode (irc, INPUT);
  pinMode (irr, INPUT);
  attachInterrupt(LEFT, LwheelSpeed, CHANGE);
  attachInterrupt(RIGHT, RwheelSpeed, CHANGE);
  motorLeft.setSpeed(turnSpeedMin);
  motorLeft.run(RELEASE);
  motorRight.setSpeed(turnSpeedMin);
  motorRight.run(RELEASE);
  setupSensorMsgs();
}

void loop(){
  static unsigned long encTimer = 0;
  static unsigned long sensTimer = 0;
 
  nh.spinOnce();

  if(millis() - sensTimer > sensInterval){
    checkSensors();
    sensTimer = millis();
  }
  
  controlMotors();
  
  if(millis() - encTimer > odomInterval){
    handleOdometry();
    encTimer = millis();
  }
  
  delay(1);
}

// TODO: Make debug statements use a function
// TODO: Move vars local to the functions where possible
// TODO: Move to constants instead of vars where possible
// TODO: Serious refactor
// TODO: Pre-compute values so they don't need to be recomputed every time
// TODO: Think about moving forward and backwards from 0, 1, 2 to 0, 1, -1
// TODO: Switch time from odomInterval to actual time passed for velocity calculations
// TODO: Don't report odom when any values are out of range

