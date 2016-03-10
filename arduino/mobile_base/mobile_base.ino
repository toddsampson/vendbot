#include <Arduino.h>
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Range.h>
//#include <tf/transform_broadcaster.h>
//#include <nav_msgs/Odometry.h>
#include <AFMotor.h>
#include <NewPing.h>
#include <SharpIR.h>

#define irl A6
#define irc A7
#define irr A8
#define model 20150
#define MAX_DISTANCE 200
#define SONAR_PERSONAL_SPACE 1250
#define IR_CENTER_PERSONAL_SPACE 50
#define IR_SIDE_PERSONAL_SPACE 40
#define MOTOR_INTERVAL 50
#define MOVEMENT_TIMEOUT 200
#define turnSpeedMin 145
#define turnSpeedMax 180
#define moveSpeedMin 140
#define moveSpeedStart 165
#define moveSpeedMax 245
#define moveSpeedMaxL 255
#define moveSpeedMaxR 220
#define moveBackSpeedMax 235
#define LEFT digitalPinToInterrupt(21)
#define RIGHT digitalPinToInterrupt(20)

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
boolean cb = false;
int currSpeed = 0;
int leftHeading = 0; //1 forward, 2 backward
int rightHeading = 0; //1 forward, 2 backward
int forwardBlocked = 0; //0 unblocked, 1 blocked
unsigned long lastMssgTime = 0;
float wheelDiameter = 6.56; // In cm
int wheelSeparation = 26; // In cm
int encoderTicks = 20; // Per rotation
int gearRatio =  1;
unsigned long lastMilli = 0;
volatile long coder0 = 0;  // rev counter
volatile long coder1 = 0;
long currCoder0 = 0;
long currCoder1 = 0;
long prevCoder0 = 0;
long prevCoder1 = 0;
long totalCoder0 = 0;
long totalCoder1 = 0;
long totalDiffCnt = 0;
long totalDiffs = 0;

ros::NodeHandle  nh;
std_msgs::String debug_msg;
ros::Publisher Debug ("debug_bot", &debug_msg);
geometry_msgs::Twist twist_msg;
//ros::Publisher Sensorpub ("sensor_debug", &twist_msg);
ros::Publisher Odompub ("odom_debug", &twist_msg);
sensor_msgs::Range ir_range_msg;
ros::Publisher irl_pub( "ir_left_depth_frame", &ir_range_msg);
ros::Publisher irc_pub( "ir_center_depth_frame", &ir_range_msg);
ros::Publisher irr_pub( "ir_right_depth_frame", &ir_range_msg);
sensor_msgs::Range sonar_range_msg;
ros::Publisher sl_pub( "sonar_left_depth_frame", &sonar_range_msg);
ros::Publisher sr_pub( "sonar_right_depth_frame", &sonar_range_msg);
geometry_msgs::Vector3Stamped rpm_msg;
ros::Publisher rpm_pub("rpm", &rpm_msg);

void messageCb(const geometry_msgs::Twist& msg){
  float msgX = msg.linear.x;
  float msgZ = msg.angular.z;
  if(abs(msgX) >= abs(msgZ) && (msgX > 0.1 || msgX < -0.1)){
    goalX = msgX;
    goalZ = 0;
  } else {
    goalX = 0;
    goalZ = msgZ;
  }
  cb = true;
  debug_msg.data = "RUNNING MSSG CALLBACK";
  Debug.publish(&debug_msg);
}

int nextSpeed(int minSpeed, int maxSpeed){
  if(currSpeed == 0){
    return minSpeed;
  } else if(currSpeed < maxSpeed){
    return currSpeed + 10;
  }
  return currSpeed;
}

int speedBump(){
  int currLeftCnt = abs(currCoder0);
  int currRightCnt = abs(currCoder1);
//  double diffPercent = 0.0;
//  if(currRightCnt > currLeftCnt){
//    diffPercent = (currRightCnt - currLeftCnt)/(double)currRightCnt
//  } else if (currLeftCnt > CurrRightCnt){
//    diffPercent = (currRightCnt - currLeftCnt)/(double)currRightCnt
//  }

  
  int diffCnt = currLeftCnt - currRightCnt;
  double avgDiff = 0;
  if(diffCnt == 0){
    //return 0;
  } else {
    totalDiffCnt += diffCnt;
    totalDiffs += 1;  
  }
  avgDiff = (double)totalDiffCnt/(double)totalDiffs;
  if(totalDiffs < 10 || avgDiff == 0){
    return 0;
  }
  if (avgDiff > 0){
    //left turned more
    return (100*avgDiff);
  } else {
    //right turned more, return positive bump
    return (-100*avgDiff);
  }
}

int getLeftSpeed(int correction, int moveMax){
  int newSpeed = currSpeed + correction;
  if(newSpeed > moveMax){
    return moveMax;
  } else if(newSpeed < moveSpeedMin){
    return moveSpeedMin;
  }
  return newSpeed;
}

int getRightSpeed(int correction, int moveMax){
  int newSpeed = currSpeed - correction;
  if(newSpeed > moveMax){
    return moveMax;
  } else if(newSpeed < moveSpeedMin){
    return moveSpeedMin;
  }
  return newSpeed;
}

void moveForward(){
//  debug_msg.data = "MOVING FORWARD";
//  Debug.publish(&debug_msg);
  running = true;
  leftHeading = 1;
  rightHeading = 1;
  currSpeed = nextSpeed(moveSpeedStart, moveSpeedMax);
  int correction = speedBump();
  int leftSpeed = getLeftSpeed(correction, moveSpeedMaxL);
  int rightSpeed = getRightSpeed(correction, moveSpeedMaxR);
  debugOdom(leftSpeed, rightSpeed, correction, currSpeed, totalDiffCnt, totalDiffs);
  motorLeft.run(FORWARD);
  motorLeft.setSpeed(leftSpeed);
  motorRight.run(FORWARD);
  motorRight.setSpeed(rightSpeed);  
}

void moveBackward(){
//  debug_msg.data = "MOVING BACKWARD";
//  Debug.publish(&debug_msg);
  running = true;
  leftHeading = 2;
  rightHeading = 2;
  currSpeed = nextSpeed(moveSpeedStart, moveBackSpeedMax);
  int correction = speedBump();
  int leftSpeed = getLeftSpeed(correction, moveBackSpeedMax);
  int rightSpeed = getRightSpeed(correction, moveBackSpeedMax);
  debugOdom(leftSpeed, rightSpeed, correction, currSpeed, totalDiffCnt, totalDiffs);
  motorLeft.run(BACKWARD);
  motorLeft.setSpeed(currSpeed);
  motorRight.run(BACKWARD);
  motorRight.setSpeed(currSpeed);
}

void turnLeft(){
//  debug_msg.data = "TURN LEFT";
//  Debug.publish(&debug_msg);
  running = true;
  leftHeading = 2;
  rightHeading = 1;
  currSpeed = nextSpeed(turnSpeedMin, turnSpeedMax);
  motorLeft.run(BACKWARD);
  motorLeft.setSpeed(currSpeed);
  motorRight.run(FORWARD);
  motorRight.setSpeed(currSpeed);
}

void turnRight(){
//  debug_msg.data = "TURN RIGHT";
//  Debug.publish(&debug_msg);
  running = true;
  leftHeading = 1;
  rightHeading = 2;
  currSpeed = nextSpeed(turnSpeedMin, turnSpeedMax);
  motorLeft.run(FORWARD);
  motorLeft.setSpeed(currSpeed);
  motorRight.run(BACKWARD);
  motorRight.setSpeed(currSpeed);
}

void stopMovement(){
//  debug_msg.data = "MOVEMENT STOPPED";
//  Debug.publish(&debug_msg);
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


boolean movingForward(){
  if(leftHeading == 1 && rightHeading == 1){
    return true;
  }
  return false;
}

boolean movingBackward(){
  if(leftHeading == 2 && rightHeading == 2){
    return true;
  }
  return false;
}

boolean turningLeft(){
  if(leftHeading == 2 && rightHeading == 1){
    return true;
  }
  return false;
}

boolean turningRight(){
  if(leftHeading == 1 && rightHeading == 2){
    return true;
  }
  return false;
}

void LwheelSpeed(){
  if(leftHeading == 1){
//    debug_msg.data = "recording left wheel forward action";
//    Debug.publish(&debug_msg);  
    coder0 ++;
  } else if(leftHeading == 2){
//    debug_msg.data = "recording left wheel backward action";
//    Debug.publish(&debug_msg);  
    coder0 --;
  }
}

void RwheelSpeed(){
  if(rightHeading == 1){
//    debug_msg.data = "recording right wheel forward action";
//    Debug.publish(&debug_msg);  
    coder1 ++;
  } else if(rightHeading == 2){
//    debug_msg.data = "recording right wheel backward action";
//    Debug.publish(&debug_msg);  
    coder1 --;
  }
}

boolean sonarBlocked(int val){
  if(val > 0 && val < SONAR_PERSONAL_SPACE){
    return true;
  }
  return false;
}

boolean irSideBlocked(int val){
  if(val > 0 && val < IR_SIDE_PERSONAL_SPACE){
    return true;
  }
  return false;
}

boolean irCenterBlocked(int val){
  if(val > 0 && val < IR_CENTER_PERSONAL_SPACE){
    return true;
  }
  return false;
}

boolean sensorBlocked(int sLeft, int sRight, int dLeft, int dCenter, int dRight){
  if(sonarBlocked(sLeft) || sonarBlocked(sRight) || irSideBlocked(dLeft) || irCenterBlocked(dCenter) || irSideBlocked(dRight)){
    return true;
  }
  return false;
}

//void debugSensors(int dLeft, int dCenter, int dRight, int sLeft, int sRight){
//  twist_msg.linear.x = dLeft;
//  twist_msg.linear.y = dCenter;
//  twist_msg.linear.z = dRight;
//  twist_msg.angular.x = sLeft;
//  twist_msg.angular.y = forwardBlocked;
//  twist_msg.angular.z = sRight;
//  Sensorpub.publish(&twist_msg);
//}

void publishIR(float dLeft, float dCenter, float dRight){
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
  nh.spinOnce();
}

void publishSonar(float sLeft, float sRight){
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
  nh.spinOnce();
}

void checkForBlocks(float sLeft, float sRight, float dLeft, float dCenter, float dRight){
  bool blocked = sensorBlocked(sLeft, sRight, dLeft, dCenter, dRight);
  if(blocked){
    debug_msg.data = "BLOCKING FORWARD MOTION";
    Debug.publish(&debug_msg);
    forwardBlocked = 1;    
  } else {
//      debug_msg.data = "FORWARD MOTION UNBLOCKED";
//      Debug.publish(&debug_msg); 
    forwardBlocked = 0;
  }

  if(blocked && goalX > 0.1){
    goalX = 0;
    goalZ = 0;
    debug_msg.data = "SHOULD STOP FORWARD MOTION by setting goal velocities to 0";
    Debug.publish(&debug_msg); 
  }
}

void checkSensors(){
  float sLeft = sonar_left.ping();
  float sRight = sonar_right.ping();
  float dLeft=ir_left.distance(); 
  float dCenter=ir_center.distance();
  float dRight=ir_right.distance();

  checkForBlocks(sLeft, sRight, dLeft, dCenter, dRight);
  publishIR(dLeft, dCenter, dRight);
  publishSonar(sLeft, sRight);
  //debugSensors(dLeft, dCenter, dRight, sLeft, sRight);
}

void controlMotors(){
  if(goalX != currX || goalZ != currZ || goalX > 0.1 || goalX < -0.1 || goalZ > 0.1 || goalZ < -0.1){
    if(cb == true){
      lastMssgTime = millis();
      cb = false;
    }
    currX = goalX;  // later we will slowly ramp curr up towards goal
    currZ = goalZ;  // and use an accel method to determine speed to set
    if(currX > 0.1 || currX < -0.1 || currZ > 0.1 || currZ < -0.1){
//      debug_msg.data = "NEW ACTION STARTING";
//      Debug.publish(&debug_msg);
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
//  debugSensors(millis()/1000, lastMssgTime/1000,(millis() - lastMssgTime)/1000,0,0);  
  if((millis() - lastMssgTime) > MOVEMENT_TIMEOUT){
    debug_msg.data = "STOPPING MOVEMENT DUE TO LASTMSSGTIME TIMEOUT";
    Debug.publish(&debug_msg);    
    goalX = 0;
    goalZ = 0;
  }  
}

void debugOdom(double vel_lx, double vel_az, long currCoder0, long currCoder1, long rpm0, long rpm1){
  twist_msg.linear.x = vel_lx;
  twist_msg.linear.y = vel_az;
  twist_msg.linear.z = currCoder0;
  twist_msg.angular.x = currCoder1;
  twist_msg.angular.y = rpm0;
  twist_msg.angular.z = rpm1;
  Odompub.publish(&twist_msg);
}

void publishOdom(double vel_lx, double vel_az, unsigned long time){
  rpm_msg.header.stamp = nh.now();
  rpm_msg.vector.x = vel_lx;
  rpm_msg.vector.y = vel_az;
  rpm_msg.vector.z = double(time)/1000;
  rpm_pub.publish(&rpm_msg);
  nh.spinOnce(); 
}

//double moveVelLx(long speed0, long speed1){
//  // 10 = 1000 second * .01 cm to m
//  return (((((speed0 + speed1) / 2.0) * 3.14 * wheelDiameter) / encoderTicks) * (10.0 / odomInterval)); 
//}
//
//double leftVelAz(long speed0, long speed1){
//  // 10 = 1000 second * .01 cm to m
//  return ((((((speed0 - speed1) / 2.0) * 3.14 * wheelDiameter) / encoderTicks) / (wheelSeparation / 2.0)) * (1000.0 / odomInterval)); 
//}
//
//double rightVelAz(long speed0, long speed1){
//  // 10 = 1000 second * .01 cm to m
//  return ((((((speed0 - speed1) / 2.0) * 3.14 * wheelDiameter) / encoderTicks) / (wheelSeparation / 2.0)) * (1000.0 / odomInterval)); 
//}

void handleOdometry(unsigned long time){
  double vel_lx = 0; // odom linear x velocity
  double vel_az = 0; // odom angular z velocity
  totalCoder0 = coder0;  // this method of holding the encoder value
  totalCoder1 = coder1;  // prevents us from losing any ticks
  currCoder0 = totalCoder0 - prevCoder0;
  currCoder1 = totalCoder1 - prevCoder1;
  prevCoder0 = totalCoder0;
  prevCoder1 = totalCoder1;

  double elapsed = time/(double)1000;
  double tickRatio0 = (double)currCoder0/encoderTicks;
  double tickRatio1 = (double)currCoder1/encoderTicks;
  double rpm0 = (60*(tickRatio0))/(double)elapsed;
  double rpm1 = (60*(tickRatio1))/(double)elapsed;
  
  vel_lx = double((currCoder0)*60*1000)/double(time*encoderTicks*gearRatio);
  vel_az = double((currCoder1)*60*1000)/double(time*encoderTicks*gearRatio);
  
//  if(movingForward() || movingBackward()) {
//    // forward or backwards
//    vel_lx =  moveVelLx(currCoder0, currCoder1);
//    vel_az = 0;
//  } else if(turningLeft()) {
//    // left turn
//    vel_lx = 0;
//    vel_az = leftVelAz(currCoder0, currCoder1);
//  } else if(turningRight()) {
//    // right turn
//    vel_lx = 0;
//    vel_az = rightVelAz(currCoder0, currCoder1);
//  }

  //debugOdom(tickRatio0, tickRatio1, currCoder0, currCoder1, rpm0, rpm1);
  publishOdom(vel_lx, vel_az, time);
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", messageCb);

void setup(){
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(rpm_pub);
  nh.advertise(Debug);
  nh.advertise(Odompub);
//  nh.advertise(Sensorpub);
  nh.advertise(irl_pub);
  nh.advertise(irc_pub);
  nh.advertise(irr_pub);
  nh.advertise(sl_pub);
  nh.advertise(sr_pub);
  pinMode (irl, INPUT);
  pinMode (irc, INPUT);
  pinMode (irr, INPUT);
  attachInterrupt(LEFT, LwheelSpeed, CHANGE);
  attachInterrupt(RIGHT, RwheelSpeed, CHANGE);
  motorLeft.setSpeed(turnSpeedMin);
  motorLeft.run(RELEASE);
  motorRight.setSpeed(turnSpeedMin);
  motorRight.run(RELEASE);
  ir_range_msg.radiation_type = sensor_msgs::Range::INFRARED;
  ir_range_msg.field_of_view = 0.01;
  ir_range_msg.min_range = 0.1;
  ir_range_msg.max_range = 0.8;
  sonar_range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  sonar_range_msg.field_of_view = 0.7;
  sonar_range_msg.min_range = 0.02;
  sonar_range_msg.max_range = 3; 
}

void loop(){
  static unsigned long motorTimer = 0;
  unsigned long time = millis();
  nh.spinOnce();

  if(lastMilli - motorTimer > MOTOR_INTERVAL){
    handleOdometry(time-motorTimer);
    checkSensors();    
    controlMotors();
    motorTimer = time;
  }

  lastMilli = time;
  delay(1);
}

// TODO only call run on the motor if we are switching the directions
// TODO: try turning off all the sensor stuff
// TODO: try spinning the nh more often
// TODO: have a max rpm set and use that to tune motor values
// TODO: less oscillating, different values per motor
// TODO: implement max rpm tracking for encoder to prevent extra speed
// TODO: use a custom message for odom - we are misusing Vector3Stamped and stufing left and right wheel into x and y and time into z
// TODO: do not clear the encoder count buffers, store lat values and subtract for consistency
// TODO: set millis to one var at top of loop and use consistent value
// TODO: store last millis time and last millis pub time for calculating
// TODO: use volatile keyword in variable declaration for things like rev counters 
// TODO: Make debug statements use a function
// TODO: Move vars local to the functions where possible
// TODO: Move to constants instead of vars where possible
// TODO: Serious refactor
// TODO: Pre-compute values so they don't need to be recomputed every time
// TODO: Think about moving forward and backwards from 0, 1, 2 to 0, 1, -1
// TODO: Switch time from odomInterval to actual time passed for velocity calculations
// TODO: Don't report odom when any values are out of range

