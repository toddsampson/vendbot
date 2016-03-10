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
#define GOAL_RPM 75
#define POWER_BUMP 10
#define POWER_BUMP_TURN 10
#define SONAR_PERSONAL_SPACE 1250
#define IR_CENTER_PERSONAL_SPACE 50
#define IR_SIDE_PERSONAL_SPACE 40
#define MOTOR_INTERVAL 50
#define MOVEMENT_TIMEOUT 200
#define turnSpeedMin 145
#define turnSpeedMax 180
#define moveSpeedMin 140
#define moveSpeedMax 240// Author: Sung Jik Cha
// Credits:
//   http://forum.arduino.cc/index.php?topic=8652.0
//   Dallaby   http://letsmakerobots.com/node/19558#comment-49685
//   Bill Porter  http://www.billporter.info/?p=286
//   bobbyorr (nice connection diagram) http://forum.pololu.com/viewtopic.php?f=15&t=1923

//ROS headers
#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif
#include <ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>
#include "robot_specs.h"

//Motor Shield headers
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"

#define encodPinA1      3     // encoder A pin
#define encodPinB1      8     // encoder B pin
#define encodPinA2      2
#define encodPinB2      7
#define LOOPTIME        100   // PID loop time(ms)
#define SMOOTH      10

#define sign(x) (x > 0) - (x < 0)

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Select which 'port' M1, M2, M3 or M4. 
Adafruit_DCMotor *motor1 = AFMS.getMotor(1);
Adafruit_DCMotor *motor2 = AFMS.getMotor(2);

unsigned long lastMilli = 0;       // loop timing 
unsigned long lastMilliPub = 0;
double rpm_req1 = 0;
double rpm_req2 = 0;
double rpm_act1 = 0;
double rpm_act2 = 0;
double rpm_req1_smoothed = 0;
double rpm_req2_smoothed = 0;
int direction1 = FORWARD;
int direction2 = FORWARD;
int prev_direction1 = RELEASE;
int prev_direction2 = RELEASE;
int PWM_val1 = 0;
int PWM_val2 = 0;
volatile long count1 = 0;          // rev counter
volatile long count2 = 0;
long countAnt1 = 0;
long countAnt2 = 0;
float Kp =   0.5;
float Kd =   0;
float Ki =   0;
ros::NodeHandle nh;

void handle_cmd( const geometry_msgs::Twist& cmd_msg) {
  double x = cmd_msg.linear.x;
  double z = cmd_msg.angular.z;
  if (z == 0) {     // go straight
    // convert m/s to rpm
    rpm_req1 = x*60/(pi*wheel_diameter);
    rpm_req2 = rpm_req1;
  }
  else if (x == 0) {
    // convert rad/s to rpm
    rpm_req2 = z*track_width*60/(wheel_diameter*pi*2);
    rpm_req1 = -rpm_req2;
  }
  else {
    rpm_req1 = x*60/(pi*wheel_diameter)-z*track_width*60/(wheel_diameter*pi*2);
    rpm_req2 = x*60/(pi*wheel_diameter)+z*track_width*60/(wheel_diameter*pi*2);
  }
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", handle_cmd);
geometry_msgs::Vector3Stamped rpm_msg;
ros::Publisher rpm_pub("rpm", &rpm_msg);
ros::Time current_time;
ros::Time last_time;

void setup() {
 AFMS.begin();  // create with the default frequency 1.6KHz
 count1 = 0;
 count2 = 0;
 countAnt1 = 0;
 countAnt2 = 0;
 rpm_req1 = 0;
 rpm_req2 = 0;
 rpm_act1 = 0;
 rpm_act2 = 0;
 PWM_val1 = 0;
 PWM_val2 = 0;
 nh.initNode();
 nh.getHardware()->setBaud(57600);
 nh.subscribe(sub);
 nh.advertise(rpm_pub);
  
 pinMode(encodPinA1, INPUT); 
 pinMode(encodPinB1, INPUT); 
 digitalWrite(encodPinA1, HIGH);                // turn on pullup resistor
 digitalWrite(encodPinB1, HIGH);
 attachInterrupt(1, encoder1, RISING);

 pinMode(encodPinA2, INPUT); 
 pinMode(encodPinB2, INPUT); 
 digitalWrite(encodPinA2, HIGH);                // turn on pullup resistor
 digitalWrite(encodPinB2, HIGH);
 attachInterrupt(0, encoder2, RISING);
 motor1->setSpeed(0);
 motor2->setSpeed(0);
 motor1->run(FORWARD);
 motor1->run(RELEASE);
 motor2->run(FORWARD);
 motor2->run(RELEASE);
}

void loop() {
  nh.spinOnce();
  unsigned long time = millis();
  if(time-lastMilli>= LOOPTIME)   {      // enter tmed loop
    getMotorData(time-lastMilli);
    PWM_val1 = updatePid(1, PWM_val1, rpm_req1, rpm_act1);
    PWM_val2 = updatePid(2, PWM_val2, rpm_req2, rpm_act2);

    if(PWM_val1 > 0) direction1 = FORWARD;
    else if(PWM_val1 < 0) direction1 = BACKWARD;
    if (rpm_req1 == 0) direction1 = RELEASE;
    if(PWM_val2 > 0) direction2 = FORWARD;
    else if(PWM_val2 < 0) direction2 = BACKWARD;
    if (rpm_req2 == 0) direction2 = RELEASE;
    motor1->run(direction1);
    motor2->run(direction2);

    motor1->setSpeed(abs(PWM_val1));
    motor2->setSpeed(abs(PWM_val2));
    
    publishRPM(time-lastMilli);
    lastMilli = time;
  }
  if(time-lastMilliPub >= LOOPTIME) {
  //  publishRPM(time-lastMilliPub);
    lastMilliPub = time;
  }
}

void getMotorData(unsigned long time)  {
 rpm_act1 = double((count1-countAnt1)*60*1000)/double(time*encoder_pulse*gear_ratio);
 rpm_act2 = double((count2-countAnt2)*60*1000)/double(time*encoder_pulse*gear_ratio);
 countAnt1 = count1;
 countAnt2 = count2;
}

int updatePid(int id, int command, double targetValue, double currentValue) {
  double pidTerm = 0;                            // PID correction
  double error = 0;
  double new_pwm = 0;
  double new_cmd = 0;
  static double last_error1 = 0;
  static double last_error2 = 0;
  static double int_error1 = 0;
  static double int_error2 = 0;
  
  error = targetValue-currentValue;
  if (id == 1) {
    int_error1 += error;
    pidTerm = Kp*error + Kd*(error-last_error1) + Ki*int_error1;
    last_error1 = error;
  }
  else {
    int_error2 += error;
    pidTerm = Kp*error + Kd*(error-last_error2) + Ki*int_error2;
    last_error2 = error;
  }
  new_pwm = constrain(double(command)*MAX_RPM/4095.0 + pidTerm, -MAX_RPM, MAX_RPM);
  new_cmd = 4095.0*new_pwm/MAX_RPM;
  return int(new_cmd);
}

void publishRPM(unsigned long time) {
  rpm_msg.header.stamp = nh.now();
  rpm_msg.vector.x = rpm_act1;
  rpm_msg.vector.y = rpm_act2;
  rpm_msg.vector.z = double(time)/1000;
  rpm_pub.publish(&rpm_msg);
  nh.spinOnce();
}

void encoder1() {
  if (digitalRead(encodPinA1) == digitalRead(encodPinB1)) count1++;
  else count1--;
}
void encoder2() {
  if (digitalRead(encodPinA2) == digitalRead(encodPinB2)) count2--;
  else count2++;
}
#define moveBackSpeedMax 235
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
boolean cb = false;
int currSpeedL = 0;
int currSpeedR = 0;
int currSpeedT = 0;
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
double rpm0 = 0;
double rpm1 = 0;

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

int nextSpeedRPM(int minSpeed, int maxSpeed, int currRPM, int currSpeed){
  if(abs(currRPM) < GOAL_RPM){
    currSpeed += POWER_BUMP;
  } else if (abs(currRPM) > GOAL_RPM){
    currSpeed -= POWER_BUMP;
  }
  if(currSpeed < minSpeed){
    return minSpeed;
  } else if (currSpeed > maxSpeed){
    return maxSpeed;
  }
  return currSpeed;
}

int nextSpeedTurn(int minSpeed, int maxSpeed, int currSpeed){
  if(currSpeed == 0){
    return minSpeed;
  } else if(currSpeed < maxSpeed){
    return currSpeed + POWER_BUMP_TURN;
  }
  return currSpeed;
}

int speedBump(){
  int currLeftCnt = abs(currCoder0);
  int currRightCnt = abs(currCoder1);
  int diffCnt = abs(currLeftCnt - currRightCnt);
    double avgDiff = 0;
  if(diffCnt != 0){
    totalDiffCnt += diffCnt;
    totalDiffs += 1;  
  }
  avgDiff = (double)totalDiffCnt/(double)totalDiffs;
  if(totalDiffs < 10 || avgDiff == 0){
    return 0;
  }
  if (avgDiff > 0){
    //left turned more
    return (-100*avgDiff);
  } else {
    //right turned more, return positive bump
    return (100*avgDiff);
  }
}

void moveForward(){
//  debug_msg.data = "MOVING FORWARD";
//  Debug.publish(&debug_msg);
  running = true;
  leftHeading = 1;
  rightHeading = 1;
  currSpeedL = nextSpeedRPM(moveSpeedMin, moveSpeedMax, rpm0, currSpeedL);
  currSpeedR = nextSpeedRPM(moveSpeedMin, moveSpeedMax, rpm1, currSpeedR);
  debugOdom(currSpeedL, currSpeedR, rpm0, rpm1, 0, 0);
  motorLeft.run(FORWARD);
  motorLeft.setSpeed(currSpeedL);
  motorRight.run(FORWARD);
  motorRight.setSpeed(currSpeedR);  
}

void moveBackward(){
//  debug_msg.data = "MOVING BACKWARD";
//  Debug.publish(&debug_msg);
  running = true;
  leftHeading = 2;
  rightHeading = 2;
  currSpeedL = nextSpeedRPM(moveSpeedMin, moveBackSpeedMax, rpm0, currSpeedL);
  currSpeedR = nextSpeedRPM(moveSpeedMin, moveBackSpeedMax, rpm1, currSpeedR);
  debugOdom(currSpeedL, currSpeedR, rpm0, rpm1, 0, 0);
  motorLeft.run(BACKWARD);
  motorLeft.setSpeed(currSpeedL);
  motorRight.run(BACKWARD);
  motorRight.setSpeed(currSpeedR);
}

void turnLeft(){
//  debug_msg.data = "TURN LEFT";
//  Debug.publish(&debug_msg);
  running = true;
  leftHeading = 2;
  rightHeading = 1;
  currSpeedT = nextSpeedTurn(turnSpeedMin, turnSpeedMax, currSpeedT);
  motorLeft.run(BACKWARD);
  motorLeft.setSpeed(currSpeedT);
  motorRight.run(FORWARD);
  motorRight.setSpeed(currSpeedT);
}

void turnRight(){
//  debug_msg.data = "TURN RIGHT";
//  Debug.publish(&debug_msg);
  running = true;
  leftHeading = 1;
  rightHeading = 2;
  currSpeedT = nextSpeedTurn(turnSpeedMin, turnSpeedMax, currSpeedT);
  motorLeft.run(FORWARD);
  motorLeft.setSpeed(currSpeedT);
  motorRight.run(BACKWARD);
  motorRight.setSpeed(currSpeedT);
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
  currSpeedL = 0;
  currSpeedR = 0;
  currSpeedT = 0;
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
  rpm0 = (60*(tickRatio0))/(double)elapsed;
  rpm1 = (60*(tickRatio1))/(double)elapsed;
  
  vel_lx = double((currCoder0)*60*1000)/double(time*encoderTicks*gearRatio);
  vel_az = double((currCoder1)*60*1000)/double(time*encoderTicks*gearRatio);
  
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

