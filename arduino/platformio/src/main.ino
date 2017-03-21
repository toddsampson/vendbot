//#define USE_USBCON // Leonardo
#include <SoftwareSerial.h>
#include <SabertoothSimplified.h>
#include <Arduino.h>
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <stdio.h>


SoftwareSerial SWSerial(NOT_A_PIN, 11); // RX on no pin (unused), TX on pin 11 (to S1).
SabertoothSimplified ST(SWSerial); // Use SWSerial as the serial port.


//SabertoothSimplified ST; // We'll name the Sabertooth object ST.
                         // For how to configure the Sabertooth, see the DIP Switch Wizard for
                         //   http://www.dimensionengineering.com/datasheets/SabertoothDIPWizard/start.htm
                         // Be sure to select Simplified Serial Mode for use with this library.
                         // This sample uses a baud rate of 9600.
                         //
                         // Connections to make:
                         //   Arduino TX->1  ->  Sabertooth S1
                         //   Arduino GND    ->  Sabertooth 0V
                         //   Arduino VIN    ->  Sabertooth 5V (OPTIONAL, if you want the Sabertooth to power the Arduino)
                         //
                         // If you want to use a pin other than TX->1, see the SoftwareSerial example.

#define MOTOR_INTERVAL 40 // running motors and reporting rpm at 20 hz
#define MOVEMENT_TIMEOUT 255
#define TURN_SPEED_MIN 10
#define TURN_SPEED_MAX 80
#define MOVE_SPEED_MIN 10
#define MOVE_SPEED_MAX 80

float currX = 0.0;
float currZ = 0.0;
float goalX = 0.0;
float goalZ = 0.0;
boolean cb = false;
byte currSpeed = 0;
//byte forwardBlocked = 0; //0 unblocked, 1 blocked
unsigned long lastMssgTime = 0;
long prevCoder0 = 0;


ros::NodeHandle  nh;
std_msgs::String debugMsg; //should be off in production - only used by debug_bot
ros::Publisher DebugPub ("debug_bot", &debugMsg); //should be off in production


/* --------------------------------- */
/* MOTORS -------------------------- */
/* --------------------------------- */
// callback for receiving a cmd_vel command
// we only expect a value on x or z
// and so we only set one of x or z
// and thus our robot either moves or turns
void messageCb(const geometry_msgs::Twist& msg){
   debugMsg.data = "CALLBACK CALLED";
      DebugPub.publish(&debugMsg);
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
}

ros::Subscriber<geometry_msgs::Twist> cmd_ctrl("cmd_vel", messageCb);

void controlMotors(){
   debugMsg.data = "CTRL MOTORS CALLED";
      DebugPub.publish(&debugMsg);
  if(goalX != currX || goalZ != currZ || goalX > 0.1 || goalX < -0.1 || goalZ > 0.1 || goalZ < -0.1){
    if(cb == true){
      lastMssgTime = millis();
      cb = false;
    }
    currX = goalX;
    currZ = goalZ;
    if(currX > 0.1 || currX < -0.1 || currZ > 0.1 || currZ < -0.1){
      debugMsg.data = "NEW ACTION STARTING";
      DebugPub.publish(&debugMsg);
      if(currX > 0.1){
        //if(forwardBlocked == 0){
          moveForward();
        //}
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
  if((millis() - lastMssgTime) > MOVEMENT_TIMEOUT){
    // debugMsg.data = "STOPPING MOVEMENT DUE TO LASTMSSGTIME TIMEOUT";
    // DebugPub.publish(&debugMsg);    
    goalX = 0;
    goalZ = 0;
  }  
}

void moveForward(){
  debugMsg.data = "MOVE FORWARD MOVE MOVE MOVE MOVE MOVE FORWARD";
  DebugPub.publish(&debugMsg);
  currSpeed = nextSpeed(MOVE_SPEED_MIN, MOVE_SPEED_MAX);
  ST.motor(1, 80);
  ST.motor(2, 80);
}

void moveBackward(){
  currSpeed = nextSpeed(MOVE_SPEED_MIN, MOVE_SPEED_MAX);
  ST.motor(1, -currSpeed);
  ST.motor(2, -currSpeed);
}

void turnLeft(){
  currSpeed = nextSpeed(TURN_SPEED_MIN, TURN_SPEED_MAX);
  ST.motor(1, -currSpeed);
  ST.motor(2, currSpeed);
}

void turnRight(){
  currSpeed = nextSpeed(TURN_SPEED_MIN, TURN_SPEED_MAX);
  ST.motor(1, currSpeed);
  ST.motor(2, -currSpeed);
}

void stopMovement(){
  //debugMsg.data = "MOVEMENT STOPPED";
  //DebugPub.publish(&debugMsg);
  if(currSpeed > 55){
    ST.motor(1, 55);
    ST.motor(2, 55);
    delay(40);
  }
  if(currSpeed < -55){
    ST.motor(1, -55);
    ST.motor(2, -55);
    delay(40);
  }
  if(currSpeed > 45){
    ST.motor(1, 45);
    ST.motor(2, 45);
    delay(40);
  }
  if(currSpeed < -45){
    ST.motor(1, -45);
    ST.motor(2, -45);
    delay(40);
  }
  if(currSpeed > 35){
    ST.motor(1, 35);
    ST.motor(2, 35);
    delay(40);
  }
  if(currSpeed < -35){
    ST.motor(1, -35);
    ST.motor(2, -35);
    delay(40);
  }
  if(currSpeed > 25){
    ST.motor(1, 25);
    ST.motor(2, 25);
    delay(40);
  }
  if(currSpeed < -25){
    ST.motor(1, -25);
    ST.motor(2, -25);
    delay(40);
  }
  if(currSpeed > 15){
    ST.motor(1, 15);
    ST.motor(2, 15);
    delay(40);
  }
  if(currSpeed < -15){
    ST.motor(1, -15);
    ST.motor(2, -15);
    delay(40);
  }  
  if(currSpeed > 5){
    ST.motor(1, 5);
    ST.motor(2, 5);
    delay(40);
  }
  if(currSpeed < -5){
    ST.motor(1, -5);
    ST.motor(2, -5);
    delay(40);
  }  
  ST.motor(1, 0);
  ST.motor(2, 0);
  currX = 0;
  currZ = 0;
  goalX = 0;
  goalZ = 0;
  currSpeed = 0;
}

int nextSpeed(int minSpeed, int maxSpeed){
  if(currSpeed == 0){
    return minSpeed;
  } else if(currSpeed < maxSpeed){
    return currSpeed + 5;
  } else {
    return currSpeed;
  }
}

void setup()
{

  nh.initNode(); // start ros
  nh.advertise(DebugPub); // general debug messages

  //motors
  nh.subscribe(cmd_ctrl); // cmd vel messages
  SWSerial.begin(9600);
  
}

void loop()
{

  static unsigned long motorTimer = 0;
  unsigned long time = millis();
  nh.spinOnce();
  //debugMsg.data = "loop";
  //DebugPub.publish(&debugMsg);
  if(time - motorTimer > MOTOR_INTERVAL){
    //handleOdometry(time-motorTimer);
    //checkSensors();    //TODO turn sensors back on
    controlMotors();
    motorTimer = time;
  }
  delay(1);
}
