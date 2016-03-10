#include <Arduino.h>
#include <AFMotor.h>
#define MOTOR_INTERVAL 500
#define GOOD_RPM 10
#define LEFT digitalPinToInterrupt(21)
#define RIGHT digitalPinToInterrupt(20)
AF_DCMotor motorLeft(3); //left wheel
AF_DCMotor motorRight(1); //right wheel

int currSpeedL = 100;
int currSpeedR = 100;
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
long totalCoder0 = 0;  // this method of holding the encoder value
long totalCoder1 = 0;  // prevents us from losing any ticks


void moveForward(){

}

void LwheelSpeed(){
  coder0 += 1;
}

void RwheelSpeed(){
  coder1 += 1;
}

void setup(){
  Serial.begin(115200); 
  attachInterrupt(LEFT, LwheelSpeed, CHANGE);
  attachInterrupt(RIGHT, RwheelSpeed, CHANGE);
  motorLeft.setSpeed(255);
  motorLeft.run(RELEASE);
  motorRight.setSpeed(255);
  motorRight.run(RELEASE);
}

void loop(){
  static unsigned long motorTimer = 0;
  unsigned long time = millis();

  if(lastMilli - motorTimer > MOTOR_INTERVAL){

  

    totalCoder0 = coder0;  // this method of holding the encoder value
    totalCoder1 = coder1;  // prevents us from losing any ticks
    currCoder0 = totalCoder0 - prevCoder0;
    currCoder1 = totalCoder1 - prevCoder1;
    prevCoder0 = totalCoder0;
    prevCoder1 = totalCoder1;    

    if(currCoder0 < GOOD_RPM){
      currSpeedL += 1;
    } else if(currCoder0 > GOOD_RPM){
      currSpeedL -= 1;  
    } else {
      Serial.println("GOOD LEFT SPEED");
    }
    Serial.print("left coder: ");
    Serial.println(currCoder0);    
    Serial.print("left speed: ");
    Serial.println(currSpeedL);

    if(currCoder1 < GOOD_RPM){
      currSpeedR += 1;
    } else if(currCoder1 > GOOD_RPM){
      currSpeedR -= 1;  
    } else {
      Serial.println("GOOD RIGHT SPEED");
    }
    Serial.print("right coder: ");
    Serial.println(currCoder1);    
    Serial.print("right speed: ");
    Serial.println(currSpeedR);
    
    motorLeft.run(FORWARD);
    motorLeft.setSpeed(currSpeedL);
    motorRight.run(FORWARD);
    motorRight.setSpeed(currSpeedR);  
    motorTimer = time;
  }
  lastMilli = time;
  
}

