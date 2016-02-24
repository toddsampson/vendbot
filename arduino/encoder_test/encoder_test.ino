// # 
// # Editor     : Lauren from DFRobot
// # Date       : 17.01.2012

// # Product name: Wheel Encoders for DFRobot 3PA and 4WD Rovers
// # Product SKU : SEN0038

// # Description:
// # The sketch for using the encoder on the DFRobot Mobile platform

// # Connection:
// #        left wheel encoder  -> Digital pin 2
// #        right wheel encoder -> Digital pin 3
// #


#define LEFT digitalPinToInterrupt(20)
#define RIGHT digitalPinToInterrupt(21)

long coder[2] = {
  0,0};
int lastSpeed[2] = {
  0,0};  


void setup(){

  Serial.begin(9600);                            //init the Serial port to print the data
  attachInterrupt(LEFT, LwheelSpeed, CHANGE);    //init the interrupt mode for the digital pin 2
  attachInterrupt(RIGHT, RwheelSpeed, CHANGE);   //init the interrupt mode for the digital pin 3

}

void loop(){

  static unsigned long timer = 0;                //print manager timer

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

}


void LwheelSpeed()
{
  Serial.println(lastSpeed[0]);
  coder[0] ++;  //count the left wheel encoder interrupts
}


void RwheelSpeed()
{
  Serial.println(lastSpeed[1]);
  coder[1] ++; //count the right wheel encoder interrupts
}


