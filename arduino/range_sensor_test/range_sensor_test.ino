#include <NewPing.h>
#include <SharpIR.h>

#define ir A6
#define model 20150
#define PING_PIN  22
#define MAX_DISTANCE 200

NewPing sonar(PING_PIN, PING_PIN, MAX_DISTANCE);

SharpIR sharp(ir, 25, 93, model);

void setup() {
  Serial.begin(9600);
  pinMode (ir, INPUT);
}

void loop() {
  delay(50);
  unsigned int uS = sonar.ping();
  Serial.print("Sonar: ");
  Serial.print(uS / US_ROUNDTRIP_CM);
  Serial.println("cm");

  int dis=sharp.distance();
  Serial.print("Sharp: ");
  Serial.println(dis);
}
