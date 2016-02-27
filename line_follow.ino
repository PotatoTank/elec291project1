const int threshold = 200;
// decrease dampening to increase sensitivity
const int driftDampening = 10;

#define rightInfraredPin A0
#define centerInfraredPin A1
#define leftInfraredPin A2

void setup(){
//  // debugging
//  Serial.begin(9600);
}

void loop(){
  int sRight = analogRead(rightInfraredPin);
  int sCenter = analogRead(centerInfraredPin);
  int sLeft = analogRead(leftInfraredPin);

//  // debugging 
//  Serial.print("Left = ");
//  Serial.print(sLeft);
//  Serial.print(" Center = ");
//  Serial.print(sCenter);
//  Serial.print(" Right = ");
//  Serial.print(sRight);

  int leftDrift = sCenter - sLeft;    // high if left is off the line
  int rightDrift = sCenter - sRight;  // high if right is off the line
  int drift = rightDrift - leftDrift; // if drift negative, rotate left

  int rotateWheels = constrain(drift/driftDampening, -90, 90);
  turn(rotateWheels);
//  Serial.print(" Rotate = ");
//  Serial.println(rotateWheels);
//
//  delay(500);
}

