
void setup(){
//  // debugging
//  Serial.begin(9600);
}

void loop(){
  int sRight = analogRead(rightInfraredPin);
  int sCenter = analogRead(centerInfraredPin);
  int sLeft = analogRead(leftInfraredPin);
  int drift = 0;
  // debugging 
//  Serial.print("Left = ");
//  Serial.print(sLeft);
//  Serial.print(" Center = ");
//  Serial.print(sCenter);
//  Serial.print(" Right = ");
//  Serial.print(sRight);

  if((abs(sCenter-sLeft))<bouncing && (abs(sCenter-sRight)) < bouncing){
   drift = 0; 
  }
  else {
    int leftDrift = sCenter - sLeft;    // high if left is off the line
    int rightDrift = sRight - sCenter;  // high if right is off the line
    drift = rightDrift - leftDrift; // if drift negative, rotate left
  }
  
  int rotateWheels = constrain(drift/driftDampening, -90, 90);
  turn(rotateWheels);
//  Serial.print(" Rotate = ");
//  Serial.println(rotateWheels);
//
//  delay(500);
}
