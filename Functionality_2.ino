//Servo Left;
//Servo Right;

// black/white infrared sensor constant
const int threshold = 200;

void setup() {
  Serial.begin(9600);

  int val = 0;
//  for(int i=0; i<5000; i++){

  //  for(int j=0; j<2; j++){
    //  val = analogRead(j);
     // if(
    //}
  //}
  
}

void loop() {
  //Sright on black == high
  //sleft on black == high
  int sRight = analogRead(A0);
  int sLeft = analogRead(A1);

  Serial.print("Left: ");
  Serial.print(sLeft);
  Serial.print(" ");
  Serial.print("Right: ");
  Serial.print(sRight);
  Serial.println("");
  delay(1000);

  // left not on line, right on line
  if(sLeft < threshold && sRight > threshold){
    // move right by increasing left speed or dec right speed
  }
  // left on line, right not on line
  if(sLeft > threshold && sRight < threshold{
    // move left by increasing right speed or dec left speed
  }
  // delay(500); to allow change?

}