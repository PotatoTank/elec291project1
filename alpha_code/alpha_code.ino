#include <Servo.h>
#include "alpha_constants.h"

/*
 * Team 4 Multifunctional Robot
 * ****************************
 * Functionalities:
 *  1. Obstacle-Avoidance
 *  2. Line-Following
 *  3. Draw a Smiley Face
 */

/* Global variables */
Servo myservo;    // Servo motor object to control the front servo
Servo penServo;   //Servo motor object to control the servo that controls the pen
int mode = 0;       // Keeps track of which mode the robot is in
bool atTopSpeed;  // True once accelerateBoth() called, false otherwise
int lastMode = 0;
long lastDebounceTime;
float referenceSpeed;
int leftWheelSpeed = 0;
int rightwheelSpeed = 0;
int switchOneState;
int switchTwoState;


/*
 * Initializes the program.
 */
void setup(){
  /* Pin inputs for the motor */
  pinMode(M1, OUTPUT);   
  pinMode(M2, OUTPUT);

  /*LED to display the current mode. */
  pinMode(LED_PIN, OUTPUT);

  /* Servo motor initialization for the motor controlling the ultrasonic*/
  Serial.begin(9600);
  myservo.attach(SERVO_PIN);
  myservo.write(90);

  /* Servo motor initialization for the motor controlling the pen */
  penServo.attach(PEN_SERVO_PIN);
  penServo.write(PEN_REST_POSITION);

  /* Ultrasonic Range Finder input pins */
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(SWITCH_ONE, INPUT);
  pinMode(SWITCH_TWO, INPUT);
  delay(1000);
}

/*
 * Main function.
 */
void loop(){
  switchOneState = digitalRead(SWITCH_ONE);
  switchTwoState = digitalRead(SWITCH_TWO);
  
  if (switchOneState == 0 && switchTwoState == 0){
      digitalWrite(LED_PIN, HIGH);
      stop();
      f_draw();
  }
  else if (switchOneState == 0 && switchTwoState == 1){
      digitalWrite(LED_PIN, HIGH);
      f_obstacle();
  }
  else if (switchOneState == 1 && switchTwoState == 0){
      digitalWrite(LED_PIN, LOW);
      f_line();
  }
  else {
    stop();
    digitalWrite(LED_PIN, LOW);
  }
  
  //This was the original switch statement for the IR controlled mode switching. We did not have a three pin IR sensor and thus, 
  //modes would switch when not intended to. We fixed this by using a DIP switch to control the different modes. 
//  switch (debounce(mode)) {
//    irRead();
//    case MODE_0:
//      digitalWrite(LED_PIN, LOW);
//      irRead();
//      break;
//    case MODE_1:
//      digitalWrite(LED_PIN, HIGH);
//      irRead();
//      f_obstacle();
//      break;
//    case MODE_2:
//      digitalWrite(LED_PIN, LOW);
//      irRead();
//      f_line();
//      break;
//    case MODE_3:
//      digitalWrite(LED_PIN, HIGH);
//      irRead();
//      f_draw();
//      break;
//  }
}

/*
 * Read the value from IR LED and change the modes accordingly. 
 */
void irRead() {
  int reading = analogRead(IR_PIN);   // Read the value from IR LED
  Serial.println("IR READ: ");
  Serial.println(reading);
  Serial.println(mode);
  if(mode == MODE_3 && reading > 1000){
    mode = MODE_0;  // Change to mode 0
  } else if (mode == MODE_0 && reading > 1000){
    mode = MODE_1;  // Change to mode 1
  } else if (mode == MODE_1 && reading > 1000){
    mode = MODE_2;  // Change to mode 2
  } else if (mode == MODE_2 && reading > 1000){
    mode = MODE_3;  // Change to mode 2
  }
}

int currentServoPosition = 90;
int currentServoDirection = 0;

/* FUNCTIONALITY 1: OBSTACLE AVOIDANCE */
/* *********************************** */
void f_obstacle() { 
  myservo.write(90);
  delay(100);
  
  //At the start of each loop get the distance
  float currentDistance = getDistance();
  
  Serial.println(currentDistance);
  
  //If there is an obstacle up ahead that is too close, then stop and choose another path
  if (currentDistance < THRESHOLD){
    slowDown();
    pickPath();
  }
  //Otherwise, if the robot is not at maximum speed, then accelerate to maximum speed. 
  //If the robot is at maximum speed, then maintain that speed and go straight.
  else {
    if(!atTopSpeed) {
      accelerateBoth(MAX_SPEED);
    } else {
      straight();
    }
  }
}

void pickBetterPath() {
  int angle = 0;
    //Set it really low
    int distanceVal = -10000;
    for(currentServoPosition = 0; currentServoPosition <= 180; currentServoPosition+=45) {
      myservo.write(currentServoPosition);
      delay(300);
    int currentDistance = getDistance(); //Define this in header file
    delay(100);
    //Get the angle where the light is the lowest
    if(currentDistance > distanceVal) {
      distanceVal = currentDistance;
      angle = currentServoPosition - 90;
    }
  }
  turn(angle);
}

void pickPath() {
    int angle = 0;
    //Set it really low
    int distanceVal = -10000;
    for(currentServoPosition = 0; currentServoPosition <= 180; currentServoPosition+=45) {
      myservo.write(currentServoPosition);
      delay(300);
    int currentDistance = getDistance(); //Define this in header file
    delay(100);
    //Get the angle where the light is the lowest
    if(currentDistance > distanceVal) {
      distanceVal = currentDistance;
      angle = currentServoPosition - 90;
    }
  }
  if(angle <= 90 && angle >= 0) {
    turn(60);
  } else {
    turn(-60);
  }
  delay(800);
}

void smoothServo(int from, int to) {
  if(from - to > 0) {
    for(from; from >= to; from --) {
      myservo.write(from);
    }
  } else {
    for(from; from <= to; from++) {
      myservo.write(from);
    }
  }
}

float getDistance() {
  //Send out a trigger signal from the ultrasonic device
  triggerSignal();

  //Record the duration of the reflected signal
  float duration = pulseIn(ECHO_PIN, HIGH);

  float distanceCM;

  //38 millis returned if no obstacle detected
  if(duration != 38000){
      // convert the time duration to distance
      distanceCM = microsecondsToCM(duration);
  }
  else {
      // no obstacles detected
      distanceCM = -1;
  }

  return distanceCM;
}

void triggerSignal(){
    // write LOW first to ensure signal is sent out correctly
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
}

float microsecondsToCM(float microseconds){
    return float(microseconds * 343.2 / 10000.0 / 2.0); 
}

/* FUNCTIONALITY 2: FOLLOW A LINE */
/* ****************************** */
void f_line() {
  //If not currently at top speed and not too close to an object accelerate
  if(!atTopSpeed) {
    accelerateBoth(200);
}

  int sRight = analogRead(RIGHT_INFRARED_PIN);
  int sCenter = analogRead(CENTER_INFRARED_PIN);
  int sLeft = analogRead(LEFT_INFRARED_PIN);
  int drift = 0;

  //Go straight if difference is small
  if((abs(sCenter-sLeft))<LINE_BOUNCING && (abs(sCenter-sRight)) < LINE_BOUNCING){
   drift = 0; 
 }

 else {
    int leftDrift = sCenter - sLeft;    // high if left is off the line
    int rightDrift = sRight - sCenter;  // high if right is off the line
    drift = rightDrift - leftDrift; // if drift negative, rotate left
  }

  // dampen drift value and limit to accepted angle range
  int rotateWheels = constrain(drift/DRIFT_DAMPENING, -90, 90);

  // avoid minute changes
  if(abs(rotateWheels) > ANGLE_THRESHOLD){
   turn(rotateWheels);
 }
}

/* FUNCTIONALITY 3: SMILE! */
/* ************************************ */

void f_draw(){
  //Draw a smiley face
  stop();
  delay(3000);
  drawSmiley();
  //draw_spiral();
  delay(2000);
}

void drawSmiley() {

  //Draw the face of the Smiley Face, which uses a medium size circle
  drawMediumCircle(SMILEY_FACE_DELAY);

  //Position the pen through a series of turns and travels so that the mouth of the smiley face can be drawn 
  turn(-60);
  delay(500);
  travelDistance(0.1);
  delay(1000);
  turn(60);
  delay(500);

  //Draw a semi circle to indicate the mouth 
  drawSmallCircle(MOUTH_DELAY);
  delay(100);

  //Position the pen through a series of turns and travels to the top of the face to begin drawing the eyes.
  travelDistance(0.1);
  delay(1000);
  turn(-60);
  delay(500);

  //Draw the left eye
  penServo.write(PEN_WRITE_POSITION);
  delay(100);
  travelDistance(0.1);
  delay(1000);
  penServo.write(PEN_REST_POSITION);
  delay(1000);

  //Travel from the left eye to the right eye
  travelDistance(0.1);

  //Draw the right eye
  penServo.write(PEN_WRITE_POSITION);
  delay(100);
  travelDistance(0.1);
  delay(1000);
  penServo.write(PEN_REST_POSITION);
  delay(100);

  //Travel off the page so that the whole image can be seen
  travelDistance(1);
}

//Function to draw a small circle with the given delay value
void drawSmallCircle(int delayValue){
   penServo.write(PEN_WRITE_POSITION);
   delay(100);
   digitalWrite(M1, HIGH);
   digitalWrite(M2, HIGH);
   analogWrite(E1, 0);   //PWM Speed Control
   analogWrite(E2, 100);   //PWM Speed Control
   penServo.write(PEN_WRITE_POSITION);
   delay(delayValue);
   stop();
   penServo.write(PEN_REST_POSITION);
   delay(100);
}

//Function to draw a medium circle with the given delay value
void drawMediumCircle(int delayValue){
   penServo.write(PEN_WRITE_POSITION);
   delay(100);
   digitalWrite(M1, HIGH);
   digitalWrite(M2, HIGH);
   
   analogWrite(E1, 75);   //PWM Speed Control
   analogWrite(E2, 150);   //PWM Speed Control
   penServo.write(PEN_WRITE_POSITION);
   delay(delayValue);
   stop();
   penServo.write(PEN_REST_POSITION);
   delay(100);
}

float frequency;
int lastVal = 0;

//Tell the robot to travel a certain distance in meters using the predetermined reference speed that was determined using intializeFrequency()
void travelDistance(float distance){
  float time = (distance / 0.425) * 1000;
  Serial.print("Travel time is ");
  Serial.println(time);
  float beginTime = millis();
  while (1){
    digitalWrite(M1, HIGH);
    digitalWrite(M2, HIGH);
    analogWrite(E1, 200);   //PWM Speed Control
    analogWrite(E2, 200);   //PWM Speed Control
    float currentTime = millis();
    if (currentTime - beginTime > time){
      break;
    }
  }
  stop();
}

//Function that was used to determine the reference speed, which is used in implementing travelDistance()
void initializeFrequency(){
  //Find the circumference of the wheel 
  float circumference = 2 * PI * RADIUS_IN_METERS;

  //Set the motors to run at the max speed and count the amount of times that the hall effect sensors read a change in value, while recording the beginning time.
  int count = 0;
  digitalWrite(M1, HIGH);
  digitalWrite(M2, HIGH);
  analogWrite(E1, MAX_SPEED);   //PWM Speed Control
  analogWrite(E2, MAX_SPEED);   //PWM Speed Control
  int beginningTime = millis();
  while (count != 20){
    int val = digitalRead(RIGHT_HALL);
    if (val != lastVal){
      lastVal = val;
      count++;
    }
  }
  count = 0;

  //Find the time interval that it took the wheels to turn one rotation and use that to determine the frequency.
  float timeInterval = (millis() - beginningTime) / 2;

  float timeIntervalInSeconds = timeInterval / 1000.0;

  //Find the frequency of the wheel
  float frequency = 1 / timeIntervalInSeconds;

  //Determine the reference speed of the wheel
  referenceSpeed = frequency * circumference;
  stop();
  delay(3000);
}

//Function to turn right while pivoting on the stationary left wheel.
void swingTurnRight(){
   digitalWrite(M1, LOW);
   digitalWrite(M2, HIGH);
   analogWrite(E1, 0);   //PWM Speed Control
   analogWrite(E2, 200);   //PWM Speed Control
   delay(600);
   stop();
}

//Function to turn left while pivoting on the stationary right wheel.
void swingTurnLeft(){
   digitalWrite(M1, HIGH);
   digitalWrite(M2, LOW);
   analogWrite(E1, 200);   //PWM Speed Control
   analogWrite(E2, 0);   //PWM Speed Control
   delay(600);
   stop();
}


/* Basic Movement of the Robot          */
/* ************************************ */

/*
 * Controls the turning of the robot.
 * Params: degrees, if negative it will turn left, otherwise if positive it will turn right
 */ 
void turn(int angle) {
  int onTime = (float) abs(5.2*((float)angle));
  if(angle < 0){
    digitalWrite(M1, LOW);
    digitalWrite(M2, HIGH);
    analogWrite(E1, 200);   //PWM Speed Control
    analogWrite(E2, 200);   //PWM Speed Control
    delay(onTime);
    } else {
      digitalWrite(M1, HIGH);
      digitalWrite(M2, LOW);
    analogWrite(E1, 200);   //PWM Speed Control
    analogWrite(E2, 200);   //PWM Speed Control
    delay(onTime);
  }
  digitalWrite(M1, HIGH);
  digitalWrite(M2, HIGH);
  myservo.write(90);
  stop();
}

/*
 * Decelerates and stops the robot
 */
 void stop() {
  analogWrite(E1, 0);   //PWM Speed Control
  analogWrite(E2, 0);   //PWM Speed Control
  atTopSpeed = false;
}

void straight() {
  //If once at top speed one wheel is spinning faster than other slow it down
  for(int count = 0; count < 255; count += 5) {
    float rightHall = 52*getFreq(RIGHT_HALL);
    float leftHall = 52*getFreq(LEFT_HALL);
    float freqDiff = rightHall - leftHall;
    if(leftHall < 1 || rightHall < 1) {
      return;
    }
    if(freqDiff > 0.5){
      analogWrite(E1, freqDiff + leftHall);
    } else if(freqDiff < -0.5) {
      analogWrite(E2, rightHall + freqDiff);
    } 
  }
}

/*
 * Accelerates the robot to top speed.
 */
void accelerateBoth(int topSpeed) {
  for(int speed = 0; speed < topSpeed; speed += 5) {
    digitalWrite(M1, HIGH);
    digitalWrite(M2, HIGH);
    analogWrite(E1, speed);   //PWM Speed Control
    analogWrite(E2, speed);   //PWM Speed Control
  }
  atTopSpeed = true;
}

/*
 * Slow down the robot from max speed. 
 */
void slowDown (){
  int currentSpeed = MAX_SPEED;
  for(currentSpeed; currentSpeed >= 0; currentSpeed -= 1) {
    digitalWrite(M1, HIGH);
    digitalWrite(M2, HIGH);
    analogWrite(E1, currentSpeed);   //PWM Speed Control
    analogWrite(E2, currentSpeed);   //PWM Speed Control
  }
  atTopSpeed = false;
}

/*
 * This section of the code reads the frequency of the hall effect sensor when we are in obstacle-avoidance mode.
 */

 int prevVal = 0;
 int count = 0;
 int start = 0;

/*
 * Returns the frequency of rotation of a given wheel
 */
float getFreq(int wheel) {
  int val = digitalRead(wheel);
  int elapsed = 0;
  float freq = -1.0;
  
  if(prevVal == 0 && prevVal != val) {
    //Start timer at state change
    if(count == 0) {
      start = millis();
      count++;
      } else {
        elapsed = millis() - start;
        count = 0;
        return 1.0/(float) elapsed*1000.0 / 5.0; 
      }
      prevVal = val;
      } else {
        prevVal = val;
      }
}

int debounce(int currentMode){
  if (currentMode != lastMode){
    lastDebounceTime = millis();
    lastMode = currentMode;
  }
  else if ((millis() - lastDebounceTime) > debounceDelay){
    if (currentMode == lastMode){
      return currentMode;
    }
  }
  return -1;
}
