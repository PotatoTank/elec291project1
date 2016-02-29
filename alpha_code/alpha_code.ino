#include <Servo.h>
#include "alpha_constants.h"

/*
 * Team 4 Multifunctional Robot
 * ****************************
 * Functionalities:
 *  1. Obstacle-Avoidance
 *  2. Line-Following
 */

/* Global variables */
Servo myservo;  // Servo motor object to control the servo
int mode;       // Keeps track of which mode the robot is in
bool atTopSpeed; // True once accelerateBoth() called, false otherwise

/*
 * Initializes the program.
 */
void setup(){
  /* Pin inputs for the motor */
  pinMode(M1, OUTPUT);   
  pinMode(M2, OUTPUT); 

  /* Servo motor initialization */
  Serial.begin(38400);
  myservo.attach(SERVO_PIN);
  myservo.write(90);

  /* Ultrasonic Range Finder input pins */
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  /* Initialize variables */
  mode = MODE_0; // initalizes robot to "stop" mode
}

/*
 * Main function.
 */
void loop(){
  // check switches to determine mode..
  mode = 1;
  switch (mode) {
    case MODE_0:
      break;
    case MODE_1:
      f_obstacle();
      break;
    case MODE_2:
      f_line();
      break;
  }
}

/* FUNCTIONALITY 1: OBSTACLE AVOIDANCE */
/* *********************************** */
void f_obstacle() { 
  //Have the sensor face forwards
  myservo.write(90);
  //At the start of each loop get the distance
  float currentDistance = getDistance();

  //If not currently at top speed and not too close to an object accelerate
  if(!atTopSpeed && currentDistance > THRESHOLD) {
    accelerateBoth(MAX_SPEED);
  }

  //If things are too close 
  if (distanceCM < THRESHOLD){
      stop();
      pickPath();
    }

  //Otherwise if nothing wrong maintain straight path
  straight();

  // cycle period - 50 ms
  delayMicroseconds(50);
}

float getDistance() {
  // send out a trigger signal from the ultrasonic device
  triggerSignal();
    
  // record the duration of the reflected signal
  float duration = pulseIn(ECHO_PIN, HIGH);

  float distanceCM;

  // 38 millis returned if no obstacle detected
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

int pickPath() {
  delay(1000);
  myservo.write(30);
  float leftVal = getDistance();
  delay(1000);

  myservo.write(90);
  
  delay(1000);
  myservo.write(150);
  float rightVal = getDistance();
  delay(1000);

  //If rightVal is greater turn right
  //If leftVal is greater turn left
  //If both are less than threshold turn 180
  if(rightVal > leftVal) {
    turn(80);
  } else if(leftVal < rightVal) {
    turn(-80);
  } else if(rightVal < THRESHOLD && leftVal < THRESHOLD) {
    turn(180);
  }
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
    return float(microseconds * speedOfSound() / 10000.0 / 2.0); 
}

// calculate the current speed of sound with temperature as one of its factors
float speedOfSound(){
    // read the temperature from the LH35 sensor
    float degrees_C = (5.0 * analogRead(TEMP_PIN) * 100.0 ) / 1024;
    // compute more accurate speed of sound by using current temp
    float speedOfSound = 331.5 + (0.6 * degrees_C);
    return speedOfSound;
}

/* FUNCTIONALITY 2: FOLLOW A LINE */
/* ****************************** */
void f_line() {

  //If not currently at top speed and not too close to an object accelerate
  if(!atTopSpeed) {
    accelerateBoth(150);
  }

  int sRight = analogRead(RIGHT_INFRARED_PIN);
  int sCenter = analogRead(CENTER_INFRARED_PIN);
  int sLeft = analogRead(LEFT_INFRARED_PIN);

  //If there isn't anything there don't move
  if(sRight > 1023 && sLeft > 1023 && sCenter > 1023) {
    stop();
  }

  int leftDrift = sCenter - sLeft;    // high if left is off the line
  int rightDrift = sCenter - sRight;  // high if right is off the line
  int drift = rightDrift - leftDrift; // if drift negative, rotate left

  int angle = constrain(drift/DRIFT_DAMPENING, -90, 90); // ask Angy what driftDampening is

  turn(angle); // continuously adjust the angle
}

/*
 * Controls the turning of the robot.
 * Params: degrees, if negative it will turn left, otherwise if positive it will turn right
 */ 
void turn(int angle) {
  int onTime = (float) 3.5*(angle - 5.0);
  if(angle < 0){
    digitalWrite(M1, LOW);
    digitalWrite(M2, HIGH);
    analogWrite(E1, 200);   //PWM Speed Control
    analogWrite(E2, 200);   //PWM Speed Control
    delay(onTime);
    stop();
  } else {
    digitalWrite(M1, HIGH);
    digitalWrite(M2, LOW);
    analogWrite(E1, 200);   //PWM Speed Control
    analogWrite(E2, 200);   //PWM Speed Control
    delay(onTime);
    stop();
  }
}

/*
 * Decelerates and stops the robot
 */
void stop() {
  for(int speed = 255; speed > 0; speed -= 5) {
    analogWrite(E1, speed);   //PWM Speed Control
    analogWrite(E2, speed);   //PWM Speed Control
    delay(50);
  }
  atTopSpeed = false;
}

void straight() {
  //If once at top speed one wheel is spinning faster than other slow it down
  for(int count = 0; count < 255; count += 5) {
    float freqDiff = abs(getFreq(3) - getFreq(2));
    if(freqDiff > 0.2){
      analogWrite(E1, 52*getFreq(3));
    }
  }
}

/*
 * Accelerates the robot to top speed.
 */
void accelerateBoth(int topSpeed) {
  for(int speed = 0; speed < topSpeed; speed += 5) {
    digitalWrite(M1, LOW);
    digitalWrite(M2, LOW);
    analogWrite(E1, speed);   //PWM Speed Control
    analogWrite(E2, speed);   //PWM Speed Control
  }

  atTopSpeed = true;
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

/* FUNCTIONALITY 3: LET THERE BE LIGHT! */
/* ************************************ */
