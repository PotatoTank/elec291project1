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
//int start;      // Used to keep track of current time

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
      //rest();
      break;
    case MODE_1:
      f_obstacle();
      break;
    case MODE_2:
      f_line();
      break;
  }
}

/*
 * Controls the turning of the robot.
 * Params: degrees, if negative it will turn left, otherwise if positive it will turn right
 */ 
/*
//TODO: SLOW/STOP THE ROBOT IF TURNING
void turn(int degrees) {
  //TODO: HAVE THIS NUMBER BE A FUNCTION OF DEGREES 
  int motorOnTime = 3.5*((float)degrees - 5.0); 
  
  if(degrees < 0){
    digitalWrite(M1, LOW);
    digitalWrite(M2, HIGH);
    analogWrite(E1, TURN_SPEED);   //PWM Speed Control
    analogWrite(E2, TURN_SPEED);   //PWM Speed Control
    delay(motorOnTime);
  } 
  else {
    digitalWrite(M1, HIGH);
    digitalWrite(M2, LOW);
    analogWrite(E1, MAX_SPEED);   //PWM Speed Control
    analogWrite(E2, MAX_SPEED);   //PWM Speed Control
    delay(motorOnTime);
  }

  stop();
}
*/

/*
 * Stops the robot.
 */
 /*
void stop() {
  analogWrite(E1, 0);   //PWM Speed Control
  analogWrite(E2, 0);   //PWM Speed Control
}
*/
/*
//TODO: FIGURE OUT A FUNCTION BETWEEN WHEEL FREQ AND PWM
void straight() {
  //Get to top speed
  //accelerate();
  //If once at top speed one wheel is spinning faster than other slow it down
//  while(currentMode == MODE_0) {
//    float freqDiff = getFreq(LEFT_HALL) - getFreq(RIGHT_HALL);
//    int speed = FREQ_RATIO*abs(freqDiff);
//    if(freqDiff > 0){
//      analogWrite(E1, speed);
//    } else {
//      analogWrite(E2, speed);
//    }
//  }
}
/*
/*
 * Accelerates the robot to top speed.
 */
 /*
void accelerateBoth() {
  for(int speed = 0; speed < MAX_SPEED; speed += 5) {
    digitalWrite(M1, HIGH);
    digitalWrite(M2, HIGH);
    analogWrite(E1, speed);   //PWM Speed Control
    analogWrite(E2, speed);   //PWM Speed Control
  }
}
*/
/*
 * Decelerates individual motor (left).
 */
 /*
void decelLeft() {
  for(int speed = 0; speed < MAX_SPEED; speed += 5) {
    digitalWrite(M1, HIGH);
    digitalWrite(M2, HIGH);
    analogWrite(E1, speed);   //PWM Speed Control
    analogWrite(E2, speed);   //PWM Speed Control
  }
}
*/

/*
 * This section of the code reads the frequency of the hall effect sensor when we are in obstacle-avoidance mode.
 */
 /*
int prevVal = 0;
int count = 0;
/*
 * Returns the frequency of rotation of a given wheel
 */
 /*
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
      //elapsed = millis() - start;
      count = 0;
      return 1.0/(float) elapsed*1000.0 / (float) numMagnets; 
    }
    prevVal = val;
  } else {
    prevVal = val;
  }
}
*/

/* FUNCTIONALITY 1: OBSTACLE AVOIDANCE */
/* *********************************** */
void f_obstacle() 
{ 
  //DC Motor control
  //int value;
  //for(value = 0 ; value <= 255; value+=5) 
  //{ 
    //digitalWrite(M1,HIGH);   
    //digitalWrite(M2, HIGH);       
    //analogWrite(E1, value);   //PWM Speed Control
    //analogWrite(E2, value);   //PWM Speed Control
    //delay(30); 
  //}  

  //Servo motor control 
  //myservo.write(0);
  //delay(1000);
  //myservo.write(90);
  //delay(1000);
  //myservo.write(180);
  //delay(1000);
  //myservo.write(90);
  //delay(1000);

  //Ultrasonic Range Finder 
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
  if (distanceCM < 10){
      stop();
      delay(1000);
      myservo.write(0);
      delay(1000);
      myservo.write(90);
      delay(1000);
      myservo.write(180);
      delay(1000);
      myservo.write(90);
      delay(1000);
  }
  else {
      accelerateBoth();
      straight();
      delay(1000);
  }

  // cycle period - 50 ms
  delayMicroseconds(50);
    
  delay(100);
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
  if(sLeft < LIGHT_THRESHOLD && sRight > LIGHT_THRESHOLD){
    // move right by increasing left speed or dec right speed
  }
  // left on line, right not on line
  if(sLeft > LIGHT_THRESHOLD && sRight < LIGHT_THRESHOLD){
    // move left by increasing right speed or dec left speed
  }
}

/*
 * Controls the turning of the robot.
 * Params: degrees, if negative it will turn left, otherwise if positive it will turn right
 */ 
void turn(int angle) {
  int onTime = (float) 3.5*(angle - 5.0);
  if(angle < 0){
    digitalWrite(M1, HIGH);
    digitalWrite(M2, LOW);
    analogWrite(E1, 200);   //PWM Speed Control
    analogWrite(E2, 200);   //PWM Speed Control
    delay(onTime);
    stop();
  } else {
    digitalWrite(M1, LOW);
    digitalWrite(M2, HIGH);
    analogWrite(E1, 200);   //PWM Speed Control
    analogWrite(E2, 200);   //PWM Speed Control
    delay(onTime);
    stop();
  }
}

/*
 * Stops the robot.
 */
void stop() {
  for(int speed = 0; speed < 255; speed += 5) {
    digitalWrite(M1, HIGH);
    digitalWrite(M2, HIGH);
    analogWrite(E1, 0);   //PWM Speed Control
    analogWrite(E2, 0);   //PWM Speed Control
  }
}

//TODO: FIGURE OUT A FUNCTION BETWEEN WHEEL FREQ AND PWM
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
void accelerateBoth() {
  for(int speed = 0; speed < 255; speed += 5) {
    digitalWrite(M1, HIGH);
    digitalWrite(M2, HIGH);
    analogWrite(E1, speed);   //PWM Speed Control
    analogWrite(E2, speed);   //PWM Speed Control
  }
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
