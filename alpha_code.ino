#include "alpha_constants.h"
/**
  Team 4 multifunctional robot
  Functionalities:
    1. Obstacle avoidance
    2. Line Following
    3. TBD  
  */

//Arduino IR Pins

const int 
const int 

//Default mode at rest
int currentMode = 0;

/**
  Controls the turning of the robot.
  Params: degrees, if negative it will turn left, otherwise if positive it will turn right
  */ 
//TODO: SLOW/STOP THE ROBOT IF TURNING
void turn(int degrees) {
  //TODO: HAVE THIS NUMBER BE A FUNCTION OF DEGREES 
  int motorOnTime = degrees; 

  if(degrees < 0){
    digitalWrite(M1, LOW);
    digitalWrite(M2, HIGH);
    analogWrite(E1, MAX_SPEED);   //PWM Speed Control
    analogWrite(E2,MAX_SPEED);   //PWM Speed Control
    delay(motorOnTime);
  } else {
    digitalWrite(M1, HIGH);
    digitalWrite(M2, LOW);
    analogWrite(E1, MAX_SPEED);   //PWM Speed Control
    analogWrite(E2, MAX_SPEED);   //PWM Speed Control
    delay(motorOnTime);
  }

  stop();
}

/**
  Stops the robot
  */
void stop() {
  analogWrite(E1, 0);   //PWM Speed Control
  analogWrite(E2, 0);   //PWM Speed Control
}

//TODO: FIGURE OUT A FUNCTION BETWEEN WHEEL FREQ AND PWM
void straight() {
  //Get to top speed
  accelerate();
  //If once at top speed one wheel is spinning faster than other slow it down
  while(currentMode == MODE_0) {
    float freqDiff = getFreq(LEFT_HALL) - getFreq(RIGHT_HALL);
    int speed = FREQ_RATIO*abs(freqDiff);
    if(freqDiff > 0){
      analogWrite(E1, speed);
    } else {
      analogWrite(E2, speed);
    }
  }
}

/**
  Accelerates the robot to top speed
  */
void accelerateBoth() {
  for(int speed = 0; speed < MAX_SPEED; speed += 5) {
    digitalWrite(M1, HIGH);
    digitalWrite(M2, HIGH);
    analogWrite(E1, speed);   //PWM Speed Control
    analogWrite(E2, speed);   //PWM Speed Control
  }
}

/**
  Decelerates individual motor
  */
void decelLeft() {
  for(int speed = 0; speed < MAX_SPEED; speed += 5) {
    digitalWrite(M1, HIGH);
    digitalWrite(M2, HIGH);
    analogWrite(E1, speed);   //PWM Speed Control
    analogWrite(E2, speed);   //PWM Speed Control
  }
}

/**
  This section of the code reads the frequency of the hall effect sensor when we are in obstacle avoidance mode
  */
int prevVal = 0;
int count = 0;
/*
  Returns the frequency of rotation of a given wheel
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
      return 1.0/(float) elapsed*1000.0 / (float) numMagnets; 
    }
    prevVal = val;
  } else {
    prevVal = val;
  }
}