#include <Servo.h>
#include "alpha_constants.h"

/*
 * Team 4 Multifunctional Robot
 * ****************************
 * Functionalities:
 *  1. Obstacle-Avoidance
 *  2. Line-Following
 *  3. Glorified Lamp
 */
int leftWheelSpeed = 0;
int rightwheelSpeed = 0;

/* Global variables */
Servo myservo;  // Servo motor object to control the servo
int mode=0;       // Keeps track of which mode the robot is in
bool atTopSpeed; // True once accelerateBoth() called, false otherwise

/*
 * Initializes the program.
 */
 void setup(){
  /* Pin inputs for the motor */
  pinMode(M1, OUTPUT);   
  pinMode(M2, OUTPUT); 
  pinMode(TRANS_IR, OUTPUT);

  pinMode(TRANS_IR, OUTPUT);

  /* Servo motor initialization */
  Serial.begin(38400);
  myservo.attach(SERVO_PIN);
  myservo.write(90);

  /* Ultrasonic Range Finder input pins */
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

/*
 * Main function.
 */
 
void loop(){
  switch (debounce(mode)) {
//  irRead();
//  Serial.println(mode);
    case MODE_0:
    digitalWrite(TRANS_IR, HIGH);
    irRead();
    break;
    case MODE_1:
    digitalWrite(TRANS_IR, HIGH);
    irRead();
    f_obstacle();
    break;
    case MODE_2:
    irRead();
    f_line();
    break;
    case MODE_3:
    digitalWrite(TRANS_IR, HIGH);
    irRead();
    f_light();
    break;
  }
}

/*
 * Read the value from IR LED and change the modes accordingly. 
 */
 void irRead() {
  reading = analogRead(IR_PIN);   // Read the value from IR LED
//  Serial.println("IR READ: ");
//  Serial.println(reading);
//  Serial.println(mode);
  if(mode == MODE_3 && reading > 800){
    mode = MODE_0;  // Change to mode 0
    } else if (mode == MODE_0 && reading > 800){
    mode = MODE_1;  // Change to mode 1
    } else if (mode == MODE_1 && reading > 800){
    mode = MODE_2;  // Change to mode 2
    } else if (mode == MODE_2 && reading > 800){
    mode = MODE_3;  // Change to mode 3
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
  //If not currently at top speed and not too close to an object accelerate
  if(!atTopSpeed && (currentDistance > THRESHOLD)) {
    accelerateBoth(MAX_SPEED);
    } else {
      straight();
    }
  Serial.println(currentDistance);
  //If things are too close 
  if (currentDistance < THRESHOLD){
    stop();
    pickPath();
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
    turn(90);
  } else {
    turn(-90);
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
    accelerateBoth(250);
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

  Serial.print(sLeft);
  Serial.print(" ");
  Serial.print(sCenter);
  Serial.print(" ");
  Serial.print(sRight);
  Serial.print(" ");
  Serial.println (rotateWheels);

  // avoid minute changes
  if(abs(rotateWheels) > ANGLE_THRESHOLD){
   turn(0.5*rotateWheels);
 }
}

/*
 * Controls the turning of the robot.
 * Params: degrees, if negative it will turn left, otherwise if positive it will turn right
 */ 
 void turn(int angle) {
  int onTime = (float) abs(5.2*((float)angle - 5.0));
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
   Serial.print("Right Hall\t");
  Serial.print(rightHall);
  Serial.print("Left Hall\t");
  Serial.print(leftHall);
  Serial.print("FreqDiff\t");
  Serial.println(freqDiff);
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

    bool atLowestLevel = false;
    /* FUNCTIONALITY 3: LET THERE BE LIGHT! */
    /* ************************************ */
    void f_light() {
  //also implements obstacle avoidance
  if(!atLowestLevel){
    f_obstacle();
  }
  //now it scans as well
  scan();
}

/*
Returns the angle where the light sensor picks up the least amount of light and moves in that direction
*/
int scan() {
  int angle = 0;
  //Set it really high
  int lowestLightLevel = 10000;
  for(int pos = 40; pos < 140; pos+=5) {
    myservo.write(pos);
    int currentLightLevel = analogRead(LIGHT_PIN); //Define this in header file
    //Get the angle where the light is the lowest
    if(abs(lowestLightLevel - currentLightLevel) > 30) {
      lowestLightLevel = currentLightLevel;
      angle = pos - 90;
    }
    delay(100);
  }
  Serial.println(lowestLightLevel);
  if(lowestLightLevel < 400) {
    analogWrite(E1,0);
    analogWrite(E2,0);

    digitalWrite(HALO_LED_PIN,HIGH);
    atLowestLevel = true;
    } else {
      atLowestLevel = false;
      turn(angle);
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
