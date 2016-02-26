#include <Servo.h>

//Servo Motor 
Servo myservo;  // Create a servo motor object
int pos = 0;    // Position of the servo motor 

//Arduino Motor PWM Speed Control：
int E1 = 5;  
int M1 = 4; 
int E2 = 6;                      
int M2 = 7;   

//Ultrasonic Range Finder Declarations
// Digital IO pins for the ultrasonic range finder
const int trigPin = 5;
const int echoPin = 6;
// Analog IO pin for temperature sensor
const int tempPin = A5;

// 38 milliseconds = 38000 microseconds
#define MICRO38MILLIS 38000

 
void setup() 
{ 
    //Pin inputs for the motor
    pinMode(M1, OUTPUT);   
    pinMode(M2, OUTPUT); 

    //Servo motor initialization
    Serial.begin(38400);
    myservo.attach(7);
    myservo.write(90);
    delay(3000);

    //Ultrasonic Range Finder input pins
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
} 
 
void loop() 
{ 

  //DC Motor control
  int value;
  for(value = 0 ; value <= 255; value+=5) 
  { 
    digitalWrite(M1,HIGH);   
    digitalWrite(M2, HIGH);       
    analogWrite(E1, value);   //PWM Speed Control
    analogWrite(E2, value);   //PWM Speed Control
    delay(30); 
  }  


  //Servo motor control 
  myservo.write(0);
  delay(1000);
  myservo.write(90);
  delay(1000);
  myservo.write(180);
  delay(1000);
  myservo.write(90);
  delay(1000);


  //Ultrasonic Range Finder 
  // send out a trigger signal from the ultrasonic device
  triggerSignal();
    
  // record the duration of the reflected signal
  float duration = pulseIn(echoPin, HIGH);

  float distanceCM;

  // 38 millis returned if no obstacle detected
  if(duration != MICRO38MILLIS){
      // convert the time duration to distance
      distanceCM = microsecondsToCM(duration);
  }
  else{
      // no obstacles detected
      distanceCM = -1;
  }
  Serial.println(distanceCM);

  // cycle period - 50 ms
  delayMicroseconds(50);
    
  delay(100);
}

void triggerSignal(){
    // write LOW first to ensure signal is sent out correctly
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
}

float microsecondsToCM(float microseconds){
    return float(microseconds * speedOfSound() / 10000.0 / 2.0); 
}

// calculate the speed of sound with temperature as one of its factors
float speedOfSound(){
    // read the temperature from the LH35 sensor
    float degrees_C = (5.0 * analogRead(tempPin) * 100.0 ) / 1024;
    // compute more accurate speed of sound by using current temp
    float speedOfSound = 331.5 + (0.6 * degrees_C);
    return speedOfSound;
}
