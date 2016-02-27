/* Modes of the robot */
#define MODE_0 0
#define MODE_1 1
#define MODE_2 2

/* IO pins for Functionality 1 (obstacle) */
#define TRIG_PIN 10
#define ECHO_PIN 9
#define TEMP_PIN A5  // Analog IO pin for temperature sensor
#define SERVO_PIN 8

//Arduino Hall Sensor Pins
const int LEFT_HALL = 3;
const int RIGHT_HALL = 4;

//Arduino Motor Pins
const int E1 = 5;
const int M1 = 4;
const int E2 = 6;
const int M2 = 7;

//These value(s) for the drivetrain
const int MAX_SPEED = 255;
const int TURN_SPEED = 200;
const byte numMagnets = 5;
#define FREQ_RATIO 52;

#define LIGHT_THRESHOLD 200
