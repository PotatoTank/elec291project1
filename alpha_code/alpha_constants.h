/* Modes of the robot */
#define MODE_0 0
#define MODE_1 1
#define MODE_2 2

/* IO pins for Functionality 1 (obstacle) */
#define TRIG_PIN 10
#define ECHO_PIN 9
#define TEMP_PIN A5  // Analog IO pin for temperature sensor
#define SERVO_PIN 8


#define RIGHT_INFRARED_PIN A0
#define CENTER_INFRARED_PIN A1
#define LEFT_INFRARED_PIN A2

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
const int numMagnets = 5;
const int FREQ_RATIO = 52;
const int LIGHT_THRESHOLD = 200;
const int DRIFT_DAMPENING = 10;

const int THRESHOLD = 30;
