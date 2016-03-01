/* Modes of the robot */
#define MODE_0 0
#define MODE_1 1
#define MODE_2 2
#define MODE_3 3

/* IO pins for Functionality 1 (obstacle) */
#define TRIG_PIN 9
#define ECHO_PIN 10
#define TEMP_PIN A5  // Analog IO pin for temperature sensor
#define SERVO_PIN 8

/* constants for Functionality 2 (line follow) */
#define RIGHT_INFRARED_PIN A0   //  blue
#define CENTER_INFRARED_PIN A1  // green
#define LEFT_INFRARED_PIN A2    // yellow
/* decrease these constants to increase sensitivity */
#define ANGLE_THRESHOLD 2     
#define DRIFT_DAMPENING 40
#define LINE_BOUNCING 80

#define LIGHT_PIN A3

/* Arduino Hall Sensor Pins */
const int LEFT_HALL = 2;
const int RIGHT_HALL = 3;

//Arduino Motor Pins
const int E1 = 5;
const int M1 = 4;
const int E2 = 6;
const int M2 = 7;

/* These value(s) for the drivetrain */
const int MAX_SPEED = 255;
const int TURN_SPEED = 200;
const int numMagnets = 5;
const int FREQ_RATIO = 52;
const int LIGHT_THRESHOLD = 200;
const int THRESHOLD = 40;


