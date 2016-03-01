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

#define rightInfraredPin A0   //  blue
#define centerInfraredPin A1  // green
#define leftInfraredPin A2    // yellow
#define ANGLE_THRESHOLD 2     // rotate after this
#define LIGHT_PIN A3

//Arduino Hall Sensor Pins
const int LEFT_HALL = 2;
const int RIGHT_HALL = 3;

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

const int THRESHOLD = 40;

// decrease dampening to increase sensitivity
const int driftDampening = 40;
const int bouncing = 80;
