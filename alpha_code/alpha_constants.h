/* Modes of the robot */
#define MODE_0 0
#define MODE_1 1
#define MODE_2 2
#define MODE_3 3
#define MODE_4 4

/* IO pins for Functionality 1 (obstacle) */
#define TRIG_PIN 8
#define ECHO_PIN 10
#define TEMP_PIN A5  // Analog IO pin for temperature sensor
#define SERVO_PIN 9
#define LED_PIN  12

/* constants for Functionality 2 (line follow) */
#define TRANS_IR 11             // transistor switch for IR sensors
#define RIGHT_INFRARED_PIN A0   //  blue
#define CENTER_INFRARED_PIN A1  // green
#define LEFT_INFRARED_PIN A2    // yellow

/* Constants for Functionality 3 (Drawing) */
const int PEN_SERVO_PIN = 11;
const int PEN_WRITE_POSITION = 1;
const int PEN_REST_POSITION = 50;
const int SMILEY_FACE_DELAY = 11000;
const int MOUTH_DELAY = 3000;
const float RADIUS_IN_METERS = 0.034;

/* decrease these constants to increase sensitivity */
#define ANGLE_THRESHOLD 1     
#define DRIFT_DAMPENING 40
#define LINE_BOUNCING 80

#define LIGHT_PIN A3
#define IR_PIN A4

#define SWITCH_ONE 2
#define SWITCH_TWO 3

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
const int THRESHOLD = 20;
const int HALO_LED_PIN = 13;

/**/
const long debounceDelay = 1000;
