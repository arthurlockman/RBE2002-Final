#ifndef ROBOTMAP_H
#define ROBOTMAP_H

#include "Servo.h"
#include "math.h"
#include "SingleEncoder.h"
#include "TimerOne.h"
#include "Ultrasonic.h"
#include "LightSensor.h"
#include "FlameSensor.h"

// Uncomment the below line to use this axis definition:
// X axis pointing forward
// Y axis pointing to the right
// and Z axis pointing down.
// Positive pitch : nose up
// Positive roll : right wing down
// Positive yaw : clockwise
int SENSOR_SIGN[9] = {1, 1, 1, -1, -1, -1, 1, 1, 1}; //Correct directions x,y,z - gyro, accelerometer, magnetometer
// Uncomment the below line to use this axis definition:
// X axis pointing forward
// Y axis pointing to the left
// and Z axis pointing up.
// Positive pitch : nose down
// Positive roll : right wing down
// Positive yaw : counterclockwise
//int SENSOR_SIGN[9] = {1,-1,-1,-1,1,1,1,-1,-1}; //Correct directions x,y,z - gyro, accelerometer, magnetometer

// tested with Arduino Uno with ATmega328 and Arduino Duemilanove with ATMega168

// LSM303 accelerometer: 8 g sensitivity
// 3.9 mg/digit; 1 g = 256
#define GRAVITY 256  //this equivalent to 1G in the raw data coming from the accelerometer 

#define ToRad(x) ((x)*0.01745329252)  // *pi/180
#define ToDeg(x) ((x)*57.2957795131)  // *180/pi

// L3G4200D gyro: 2000 dps full scale
// 70 mdps/digit; 1 dps = 0.07
#define Gyro_Gain_X 0.07 //X axis Gyro gain
#define Gyro_Gain_Y 0.07 //Y axis Gyro gain
#define Gyro_Gain_Z 0.07 //Z axis Gyro gain
#define Gyro_Scaled_X(x) ((x)*ToRad(Gyro_Gain_X)) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Y(x) ((x)*ToRad(Gyro_Gain_Y)) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Z(x) ((x)*ToRad(Gyro_Gain_Z)) //Return the scaled ADC raw data of the gyro in radians for second

// LSM303 magnetometer calibration constants; use the Calibrate example from
// the Pololu LSM303 library to find the right values for your board
#define M_X_MIN -1813
#define M_Y_MIN -1939
#define M_Z_MIN -1933
#define M_X_MAX 1763
#define M_Y_MAX 1675
#define M_Z_MAX -150

#define Kp_ROLLPITCH 0.02
#define Ki_ROLLPITCH 0.00002
#define Kp_YAW 1.2
#define Ki_YAW 0.00002

/*For debugging purposes*/
//OUTPUTMODE=1 will print the corrected data,
//OUTPUTMODE=0 will print uncorrected data of the gyros (with drift)
#define OUTPUTMODE 1

//#define PRINT_DCM 0     //Will print the whole direction cosine matrix
#define PRINT_ANALOGS 0 //Will print the analog raw data
#define PRINT_EULER 1   //Will print the Euler angles Roll, Pitch and Yaw

#define STATUS_LED 13

float G_Dt = 0.02;  // Integration time (DCM algorithm)  We will run the integration loop at 50Hz if possible

long timer = 0; //general purpuse timer
long timer_old;
long timer24 = 0; //Second timer used to print values
int AN[6]; //array that stores the gyro and accelerometer data
int AN_OFFSET[6] = {0, 0, 0, 0, 0, 0}; //Array that stores the Offset of the sensors

int gyro_x;
int gyro_y;
int gyro_z;
int accel_x;
int accel_y;
int accel_z;
int magnetom_x;
int magnetom_y;
int magnetom_z;
float c_magnetom_x;
float c_magnetom_y;
float c_magnetom_z;
float MAG_Heading;

float Accel_Vector[3] = {0, 0, 0}; //Store the acceleration in a vector
float Gyro_Vector[3] = {0, 0, 0}; //Store the gyros turn rate in a vector
float Omega_Vector[3] = {0, 0, 0}; //Corrected Gyro_Vector data
float Omega_P[3] = {0, 0, 0}; //Omega Proportional correction
float Omega_I[3] = {0, 0, 0}; //Omega Integrator
float Omega[3] = {0, 0, 0};

// Euler angles
float roll;
float pitch;
float yaw;

float errorRollPitch[3] = {0, 0, 0};
float errorYaw[3] = {0, 0, 0};

unsigned int counter = 0;
byte gyro_sat = 0;

float DCM_Matrix[3][3] =
{
    {
        1, 0, 0
    }
    , {
        0, 1, 0
    }
    , {
        0, 0, 1
    }
};
float Update_Matrix[3][3] = {{0, 1, 2}, {3, 4, 5}, {6, 7, 8}}; //Gyros here


float Temporary_Matrix[3][3] =
{
    {
        0, 0, 0
    }
    , {
        0, 0, 0
    }
    , {
        0, 0, 0
    }
};

enum MotorPins
{
    kNorthMotor = 4,
    kWestMotor  = 5,
    kSouthMotor = 6,
    kEastMotor  = 7
};

enum Direction
{
    kNorth,
    kWest,
    kSouth,
    kEast
};

enum InterruptPins
{
    kNorthEncoderA = 2,
    kWestEncoderA  = 3,
    kNorthEncoderB = 12,
    kWestEncoderB  = 13,
    kSouthEncoderA = 18,
    kEastEncoderA  = 19
};

enum DigitalPins
{
    kFanNorth      = 47,
    kFanWest       = 49,
    kFanSouth      = 51,
    kFanEast       = 53
};

enum AnalogPins
{
    kLightSensorWest  = 0,
    kLightSensorSouth = 1,
    kLightSensorEast  = 2,
    kFlameSensorNorth = 3,
    kFlameSensorWest  = 4,
    kFlameSensorSouth = 5,
    kFlameSensorEast  = 6,
    kEastRangeOut     = 7,
    kSouthRangeOut    = 8,
    kWestRangeOut     = 9,
    kNorthRangeOut    = 10,
    kLightSensorNorth = 11
};

// Number between 0 and 90
int driveSpeed = 30;

float kProportionalCompass = 1.0;

int decelerationTime = 500;

Direction currentDirection;

float startOrientation;

//Drive storage values
int currentHeading;
int southDirection = 1;
int eastDirection = 1;

//Drive setpoints for PID
int northSetpoint;
int westSetpoint;
int southSetpoint;
int eastSetpoint;
volatile float northSpeed;
volatile float westSpeed;
volatile float southSpeed;
volatile float eastSpeed;
volatile long northLast;
volatile long westLast;
volatile long southLast;
volatile long eastLast;
static const int kQuadEncTicksPerRev = 360;
static const int kSingleEncTicksPerRev = 180;
static const float kDriveP = 0.1;
static const float kGyroCorrectionP = 0.000;
static const float kCompassCorrectionP = 2.0;

static const int kISRRate = 1000000;
static const int kISRMillis = kISRRate / 1000;

//Position things
volatile long northSouthPosition = 0;
volatile long eastWestPosition   = 0;
volatile float imuRotation = 0.0;
//Sensor Constants
static const int kLightSensorThresh = 500;
static const float kWallMaxdist = 9.0;
static const float kWallMinDist = 8.0;
/**
 * @brief Applies a deadband to a number.
 * @details Applies a deadband to a number. Useful
 * for joystick input and motor output.
 *
 * @param value The value to apply the deadband to.
 * @param deadbandUpper The upper limit of the deadband.
 * @param deadbandLower The lower limit of the deadband.
 * @param deadbandMid The middle value of the deadband.
 * @return Returns the deadbanded value.
 */
template <typename T>
inline T Deadband(T value, T deadbandUpper, T deadbandLower, T deadbandMid)
{
    if (value <= deadbandUpper && value >= deadbandLower)
        return deadbandMid;
    else
        return value;
};

inline float CalculateEncoderSpeed(int ticks, int dT, int ticksPerRev)
{
    // (revs per tick) * (ticks per unit time) * (unit time per minute)
    return (1.0 / (float)ticksPerRev) * ((float)ticks / (float)dT) * (60000);
};

#endif
