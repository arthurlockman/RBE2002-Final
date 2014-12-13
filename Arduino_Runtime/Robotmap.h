#ifndef ROBOTMAP_H
#define ROBOTMAP_H

#include "Servo.h"
#include "math.h"
#include "SingleEncoder.h"
#include "TimerOne.h"
#include "Ultrasonic.h"
#include "LightSensor.h"
#include "FlameSensor.h"
#include "StackList.h"

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

enum NavigationState
{
    kNavigationStart,
    kNavigationFollowWall,
    kNavigationDecideNext,
    kNavigationHomeOnCandle,
    kNavigationExtinguishFlame
};

struct FollowCommand
{
    int side; //north, south, east, west
    int direction; //left or right
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
static const float kCompassCorrectionP = 1.5;

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

//Navigation Constants
StackList<FollowCommand> m_navStack;

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

inline int greatestIndex(double numargs, ...)
{
    va_list listPointer;
    va_start(listPointer, numargs);
    double greatest = 0;
    int index = 0;
    for(int i = 0 ; i < (int)numargs; i++)
    {
        double arg = va_arg(listPointer, double);
        if (arg > greatest) { index = i + 1; greatest = arg; }
    }
    va_end(listPointer);
    return index;
};

inline int leastIndex(double numargs, ...)
{
    va_list listPointer;
    va_start(listPointer, numargs);
    double least = 1000000;
    int index = 0;
    for(int i = 0 ; i < (int)numargs; i++)
    {
        double arg = va_arg(listPointer, double);
        if (arg < least) { index = i + 1; least = arg; }
    }
    va_end(listPointer);
    return index;
};

inline float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
