#ifndef ROBOTMAP_H
#define ROBOTMAP_H

#include "Servo.h"
#include "math.h"
#include "LSM303.h"
#include "L3G.h"
#include "SingleEncoder.h"
#include <Wire.h>
#include "TimerOne.h"
#include "Ultrasonic.h"
#include "LightSensor.h"
#include "FlameSensor.h"

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
	kNorthRangeOut = 22,
	kNorthRangeIn  = 23,
	kWestRangeOut  = 24,
	kWestRangeIn   = 25,
	kSouthRangeOut = 26,
	kSouthRangeIn  = 27,
	kEastRangeOut  = 28,
	kEastRangeIn   = 29,
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
	kFlameSensorNorth = 7,
	kFlameSensorWest  = 8,
	kFlameSensorSouth = 9,
	kFlameSensorEast  = 10,
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
static const float kCompassCorrectionP = 1.0;

static const int kISRRate = 1000000;
static const int kISRMillis = kISRRate / 1000;

//Position things
volatile long northSouthPosition = 0;
volatile long eastWestPosition   = 0;

//Sensor Constants
static const int kLightSensorThresh = 500;

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
