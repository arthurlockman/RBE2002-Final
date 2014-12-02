#ifndef ROBOTMAP_H
#define ROBOTMAP_H

#include "Servo.h"
#include "math.h"
#include "Encoder.h"
#include "LSM303.h"
#include "L3G.h"
#include "SingleEncoder.h"
#include <Wire.h>
#include "TimerOne.h"
#include "Ultrasonic.h"
#include "LightSensor.h"

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
	kNorthRangeOut = 1,
	kNorthRangeIn  = 2,
	kWestRangeOut  = 3,
	kWestRangeIn   = 4,
	kSouthRangeOut = 5,
	kSouthRangeIn  = 6,
	kEastRangeOut  = 7,
	kEastRangeIn   = 8
};

enum AnalogPins
{
	kLightSensorNorth = 0,
	kLightSensorWest  = 1,
	kLightSensorSouth = 2,
	kLightSensorEast  = 3
};

// Number between 0 and 90
int driveSpeed = 50;

int decelerationTime = 500;

Direction currentDirection;

//Drive storage values
int currentHeading;
int southDirection = 1;
int eastDirection = 1;

//Drive setpoints for PID
int northSetpoint;
int westSetpoint;
int southSetpoint;
int eastSetpoint;
volatile int northSpeed;
volatile int westSpeed;
volatile int southSpeed;
volatile int eastSpeed;
static const int kQuadEncTicksPerRev = 360;
static const int kSingleEncTicksPerRev = 360;
static const int kEncoderISRRate = 10000;
static const int kEncoderISRMillis = kEncoderISRRate / 1000;

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
	return (1.0 / (float)ticksPerRev) * ((float)ticks / (float)dT) * (60.0);
};

#endif
