#include "Servo.h"
#include "math.h"
#include "Encoder.h"
#include "LSM303.h"
#include "L3G.h"
#include <Wire.h>

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

int currentHeading;
