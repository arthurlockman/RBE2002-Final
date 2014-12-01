#include "Servo.h"
#include "math.h"
#include "Encoder.h"
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
	kNorthEncoderA = 0,
	kNorthEncoderB = 1,
	kWestEncoderA  = 2,
	kWestEncoderB  = 3,
	kSouthEncoderA = 5,
	kEastEncoderA  = 5
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
