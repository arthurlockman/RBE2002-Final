#include "Servo.h"
#include "math.h"

enum MotorPins
{
	kNorthMotor = 4,
	kWestMotor = 5,
	kSouthMotor = 6,
	kEastMotor = 7
};

enum Direction
{
	kNorth,
	kWest,
	kSouth,
	kEast
};

// Number between 0 and 90
int driveSpeed = 50;

int decelerationTime = 500;

Direction currentDirection;

int currentHeading;
