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

/*
my temporary storage











void decelerate(Servo a, Servo b, int timeInMillis)
{
	for(int i = driveSpeed; i<=90; i++)
	{
		driveMotors(a, b, i);
		delay(timeInMillis / driveSpeed);
	}
}






void drive(Direction dir)
{
	currentDirection = dir;
	switch(dir)
	{
		case kNorth:
			driveMotors(m_west, m_east, driveSpeed);
			break;
		case kWest:
			driveMotors(m_south, m_north, driveSpeed);
			break;
		case kSouth:
			driveMotors(m_east, m_west, driveSpeed);
			break;
		case kEast:
			driveMotors(m_north, m_south, driveSpeed);
			break;
		default:
			Serial.println("Not a direction.");
			break;
	}
}


void stopAllMotors()
{
	switch(currentDirection)
	{
		case kNorth:
			stopNorth();
			break;
		case kWest:
			stopWest();
			break;
		case kSouth:
			stopSouth();
			break;
		case kEast:
			stopEast();
			break;
		default:
			Serial.println("Not a direction.");
			break;
	}
}

void stopNorth()
{
	stopMotors(m_west, m_east);
}

void stopSouth()
{
	stopMotors(m_east, m_west);
}

void stopEast()
{
	stopMotors(m_north, m_south);
}

void stopWest()
{
	stopMotors(m_south, m_north);
}



void stopMotors(Servo a, Servo b)
{
	decelerate(a, b, decelerationTime);
}


*/