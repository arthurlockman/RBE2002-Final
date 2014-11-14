#include <Robotmap.h>

Servo m_north, m_west, m_south, m_east;

void setup()
{

	Serial.begin(9600);

	initializeMotors();

	testCode();
}

void loop()
{

}

void testCode()
{

	// Drive in a circle
	for(int i=0; i<360;i++)
	{
		drive(i);
		delay(7200/360);
	}

	decelerate();
	
	
}

void initializeMotors()
{
	m_north.attach(kNorthMotor, 1000, 2000);
	m_west.attach(kWestMotor, 1000, 2000);
	m_south.attach(kSouthMotor, 1000, 2000);
	m_east.attach(kEastMotor, 1000, 2000);
}


void drive(int degreesFromNorth)
{
	currentHeading = degreesFromNorth;

	int motorSpeedNorthSouth = calcMotorSpeedNorthSouth(driveSpeed);
	int motorSpeedEastWest = calcMotorSpeedEastWest(driveSpeed);

	driveMotors(m_north, m_south, motorSpeedNorthSouth);
	driveMotors(m_east, m_west, motorSpeedEastWest);
}

void driveMotors(Servo a, Servo b, int motorSpeed)
{
	if(motorSpeed < 0)
	{
		motorSpeed += 180;
	}

	a.write(180-motorSpeed);
	b.write(motorSpeed);
}

void decelerate()
{
	for(int i = driveSpeed; i>=0; i--)
	{
		int motorSpeedNorthSouth = calcMotorSpeedNorthSouth(i);
		int motorSpeedEastWest =  calcMotorSpeedEastWest(i);

		driveMotors(m_north, m_south, motorSpeedNorthSouth);
		driveMotors(m_east, m_west, motorSpeedEastWest);

		delay(decelerationTime / driveSpeed);
	}
}

int calcMotorSpeedNorthSouth(int motorSpeed)
{
	return -(int)(sin(currentHeading*M_PI/180) * motorSpeed) + 90;
}

int calcMotorSpeedEastWest(int motorSpeed)
{
	return (int)(cos(currentHeading*M_PI/180) * motorSpeed) + 90;
}








