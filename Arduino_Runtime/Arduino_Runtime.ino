#include "Robotmap.h"

Servo m_north, m_west, m_south, m_east;

void setup()
{

	Serial.begin(115200);

	initializeMotors();

	//testCode();
}

void loop()
{
  while(Serial.available() > 0)
  {
    String command = Serial.readStringUntil('\n');
    Serial.print("Got command: ");
    Serial.println(command);
    if (command == "n") { drive(0); }
    else if (command == "s") { drive(180); }
    else if (command == "nw") { drive(315); }
    else if (command == "ne") { drive(45); }
    else if (command == "e") { drive(90); }
    else if (command == "se") { drive(135); }
    else if (command == "sw") { drive(225); }
    else if (command == "w") { drive(270); }
    else if (command == "st") { stopDrive(); }
    else if (command == "sp") { spin(1); }
  }
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

void stopDrive()
{
  m_north.write(90);
  m_west.write(90);
  m_east.write(90);
  m_south.write(90);
}

void spin(int left)
{
    if (left == 1)
    {
        m_north.write(40);
        m_east.write(40);
        m_west.write(40);
        m_south.write(40);
    }
    else if (left == 0)
    {
        m_north.write(140);
        m_east.write(140);
        m_west.write(140);
        m_south.write(140);
    }
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








