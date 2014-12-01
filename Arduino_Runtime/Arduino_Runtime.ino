#include "Robotmap.h"

// #define DEBUG //Comment out to disable debug messages.
// #define TESTING //Comment out to disable testing.

Servo    m_north, m_west, m_south, m_east;
boolean  enabled = false;

Encoder  m_encoderNorth(kNorthEncoderA, kNorthEncoderB);
Encoder  m_encoderWest(kWestEncoderA, kWestEncoderB);
Encoder  m_encoderEast(kEastEncoderA, kEastEncoderA);
Encoder  m_encoderSouth(kSouthEncoderA, kSouthEncoderA);

LSM303   m_compass;
L3G      m_gyro;

void setup()
{
    Serial.begin(115200);
    initializeMotors();
    testCode();
    
    // Initialize Compass
    Wire.begin();
    m_compass.init();
    m_compass.enableDefault();
    m_compass.m_min = (LSM303::vector<int16_t>){-1748,  -1899,  -2515};
    m_compass.m_max = (LSM303::vector<int16_t>){+1909,  +1794,  +1076};

    //Initialize gyro
    if (!m_gyro.init())
    {
        Serial.println("Failed to autodetect gyro type!");
        while (1);
    }
    m_gyro.enableDefault();
}

void loop()
{
    m_compass.read();

    while (Serial.available() > 0)
    {
        String command = Serial.readStringUntil('\n');
        printToConsole(command);
        if (command == "n")
        {
            drive(0);
        }
        else if (command == "s")
        {
            drive(180);
        }
        else if (command == "nw")
        {
            drive(315);
        }
        else if (command == "ne")
        {
            drive(45);
        }
        else if (command == "e")
        {
            drive(90);
        }
        else if (command == "se")
        {
            drive(135);
        }
        else if (command == "sw")
        {
            drive(225);
        }
        else if (command == "w")
        {
            drive(270);
        }
        else if (command == "st")
        {
            stopDrive();
        }
        else if (command == "sp")
        {
            spin(1);
        }
        else if (command == "en")
        {
            enabled = true;
        }
        else if (command == "ds")
        {
            enabled = false;
        }
    }
    printDebuggingMessages();
}

void printToConsole(String message)
{
    String command = "cons:" + message;
    Serial.println(command);
}

void printDebuggingMessages()
{
    #if defined(DEBUG)
        float heading = m_compass.heading();
        Serial.print(m_encoderNorth.read());
        Serial.print('\t');
        Serial.print(m_encoderWest.read());
        Serial.print('\t');
        Serial.print(m_encoderSouth.read());
        Serial.print('\t');
        Serial.print(m_encoderEast.read());
        Serial.print('\t');
        Serial.print(heading);
        Serial.print('\t');
        m_gyro.read();
        Serial.print("G ");
        Serial.print("X: ");
        Serial.print((int)m_gyro.g.x);
        Serial.print(" Y: ");
        Serial.print((int)m_gyro.g.y);
        Serial.print(" Z: ");
        Serial.println((int)m_gyro.g.z);
        delay(100);
    #endif
}

void testCode()
{
    #if defined(TESTING)
        // Drive in a circle
        for (int i = 0; i < 360; i++)
        {
            drive(i);
            delay(7200 / 360);
        }
        decelerate();
    #endif
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
    String msg = "Driving at heading " + degreesFromNorth;
    printToConsole(msg);

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
    if (motorSpeed < 0)
    {
        motorSpeed += 180;
    }

    a.write(180 - motorSpeed);
    b.write(motorSpeed);
}

void decelerate()
{
    for (int i = driveSpeed; i >= 0; i--)
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
    return -(int)(sin(currentHeading * M_PI / 180) * motorSpeed) + 90;
}

int calcMotorSpeedEastWest(int motorSpeed)
{
    return (int)(cos(currentHeading * M_PI / 180) * motorSpeed) + 90;
}
