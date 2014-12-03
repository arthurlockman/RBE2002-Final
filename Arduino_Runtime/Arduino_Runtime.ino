#include "Robotmap.h"

// #define DEBUG //Comment out to disable debug messages.
// #define TESTING //Comment out to disable testing.
#define IMU //Comment out to disable IMU

Servo          m_north, m_west, m_south, m_east;
boolean        enabled = false;
SingleEncoder  m_encoderNorth(kNorthEncoderA, kSingleEncTicksPerRev);
SingleEncoder  m_encoderWest(kWestEncoderA, kSingleEncTicksPerRev);
SingleEncoder  m_encoderEast(kEastEncoderA, kSingleEncTicksPerRev);
SingleEncoder  m_encoderSouth(kSouthEncoderA, kSingleEncTicksPerRev);
LSM303         m_compass;
L3G            m_gyro;
Ultrasonic     m_rangeNorth(kNorthRangeOut, kNorthRangeIn);
Ultrasonic     m_rangeWest(kWestRangeOut, kWestRangeIn);
Ultrasonic     m_rangeSouth(kSouthRangeOut, kSouthRangeIn);
Ultrasonic     m_rangeEast(kEastRangeIn, kEastRangeOut);
LightSensor    m_lightNorth(kLightSensorNorth, kLightSensorThresh);
LightSensor    m_lightWest(kLightSensorWest, kLightSensorThresh);
LightSensor    m_lightSouth(kLightSensorSouth, kLightSensorThresh);
LightSensor    m_lightEast(kLightSensorEast, kLightSensorThresh);

void setup()
{
    Serial.begin(115200);
    initializeMotors();
    testCode();
    
    #if defined(IMU)
        // Initialize Compass
        Wire.begin();
        m_compass.init();
        m_compass.enableDefault();
        m_compass.m_min = (LSM303::vector<int16_t>){-1748,  -1899,  -2515};
        m_compass.m_max = (LSM303::vector<int16_t>){+1909,  +1794,  +1076};

        // Initialize gyro
        if (!m_gyro.init())
        {
            Serial.println("Failed to autodetect gyro type!");
            while (1);
        }
        m_gyro.enableDefault();
    #endif

    // Attach interrupt for speed-only encoders
    attachInterrupt(m_encoderEast.interruptPin, updateEastEncoder, CHANGE);
    attachInterrupt(m_encoderSouth.interruptPin, updateSouthEncoder, CHANGE);
    attachInterrupt(m_encoderNorth.interruptPin, updateNorthEncoder, CHANGE);
    attachInterrupt(m_encoderWest.interruptPin, updateWestEncoder, CHANGE);

    // Initialize fan control
    pinMode(kFanNorth, OUTPUT);
    pinMode(kFanWest, OUTPUT);
    pinMode(kFanSouth, OUTPUT);
    pinMode(kFanEast, OUTPUT);
    setFans(0);

    //Setup timer interrupts
    Timer1.initialize(kISRRate);
    Timer1.attachInterrupt(periodicUpdate);
}

void loop()
{
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
    imuRoutine();
}

void updateEastEncoder()
{
    m_encoderEast.update(eastDirection);
}

void updateSouthEncoder()
{
    m_encoderSouth.update(southDirection);
}

void updateNorthEncoder()
{
    m_encoderNorth.update(southDirection);
}

void updateWestEncoder()
{
    m_encoderWest.update(eastDirection);
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
        Serial.print(m_encoderNorth.speed());
        Serial.print('\t');
        Serial.print(m_encoderWest.speed());
        Serial.print('\t');
        Serial.print(m_encoderSouth.speed());
        Serial.print('\t');
        Serial.print(m_encoderEast.speed());
        Serial.print('\t');
        Serial.print(heading);
        Serial.print('\t');
        Serial.print("G ");
        Serial.print("X: ");
        Serial.print((int)m_gyro.g.x);
        Serial.print(" Y: ");
        Serial.print((int)m_gyro.g.y);
        Serial.print(" Z: ");
        Serial.print((int)m_gyro.g.z);
        Serial.println();
        delay(100);
    #endif
}

void testCode()
{
    #if defined(TESTING)
        drive(225);
    #endif
}

void imuRoutine()
{
    #if defined(IMU)
        m_compass.read();
        m_gyro.read();
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
    currentHeading = degreesFromNorth;

    int motorSpeedNorthSouth = calcMotorSpeedNorthSouth(driveSpeed);
    int motorSpeedEastWest = calcMotorSpeedEastWest(driveSpeed);

    //Set directions for encoders
    if (motorSpeedNorthSouth < 90) southDirection = 1;
    else southDirection = 0;
    if (motorSpeedEastWest < 90) eastDirection = 0;
    else eastDirection = 1;

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

    //float adjustedValue = adjust();

    a.write(180 - motorSpeed);
    b.write(motorSpeed);
}

void decelerate()
{
    for (int i = driveSpeed; i >= 0; i--)
    {
        int motorSpeedNorthSouth = calcMotorSpeedNorthSouth(i);
        int motorSpeedEastWest =  calcMotorSpeedEastWest(i);

        if (motorSpeedNorthSouth < 90) southDirection = 0;
        else southDirection = 1;
        if (motorSpeedEastWest < 90) eastDirection = 0;
        else eastDirection = 1;

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

float adjust(float desiredHeading)
{
    float nowHeading = m_compass.heading();
    return kProportionalCompass * (desiredHeading - nowHeading);
}

void setFans(int fan)
{
    switch (fan)
    {
        case 0:
            digitalWrite(kFanNorth, LOW);
            digitalWrite(kFanWest, LOW);
            digitalWrite(kFanSouth, LOW);
            digitalWrite(kFanEast, LOW);
            break;
        case 1:
            digitalWrite(kFanNorth, HIGH);
            digitalWrite(kFanWest, LOW);
            digitalWrite(kFanSouth, LOW);
            digitalWrite(kFanEast, LOW);
            break;
        case 2:
            digitalWrite(kFanNorth, LOW);
            digitalWrite(kFanWest, HIGH);
            digitalWrite(kFanSouth, LOW);
            digitalWrite(kFanEast, LOW);
            break;
        case 3:
            digitalWrite(kFanNorth, LOW);
            digitalWrite(kFanWest, LOW);
            digitalWrite(kFanSouth, HIGH);
            digitalWrite(kFanEast, LOW);
            break;
        case 4:
            digitalWrite(kFanNorth, LOW);
            digitalWrite(kFanWest, LOW);
            digitalWrite(kFanSouth, LOW);
            digitalWrite(kFanEast, HIGH);
            break;
    }
}

void periodicUpdate()
{
    //Handle ISR business
}
