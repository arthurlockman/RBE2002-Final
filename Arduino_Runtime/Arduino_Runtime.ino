#include "Robotmap.h"

#define DEBUG //Comment out to disable debug messages.
// #define TESTING //Comment out to disable testing.
#define IMU //Comment out to disable IMU
// #define DRIVE_PID //Comment out to disable Drive PID

Servo          m_north, m_west, m_south, m_east;
int            m_loopState = 0;
int            m_gyroZero = 0;
boolean        enabled = false;
boolean        m_stopped = true;
SingleEncoder  m_encoderNorth(kNorthEncoderA, kSingleEncTicksPerRev);
SingleEncoder  m_encoderWest(kWestEncoderA, kSingleEncTicksPerRev);
SingleEncoder  m_encoderEast(kEastEncoderA, kSingleEncTicksPerRev);
SingleEncoder  m_encoderSouth(kSouthEncoderA, kSingleEncTicksPerRev);
LSM303         m_compass;
L3G            m_gyro;
Ultrasonic     m_rangeNorth(kNorthRangeIn, kNorthRangeOut);
Ultrasonic     m_rangeWest(kWestRangeIn, kWestRangeOut);
Ultrasonic     m_rangeSouth(kSouthRangeIn, kSouthRangeOut);
Ultrasonic     m_rangeEast(kEastRangeIn, kEastRangeOut);
LightSensor    m_lightNorth(kLightSensorNorth, kLightSensorThresh);
LightSensor    m_lightWest(kLightSensorWest, kLightSensorThresh);
LightSensor    m_lightSouth(kLightSensorSouth, kLightSensorThresh);
LightSensor    m_lightEast(kLightSensorEast, kLightSensorThresh);
FlameSensor    m_flameNorth(kFlameSensorNorth);
FlameSensor    m_flameWest(kFlameSensorWest);
FlameSensor    m_flameSouth(kFlameSensorSouth);
FlameSensor    m_flameEast(kFlameSensorEast);

void setup()
{
    Serial.begin(115200);
    initializeMotors();
    testCode();

#if defined(IMU)
    // Initialize Compass
    Serial.println("Init IMU");
    Wire.begin();
    m_compass.init();
    m_compass.enableDefault();
    m_compass.m_min = (LSM303::vector<int16_t>)
    {
        -828,  -1899,  -2080
    };
    m_compass.m_max = (LSM303::vector<int16_t>)
    {
        1001,  1794,  -253
    };

    // Initialize gyro
    if (!m_gyro.init())
    {
        Serial.println("Failed to autodetect gyro type!");
        while (1);
    }
    m_gyro.enableDefault();

    Serial.println("Zeroing Gyro...");
    m_compass.read();
    for (int i = 0; i < 100; i++)
    {
        m_compass.read();
        m_gyro.read();
        m_gyroZero += m_gyro.g.y;
        startOrientation += getCurrentOrientation();
    }
    m_gyroZero = (int)((float)m_gyroZero / 100.0);
    startOrientation = startOrientation / 100.0;
    Serial.print("Gyro Y Zero: ");
    Serial.println(m_gyroZero);
    Serial.print("Starting orientation: ");
    Serial.println(startOrientation);
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

#if defined(DRIVE_PID)
    //Setup timer interrupts
    Timer1.initialize(kISRRate);
    Timer1.attachInterrupt(periodicUpdate);
#endif
}

void loop()
{
    // drive(45);
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
        else if (command == "fn")
        {
            setFans(1);
        }
        else if (command == "fw")
        {
            setFans(2);
        }
        else if (command == "fs")
        {
            setFans(3);
        }
        else if (command == "fe")
        {
            setFans(4);
        }
        else if (command == "fa")
        {
            setFans(5);
        }
        else if (command == "fo")
        {
            setFans(0);
        }
    }
    printDebuggingMessages();
    imuRoutine();
    updateDrive();
}

int readGyroY()
{
    if (m_gyro.g.y > (m_gyroZero + 50) || m_gyro.g.y < (m_gyroZero - 50))
    {
        return m_gyro.g.y - m_gyroZero;
    }
    else
    {
        return 0;
    }
}

int followWall(int side, int dir)
{
    int driveDir;
    float wallDist;
    float wall2Dist;

    switch (side)
    {
    case 1:
        wallDist = m_rangeNorth.distance();
        wall2Dist = (dir == 1) ? m_rangeEast.distance() : m_rangeWest.distance();
        if (wall2Dist < 6.0)
        {
            stopDrive();
            return 1;
        }
        else if (wallDist > 6.0 && dir == 1)
        {
            drive(45);
        }
        else if (wallDist > 6.0 && dir == 0)
        {
            drive(315);
        }
        else if (wallDist < 3.0 && dir == 1)
        {
            drive(135);
        }
        else if (wallDist < 3.0 && dir == 0)
        {
            drive(225);
        }
        else if (dir == 1)
        {
            drive(90);
        }
        else if (dir == 0)
        {
            drive(270);
        }
        break;
    case 2:
        wallDist = m_rangeWest.distance();
        wall2Dist = (dir == 1) ? m_rangeNorth.distance() : m_rangeSouth.distance();
        if (wall2Dist < 6.0)
        {
            stopDrive();
            return 1;
        }
        else if (wallDist > 6.0 && dir == 1)
        {
            drive(315);
        }
        else if (wallDist > 6.0 && dir == 0)
        {
            drive(225);
        }
        else if (wallDist < 3.0 && dir == 1)
        {
            drive(45);
        }
        else if (wallDist < 3.0 && dir == 0)
        {
            drive(135);
        }
        else if (dir == 1)
        {
            drive(0);
        }
        else if (dir == 0)
        {
            drive(180);
        }
        break;
    case 3:
        wallDist = m_rangeSouth.distance();
        wall2Dist = (dir == 1) ? m_rangeWest.distance() : m_rangeEast.distance();
        if (wall2Dist < 6.0)
        {
            stopDrive();
            return 1;
        }
        else if (wallDist > 6.0 && dir == 1)
        {
            drive(225);
        }
        else if (wallDist > 6.0 && dir == 0)
        {
            drive(135);
        }
        else if (wallDist < 3.0 && dir == 1)
        {
            drive(315);
        }
        else if (wallDist < 3.0 && dir == 0)
        {
            drive(45);
        }
        else if (dir == 1)
        {
            drive(270);
        }
        else if (dir == 0)
        {
            drive(90);
        }
        break;
    case 4:
        wallDist = m_rangeEast.distance();
        wall2Dist = (dir == 1) ? m_rangeSouth.distance() : m_rangeNorth.distance();
        if (wall2Dist < 6.0)
        {
            stopDrive();
            return 1;
        }
        else if (wallDist > 6.0 && dir == 1)
        {
            drive(135);
        }
        else if (wallDist > 6.0 && dir == 0)
        {
            drive(45);
        }
        else if (wallDist < 3.0 && dir == 1)
        {
            drive(225);
        }
        else if (wallDist < 3.0 && dir == 0)
        {
            drive(315);
        }
        else if (dir == 1)
        {
            drive(180);
        }
        else if (dir == 0)
        {
            drive(0);
        }
        break;
    }
    return 0;
}

/**
 * @brief Updates the east encoder.
 * @details Updates the east encoder with the proper
 * direction variable.
 */
void updateEastEncoder()
{
    m_encoderEast.update(eastDirection);
}

/**
 * @brief Updates the south encoder.
 * @details Updates the south encoder with the proper
 * direction variable.
 */
void updateSouthEncoder()
{
    m_encoderSouth.update(southDirection);
}

/**
 * @brief Updates the north encoder.
 * @details Updates the north encoder with the proper
 * direction variable.
 */
void updateNorthEncoder()
{
    m_encoderNorth.update(southDirection);
}

/**
 * @brief Updates the west encoder.
 * @details Updates the west encoder with the proper
 * direction variable.
 */
void updateWestEncoder()
{
    m_encoderWest.update(eastDirection);
}

/**
 * @brief Prints to console.
 * @details Prints a message to the control
 * server console. This shows up on the web controller.
 *
 * @param message The message to send.
 */
void printToConsole(String message)
{
    String command = "cons:" + message;
    Serial.println(command);
}

/**
 * @brief Prints debug messages.
 * @details Prints messages useful for debugging. Enabled
 * by uncommenting the #DEBUG line at the top of this file.
 */
void printDebuggingMessages()
{
#if defined(DEBUG)
    Serial.print(m_encoderNorth.speed());
    Serial.print('\t');
    Serial.print(m_encoderWest.speed());
    Serial.print('\t');
    Serial.print(m_encoderSouth.speed());
    Serial.print('\t');
    Serial.print(m_encoderEast.speed());
    Serial.print('\t');
    Serial.print(getCurrentOrientation());
    Serial.print('\t');
    Serial.print("G ");
    Serial.print("X: ");
    Serial.print((int)m_gyro.g.x);
    Serial.print(" Y: ");
    Serial.print(readGyroY());
    Serial.print(" Z: ");
    Serial.print((int)m_gyro.g.z);
    Serial.println();
    delay(100);
#endif
}

/**
 * @brief Runs tests.
 * @details This method can be used to run code tests. To
 * use it, uncomment the #TESTING block at the top of this
 * file. This enables the testing block to be run each loop.
 */
void testCode()
{
#if defined(TESTING)

#endif
}

float getCurrentOrientation()
{
    float accum = 0;
    for (int i = 0; i < 10; i++)
    {
        float x_value = m_compass.m.x;
        float z_value = m_compass.m.z;

        float x_corrected = (x_value - m_compass.m_min.x) * (180) / (m_compass.m_max.x - m_compass.m_min.x) - 90;
        float z_corrected = (z_value - m_compass.m_min.z) * (180) / (m_compass.m_max.z - m_compass.m_min.z) - 90;

        float raw_orientation = atan2(z_corrected, x_corrected) * (180 / M_PI);

        accum += (raw_orientation < 0) ? raw_orientation + 360.0 : raw_orientation;
    }  
    return accum / 10.0;
}

/**
 * @brief Handles IMU readings.
 * @details This method handles the readings for the
 * IMU. This ensures that the IMU is only read once per
 * loop.
 */
void imuRoutine()
{
#if defined(IMU)
    m_compass.read();
    m_gyro.read();
#endif
}

/**
 * @brief Initializes motors.
 * @details Handles motor startup. Attaches all servos
 * with proper constants.
 */
void initializeMotors()
{
    m_north.attach(kNorthMotor, 1000, 2000);
    m_west.attach(kWestMotor, 1000, 2000);
    m_south.attach(kSouthMotor, 1000, 2000);
    m_east.attach(kEastMotor, 1000, 2000);
}

/**
 * @brief Drives the robot.
 * @details This method drives the robot at a heading as
 * described in degrees from north. For instance, entering
 * a value of 90 causes the robot to drive due east, while
 * 135 causes it to drive south-east.
 *
 * @param degreesFromNorth The heading in degrees from north.
 */
void drive(int degreesFromNorth)
{
    m_stopped = false;
#if defined(DRIVE_PID)
    //Do PID Drive
    currentHeading = degreesFromNorth;

    int motorSpeedNorthSouth = calcMotorSpeedNorthSouth(driveSpeed);
    int motorSpeedEastWest = calcMotorSpeedEastWest(driveSpeed);

    //Set directions for encoders
    if (motorSpeedNorthSouth < 90) southDirection = 1;
    else southDirection = 0;
    if (motorSpeedEastWest < 90) eastDirection = 0;
    else eastDirection = 1;

    northSetpoint = motorSpeedNorthSouth;
    westSetpoint  = motorSpeedEastWest;
    southSetpoint = motorSpeedNorthSouth;
    eastSetpoint  = motorSpeedEastWest;
#else
    currentHeading = degreesFromNorth;

    int motorSpeedNorthSouth = calcMotorSpeedNorthSouth(driveSpeed);
    int motorSpeedEastWest = calcMotorSpeedEastWest(driveSpeed);

    //Set directions for encoders
    if (motorSpeedNorthSouth < 90) southDirection = 1;
    else southDirection = 0;
    if (motorSpeedEastWest < 90) eastDirection = 0;
    else eastDirection = 1;

    northSetpoint = motorSpeedNorthSouth;
    southSetpoint = motorSpeedNorthSouth;
    eastSetpoint  = motorSpeedEastWest;
    westSetpoint  = motorSpeedEastWest;
    // driveMotors(m_north, m_south, motorSpeedNorthSouth);
    // driveMotors(m_east, m_west, motorSpeedEastWest);
#endif
}

/**
 * @brief Handles PID drive update.
 * @details This method updates the motor speeds according to
 * the values calculated in the PID service routine.
 */
void updateDrive()
{
#if defined(DRIVE_PID)
    if (!m_stopped)
    {
        m_north.write(180 - ((northSpeed < 0) ? northSpeed + 180 : northSpeed));
        m_south.write(((southSpeed < 0) ? southSpeed + 180 : southSpeed));
        m_east.write(180 - ((eastSpeed < 0) ? eastSpeed + 180 : eastSpeed));
        m_west.write(((westSpeed < 0) ? westSpeed + 180 : westSpeed));
    }
    else
    {
        m_north.write(90);
        m_west.write(90);
        m_east.write(90);
        m_south.write(90);
    }
#else
    int gyroRate = readGyroY();
    if (gyroRate != 0) //turning CCW
    {
        northSetpoint += (gyroRate * kGyroCorrectionP);
        westSetpoint  += -(gyroRate * kGyroCorrectionP);
        southSetpoint += -(gyroRate * kGyroCorrectionP);
        eastSetpoint  += (gyroRate * kGyroCorrectionP);
    }
    if (!m_stopped)
    {
        m_north.write(180 - ((northSetpoint < 0) ? northSetpoint + 180 : northSetpoint));
        m_south.write(((southSetpoint < 0) ? southSetpoint + 180 : southSetpoint));
        m_east.write(180 - ((eastSetpoint < 0) ? eastSetpoint + 180 : eastSetpoint));
        m_west.write(((westSetpoint < 0) ? westSetpoint + 180 : westSetpoint));
    }
    else
    {
        m_north.write(90);
        m_west.write(90);
        m_east.write(90);
        m_south.write(90);
    }
#endif
}

/**
 * @brief Stop the drivetrain.
 * @details Stops all motors regardless of drive mode.
 */
void stopDrive()
{
#if defined(DRIVE_PID)
    northSetpoint = 0;
    westSetpoint = 0;
    eastSetpoint = 0;
    southSetpoint = 0;
    m_stopped = true;
#else
    m_north.write(90);
    m_west.write(90);
    m_east.write(90);
    m_south.write(90);
#endif
}

/**
 * @brief Spin the robot
 * @details Spins the robot in a desired
 * direction at a fixed speed.
 *
 * @param left If set to 1, robot turns
 * left, else turns right.
 */
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

/**
 * @brief Drive motors.
 * @details This method drives the robot motors. It takes
 * in two servo objects and writes the correct speeds to
 * them. This assumes that the two motors are on opposite
 * sides of the robot and are in direct opposition, like
 * using north as A and south as B.
 *
 * @param a The first motor.
 * @param b The second motor.
 * @param motorSpeed The desired speed to set them at.
 */
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

/**
 * @brief Calculates north-south speeds.
 * @details Calculates the motor speeds for the north-south
 * motors based on the current heading and the desired
 * speed.
 *
 * @param motorSpeed The desired motor speed.
 * @return The calculated north-south speed.
 */
int calcMotorSpeedNorthSouth(int motorSpeed)
{
    return -(int)(sin(currentHeading * M_PI / 180) * motorSpeed) + 90;
}

/**
 * @brief Calculates east-west speeds.
 * @details Calculates the motor speeds for the east-west
 * motors based on the current heading and the desired
 * speed.
 *
 * @param motorSpeed The desired motor speed.
 * @return The calculated east-west speed.
 */
int calcMotorSpeedEastWest(int motorSpeed)
{
    return (int)(cos(currentHeading * M_PI / 180) * motorSpeed) + 90;
}

float adjust(float desiredHeading)
{
    float nowHeading = m_compass.heading();
    return kProportionalCompass * (desiredHeading - nowHeading);
}

/**
 * @brief Enables fans.
 * @details This function handles turning on and off the
 * flame extinguisher fans on the robot. Sending a value of
 * 0 turns all fans off, and 5 turns all on. Values 1-4 set
 * fans counter-clockwise starting at North going to East.
 *
 * @param fan The fan control value.
 */
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
    case 5:
        digitalWrite(kFanNorth, HIGH);
        digitalWrite(kFanWest, HIGH);
        digitalWrite(kFanSouth, HIGH);
        digitalWrite(kFanEast, HIGH);
        break;
    }
}

/**
 * @brief ISR Routine
 * @details This function handles updating motor PID
 * control variables, as well as other ISR business.
 */
void periodicUpdate()
{
    float northError = northSetpoint - m_encoderNorth.speed();
    float westError  = westSetpoint  - m_encoderWest.speed();
    float southError = southSetpoint - m_encoderSouth.speed();
    float eastError  = eastSetpoint  - m_encoderEast.speed();
    if (!m_stopped)
    {
        northSpeed = northError * kDriveP + m_encoderNorth.speed();
        westSpeed  = westError  * kDriveP + m_encoderWest.speed();
        southSpeed = southError * kDriveP + m_encoderSouth.speed();
        eastSpeed  = eastError  * kDriveP + m_encoderEast.speed();
    }
    else
    {
        northSpeed = 90;
        southSpeed = 90;
        eastSpeed  = 90;
        westSpeed  = 90;
    }
}
