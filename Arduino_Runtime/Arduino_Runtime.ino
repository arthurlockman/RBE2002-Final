#include "Robotmap.h"

// #define DEBUG //Comment out to disable debug messages.
// #define TESTING //Comment out to disable testing.
// #define IMU //Comment out to disable IMU
#define OPENIMU //Use OpenIMU Library
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

    // Initialize m_gyro
    if (!m_gyro.init())
    {
        Serial.println("Failed to autodetect m_gyro type!");
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
#if defined(OPENIMU)
    Wire.begin();
    TWBR = ((F_CPU / 400000) - 16) / 2;//set the I2C speed to 400KHz
    IMUinit();
    printTimer = millis();
    timer = micros();
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
#if defined(OPENIMU)
    if (micros() - timer >= 5000)
    {
        G_Dt = (micros() - timer) / 1000000.0;
        timer = micros();
        m_compass.read();
        floatMagX = ((float)m_compass.m.x - compassXMin) * inverseXRange - 1.0;
        floatMagY = ((float)m_compass.m.y - compassYMin) * inverseYRange - 1.0;
        floatMagZ = ((float)m_compass.m.z - compassZMin) * inverseZRange - 1.0;
        Smoothing(&m_compass.a.x, &smoothAccX);
        Smoothing(&m_compass.a.y, &smoothAccY);
        Smoothing(&m_compass.a.z, &smoothAccZ);
        accToFilterX = smoothAccX;
        accToFilterY = smoothAccY;
        accToFilterZ = smoothAccZ;
        m_gyro.read();
        AHRSupdate(&G_Dt);
    }

    if (millis() - printTimer > 50)
    {
        printTimer = millis();
        GetEuler();
        Serial.print(printTimer);
        Serial.print(",");
        Serial.print(pitch);
        Serial.print(",");
        Serial.print(roll);
        Serial.print(",");
        Serial.println(yaw);
    }
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
    float headingError = getCurrentOrientation() - startOrientation;
    if (gyroRate != 0) //turning CCW
    {
        northSetpoint += (gyroRate * kGyroCorrectionP) + (int)(headingError * kCompassCorrectionP);
        westSetpoint  += -(gyroRate * kGyroCorrectionP) - (int)(headingError * kCompassCorrectionP);
        southSetpoint += -(gyroRate * kGyroCorrectionP) - (int)(headingError * kCompassCorrectionP);
        eastSetpoint  += (gyroRate * kGyroCorrectionP) + (int)(headingError * kCompassCorrectionP);
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

//OpenIMU Methods
void IMUinit()
{
    //init devices
    m_compass.init();
    m_gyro.init();

    m_gyro.writeReg(L3G_CTRL_REG1, 0xCF);
    m_gyro.writeReg(L3G_CTRL_REG2, 0x00);
    m_gyro.writeReg(L3G_CTRL_REG3, 0x00);
    m_gyro.writeReg(L3G_CTRL_REG4, 0x20); //
    m_gyro.writeReg(L3G_CTRL_REG5, 0x02);

    m_compass.writeAccReg(LSM303::CTRL_REG1_A, 0x77);//400hz all enabled
    m_compass.writeAccReg(LSM303::CTRL_REG4_A, 0x20);//+/-8g 4mg/LSB

    m_compass.writeMagReg(LSM303::CRA_REG_M, 0x1C);
    m_compass.writeMagReg(LSM303::CRB_REG_M, 0x60);
    m_compass.writeMagReg(LSM303::MR_REG_M, 0x00);

    beta = betaDef;
    //calculate initial quaternion
    //take an average of the m_gyro readings to remove the bias

    for (int i = 0; i < 500; i++)
    {
        m_gyro.read();
        m_compass.read();
        Smoothing(&m_compass.a.x, &smoothAccX);
        Smoothing(&m_compass.a.y, &smoothAccY);
        Smoothing(&m_compass.a.z, &smoothAccZ);
        delay(3);
    }
    gyroSumX = 0;
    gyroSumY = 0;
    gyroSumZ = 0;
    for (int i = 0; i < 500; i++)
    {
        m_gyro.read();
        m_compass.read();
        Smoothing(&m_compass.a.x, &smoothAccX);
        Smoothing(&m_compass.a.y, &smoothAccY);
        Smoothing(&m_compass.a.z, &smoothAccZ);
        gyroSumX += (m_gyro.g.x);
        gyroSumY += (m_gyro.g.y);
        gyroSumZ += (m_gyro.g.z);
        delay(3);
    }
    offSetX = gyroSumX / 500.0;
    offSetY = gyroSumY / 500.0;
    offSetZ = gyroSumZ / 500.0;
    m_compass.read();

    //calculate the initial quaternion
    //these are rough values. This calibration works a lot better if the device is kept as flat as possible
    //find the initial pitch and roll
    pitch = ToDeg(fastAtan2(m_compass.a.x, sqrt(m_compass.a.y * m_compass.a.y + m_compass.a.z * m_compass.a.z)));
    roll = ToDeg(fastAtan2(-1 * m_compass.a.y, sqrt(m_compass.a.x * m_compass.a.x + m_compass.a.z * m_compass.a.z)));


    if (m_compass.a.z > 0)
    {
        if (m_compass.a.x > 0)
        {
            pitch = 180.0 - pitch;
        }
        else
        {
            pitch = -180.0 - pitch;
        }
        if (m_compass.a.y > 0)
        {
            roll = -180.0 - roll;
        }
        else
        {
            roll = 180.0 - roll;
        }
    }

    floatMagX = (m_compass.m.x - compassXMin) * inverseXRange - 1.0;
    floatMagY = (m_compass.m.y - compassYMin) * inverseYRange - 1.0;
    floatMagZ = (m_compass.m.z - compassZMin) * inverseZRange - 1.0;
    //tilt compensate the m_compass
    float xMag = (floatMagX * cos(ToRad(pitch))) + (floatMagZ * sin(ToRad(pitch)));
    float yMag = -1 * ((floatMagX * sin(ToRad(roll))  * sin(ToRad(pitch))) + (floatMagY * cos(ToRad(roll))) - (floatMagZ * sin(ToRad(roll)) * cos(ToRad(pitch))));

    yaw = ToDeg(fastAtan2(yMag, xMag));

    if (yaw < 0)
    {
        yaw += 360;
    }
    Serial.println(pitch);
    Serial.println(roll);
    Serial.println(yaw);
    //calculate the rotation matrix
    float cosPitch = cos(ToRad(pitch));
    float sinPitch = sin(ToRad(pitch));

    float cosRoll = cos(ToRad(roll));
    float sinRoll = sin(ToRad(roll));

    float cosYaw = cos(ToRad(yaw));
    float sinYaw = sin(ToRad(yaw));

    //need the transpose of the rotation matrix
    float r11 = cosPitch * cosYaw;
    float r21 = cosPitch * sinYaw;
    float r31 = -1.0 * sinPitch;

    float r12 = -1.0 * (cosRoll * sinYaw) + (sinRoll * sinPitch * cosYaw);
    float r22 = (cosRoll * cosYaw) + (sinRoll * sinPitch * sinYaw);
    float r32 = sinRoll * cosPitch;

    float r13 = (sinRoll * sinYaw) + (cosRoll * sinPitch * cosYaw);
    float r23 = -1.0 * (sinRoll * cosYaw) + (cosRoll * sinPitch * sinYaw);
    float r33 = cosRoll * cosPitch;



    //convert to quaternion
    q0 = 0.5 * sqrt(1 + r11 + r22 + r33);
    q1 = (r32 - r23) / (4 * q0);
    q2 = (r13 - r31) / (4 * q0);
    q3 = (r21 - r12) / (4 * q0);


}

void IMUupdate(float *dt)
{
    static float gx;
    static float gy;
    static float gz;
    static float ax;
    static float ay;
    static float az;

    static float recipNorm;
    static float s0, s1, s2, s3;
    static float qDot1, qDot2, qDot3, qDot4;
    static float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 , _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    gx = ToRad((m_gyro.g.x - offSetX) * GYRO_SCALE);
    gy = ToRad((m_gyro.g.y - offSetY) * GYRO_SCALE);
    gz = ToRad((m_gyro.g.z - offSetZ) * GYRO_SCALE);

    ax = -1.0 * m_compass.a.x;
    ay = -1.0 * m_compass.a.y;
    az = -1.0 * m_compass.a.z;
    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    magnitude = sqrt(ax * ax + ay * ay + az * az);
    if ((magnitude > 384) || (magnitude < 128))
    {
        ax = 0;
        ay = 0;
        az = 0;
    }

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {

        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _4q0 = 4.0f * q0;
        _4q1 = 4.0f * q1;
        _4q2 = 4.0f * q2;
        _8q1 = 8.0f * q1;
        _8q2 = 8.0f * q2;
        q0q0 = q0 * q0;
        q1q1 = q1 * q1;
        q2q2 = q2 * q2;
        q3q3 = q3 * q3;

        // Gradient decent algorithm corrective step
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback step
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    q0 += qDot1 **dt;
    q1 += qDot2 **dt;
    q2 += qDot3 **dt;
    q3 += qDot4 **dt;

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

void AHRSupdate(float *dt)
{
    static float gx;
    static float gy;
    static float gz;
    static float ax;
    static float ay;
    static float az;
    static float mx;
    static float my;
    static float mz;


    static float recipNorm;
    static float s0, s1, s2, s3;
    static float qDot1, qDot2, qDot3, qDot4;
    static float hx, hy;
    static float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2;
    static float _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

    gx = ToRad((m_gyro.g.x - offSetX) * GYRO_SCALE);
    gy = ToRad((m_gyro.g.y - offSetY) * GYRO_SCALE);
    gz = ToRad((m_gyro.g.z - offSetZ) * GYRO_SCALE);

    ax = -1.0 * m_compass.a.x;
    ay = -1.0 * m_compass.a.y;
    az = -1.0 * m_compass.a.z;

    mx = floatMagX;
    my = floatMagY;
    mz = floatMagZ;
    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    magnitude = sqrt(ax * ax + ay * ay + az * az);

    if ((magnitude > 384) || (magnitude < 128))
    {
        ax = 0;
        ay = 0;
        az = 0;
    }

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {


        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;
        // Normalise magnetometer measurement
        recipNorm = invSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;
        // Auxiliary variables to avoid repeated arithmetic
        _2q0mx = 2.0f * q0 * mx;
        _2q0my = 2.0f * q0 * my;
        _2q0mz = 2.0f * q0 * mz;
        _2q1mx = 2.0f * q1 * mx;
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _2q0q2 = 2.0f * q0 * q2;
        _2q2q3 = 2.0f * q2 * q3;
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;



        // Reference direction of Earth's magnetic field
        hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
        hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
        _2bx = sqrt(hx * hx + hy * hy);
        _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
        _4bx = 2.0f * _2bx;
        _4bz = 2.0f * _2bz;

        // Gradient decent algorithm corrective step
        s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) -
             _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
             (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
             _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) -
             4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) +
                     _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz *
                             (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 *
             (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3)
                     + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz *
                             (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) *
             (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) *
             (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) +
                     _2bz * (0.5f - q1q1 - q2q2) - mz);
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback step
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    q0 += qDot1 **dt;
    q1 += qDot2 **dt;
    q2 += qDot3 **dt;
    q3 += qDot4 **dt;

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

void GetEuler(void)
{
    roll = ToDeg(fastAtan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2)));
    pitch = ToDeg(asin(2 * (q0 * q2 - q3 * q1)));
    yaw = ToDeg(fastAtan2(2 * (q0 * q3 + q1 * q2) , 1 - 2 * (q2 * q2 + q3 * q3)));
    if (yaw < 0)
    {
        yaw += 360;
    }

}
float fastAtan2( float y, float x)
{
    static float atan;
    static float z;
    if ( x == 0.0f )
    {
        if ( y > 0.0f ) return PIBY2_FLOAT;
        if ( y == 0.0f ) return 0.0f;
        return -PIBY2_FLOAT;
    }
    z = y / x;
    if ( fabs( z ) < 1.0f )
    {
        atan = z / (1.0f + 0.28f * z * z);
        if ( x < 0.0f )
        {
            if ( y < 0.0f ) return atan - PI_FLOAT;
            return atan + PI_FLOAT;
        }
    }
    else
    {
        atan = PIBY2_FLOAT - z / (z * z + 0.28f);
        if ( y < 0.0f ) return atan - PI_FLOAT;
    }
    return atan;
}

float invSqrt(float number)
{
    volatile long i;
    volatile float x, y;
    volatile const float f = 1.5F;

    x = number * 0.5F;
    y = number;
    i = * ( long *) &y;
    i = 0x5f375a86 - ( i >> 1 );
    y = * ( float *) &i;
    y = y * ( f - ( x * y * y ) );
    return y;
}



void Smoothing(int *raw, float *smooth)
{
    *smooth = (*raw * (0.15)) + (*smooth * 0.85);
}
