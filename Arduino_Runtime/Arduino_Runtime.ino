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
    pinMode (STATUS_LED, OUTPUT); // Status LED

    I2C_Init();

    Serial.println("Pololu MinIMU-9 + Arduino AHRS");

    digitalWrite(STATUS_LED, LOW);
    delay(1500);

    Accel_Init();
    Compass_Init();
    Gyro_Init();

    delay(20);

    for (int i = 0; i < 32; i++) // We take some readings...
    {
        Read_Gyro();
        Read_Accel();
        for (int y = 0; y < 6; y++) // Cumulate values
            AN_OFFSET[y] += AN[y];
        delay(20);
    }

    for (int y = 0; y < 6; y++)
        AN_OFFSET[y] = AN_OFFSET[y] / 32;

    AN_OFFSET[5] -= GRAVITY * SENSOR_SIGN[5];

    //Serial.println("Offset:");
    for (int y = 0; y < 6; y++)
        Serial.println(AN_OFFSET[y]);

    delay(2000);
    digitalWrite(STATUS_LED, HIGH);

    timer = millis();
    delay(20);
    counter = 0;
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
    // Timer1.initialize(kISRRate);
    // Timer1.attachInterrupt(imuRoutine);
}

void loop()
{
    drive(0);
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
        else
        {
            imuRotation = atof(command.c_str());
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
        Serial.print(wallDist);
        Serial.print('\t');
        Serial.println(wall2Dist);
        if (wall2Dist < 6.0)
        {
            stopDrive();
            Serial.println("stopping");
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
#if defined(OPENIMU)
    return ToDeg(pitch);
#else
    float accum = 0;
    for (int i = 0; i < 20; i++)
    {
        float x_value = m_compass.m.x;
        float z_value = m_compass.m.z;

        float x_corrected = (x_value - m_compass.m_min.x) * (180) / (m_compass.m_max.x - m_compass.m_min.x) - 90;
        float z_corrected = (z_value - m_compass.m_min.z) * (180) / (m_compass.m_max.z - m_compass.m_min.z) - 90;

        float raw_orientation = atan2(z_corrected, x_corrected) * (180 / M_PI);

        accum += (raw_orientation < 0) ? raw_orientation + 360.0 : raw_orientation;
        return accum / 10.0;
#endif
}

/**
 * @brief Handles IMU readings.
 * @details This method handles the readings for the
 * IMU. This ensures that the IMU is only read once per
 * loop.
 */
void imuRoutine()
{
    if ((millis() - timer) >= 20) // Main loop runs at 50Hz
    {
        counter++;
        timer_old = timer;
        timer = millis();
        if (timer > timer_old)
            G_Dt = (timer - timer_old) / 1000.0; // Real time of loop run. We use this on the DCM algorithm (m_gyro integration time)
        else
            G_Dt = 0;

        // *** DCM algorithm
        // Data adquisition
        Read_Gyro();   // This read m_gyro data
        Read_Accel();     // Read I2C accelerometer

        if (counter > 5)  // Read m_compass data at 10Hz... (5 loop runs)
        {
            counter = 0;
            Read_Compass();    // Read I2C magnetometer
            Compass_Heading(); // Calculate magnetic heading
        }

        // Calculations...
        Matrix_update();
        Normalize();
        Drift_correction();
        Euler_angles();
        // ***

        // printdata();
    }
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
    m_stopped = true;
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

// OpenIMU Things
void Compass_Heading()
{
    float MAG_X;
    float MAG_Y;
    float cos_roll;
    float sin_roll;
    float cos_pitch;
    float sin_pitch;

    cos_roll = cos(roll);
    sin_roll = sin(roll);
    cos_pitch = cos(pitch);
    sin_pitch = sin(pitch);

    // adjust for LSM303 m_compass axis offsets/sensitivity differences by scaling to +/-0.5 range
    c_magnetom_x = (float)(magnetom_x - SENSOR_SIGN[6] * M_X_MIN) / (M_X_MAX - M_X_MIN) - SENSOR_SIGN[6] * 0.5;
    c_magnetom_y = (float)(magnetom_y - SENSOR_SIGN[7] * M_Y_MIN) / (M_Y_MAX - M_Y_MIN) - SENSOR_SIGN[7] * 0.5;
    c_magnetom_z = (float)(magnetom_z - SENSOR_SIGN[8] * M_Z_MIN) / (M_Z_MAX - M_Z_MIN) - SENSOR_SIGN[8] * 0.5;

    // Tilt compensated Magnetic filed X:
    MAG_X = c_magnetom_x * cos_pitch + c_magnetom_y * sin_roll * sin_pitch + c_magnetom_z * cos_roll * sin_pitch;
    // Tilt compensated Magnetic filed Y:
    MAG_Y = c_magnetom_y * cos_roll - c_magnetom_z * sin_roll;
    // Magnetic Heading
    MAG_Heading = atan2(-MAG_Y, MAG_X);
}

void Normalize(void)
{
    float error = 0;
    float temporary[3][3];
    float renorm = 0;

    error = -Vector_Dot_Product(&DCM_Matrix[0][0], &DCM_Matrix[1][0]) * .5; //eq.19

    Vector_Scale(&temporary[0][0], &DCM_Matrix[1][0], error); //eq.19
    Vector_Scale(&temporary[1][0], &DCM_Matrix[0][0], error); //eq.19

    Vector_Add(&temporary[0][0], &temporary[0][0], &DCM_Matrix[0][0]);//eq.19
    Vector_Add(&temporary[1][0], &temporary[1][0], &DCM_Matrix[1][0]);//eq.19

    Vector_Cross_Product(&temporary[2][0], &temporary[0][0], &temporary[1][0]); // c= a x b //eq.20

    renorm = .5 * (3 - Vector_Dot_Product(&temporary[0][0], &temporary[0][0])); //eq.21
    Vector_Scale(&DCM_Matrix[0][0], &temporary[0][0], renorm);

    renorm = .5 * (3 - Vector_Dot_Product(&temporary[1][0], &temporary[1][0])); //eq.21
    Vector_Scale(&DCM_Matrix[1][0], &temporary[1][0], renorm);

    renorm = .5 * (3 - Vector_Dot_Product(&temporary[2][0], &temporary[2][0])); //eq.21
    Vector_Scale(&DCM_Matrix[2][0], &temporary[2][0], renorm);
}

/**************************************************/
void Drift_correction(void)
{
    float mag_heading_x;
    float mag_heading_y;
    float errorCourse;
    //Compensation the Roll, Pitch and Yaw drift.
    static float Scaled_Omega_P[3];
    static float Scaled_Omega_I[3];
    float Accel_magnitude;
    float Accel_weight;


    //*****Roll and Pitch***************

    // Calculate the magnitude of the accelerometer vector
    Accel_magnitude = sqrt(Accel_Vector[0] * Accel_Vector[0] + Accel_Vector[1] * Accel_Vector[1] + Accel_Vector[2] * Accel_Vector[2]);
    Accel_magnitude = Accel_magnitude / GRAVITY; // Scale to gravity.
    // Dynamic weighting of accelerometer info (reliability filter)
    // Weight for accelerometer info (<0.5G = 0.0, 1G = 1.0 , >1.5G = 0.0)
    Accel_weight = constrain(1 - 2 * abs(1 - Accel_magnitude), 0, 1); //

    Vector_Cross_Product(&errorRollPitch[0], &Accel_Vector[0], &DCM_Matrix[2][0]); //adjust the ground of reference
    Vector_Scale(&Omega_P[0], &errorRollPitch[0], Kp_ROLLPITCH * Accel_weight);

    Vector_Scale(&Scaled_Omega_I[0], &errorRollPitch[0], Ki_ROLLPITCH * Accel_weight);
    Vector_Add(Omega_I, Omega_I, Scaled_Omega_I);

    //*****YAW***************
    // We make the m_gyro YAW drift correction based on m_compass magnetic heading

    mag_heading_x = cos(MAG_Heading);
    mag_heading_y = sin(MAG_Heading);
    errorCourse = (DCM_Matrix[0][0] * mag_heading_y) - (DCM_Matrix[1][0] * mag_heading_x); //Calculating YAW error
    Vector_Scale(errorYaw, &DCM_Matrix[2][0], errorCourse); //Applys the yaw correction to the XYZ rotation of the aircraft, depeding the position.

    Vector_Scale(&Scaled_Omega_P[0], &errorYaw[0], Kp_YAW); //.01proportional of YAW.
    Vector_Add(Omega_P, Omega_P, Scaled_Omega_P); //Adding  Proportional.

    Vector_Scale(&Scaled_Omega_I[0], &errorYaw[0], Ki_YAW); //.00001Integrator
    Vector_Add(Omega_I, Omega_I, Scaled_Omega_I); //adding integrator to the Omega_I
}
/**************************************************/
/*
void Accel_adjust(void)
{
 Accel_Vector[1] += Accel_Scale(speed_3d*Omega[2]);  // Centrifugal force on Acc_y = GPS_speed*GyroZ
 Accel_Vector[2] -= Accel_Scale(speed_3d*Omega[1]);  // Centrifugal force on Acc_z = GPS_speed*GyroY
}
*/
/**************************************************/

void Matrix_update(void)
{
    Gyro_Vector[0] = Gyro_Scaled_X(gyro_x); //m_gyro x roll
    Gyro_Vector[1] = Gyro_Scaled_Y(gyro_y); //m_gyro y pitch
    Gyro_Vector[2] = Gyro_Scaled_Z(gyro_z); //m_gyro Z yaw

    Accel_Vector[0] = accel_x;
    Accel_Vector[1] = accel_y;
    Accel_Vector[2] = accel_z;

    Vector_Add(&Omega[0], &Gyro_Vector[0], &Omega_I[0]);  //adding proportional term
    Vector_Add(&Omega_Vector[0], &Omega[0], &Omega_P[0]); //adding Integrator term

    //Accel_adjust();    //Remove centrifugal acceleration.   We are not using this function in this version - we have no speed measurement

#if OUTPUTMODE==1
    Update_Matrix[0][0] = 0;
    Update_Matrix[0][1] = -G_Dt * Omega_Vector[2]; //-z
    Update_Matrix[0][2] = G_Dt * Omega_Vector[1]; //y
    Update_Matrix[1][0] = G_Dt * Omega_Vector[2]; //z
    Update_Matrix[1][1] = 0;
    Update_Matrix[1][2] = -G_Dt * Omega_Vector[0]; //-x
    Update_Matrix[2][0] = -G_Dt * Omega_Vector[1]; //-y
    Update_Matrix[2][1] = G_Dt * Omega_Vector[0]; //x
    Update_Matrix[2][2] = 0;
#else                    // Uncorrected data (no drift correction)
    Update_Matrix[0][0] = 0;
    Update_Matrix[0][1] = -G_Dt * Gyro_Vector[2]; //-z
    Update_Matrix[0][2] = G_Dt * Gyro_Vector[1]; //y
    Update_Matrix[1][0] = G_Dt * Gyro_Vector[2]; //z
    Update_Matrix[1][1] = 0;
    Update_Matrix[1][2] = -G_Dt * Gyro_Vector[0];
    Update_Matrix[2][0] = -G_Dt * Gyro_Vector[1];
    Update_Matrix[2][1] = G_Dt * Gyro_Vector[0];
    Update_Matrix[2][2] = 0;
#endif

    Matrix_Multiply(DCM_Matrix, Update_Matrix, Temporary_Matrix); //a*b=c

    for (int x = 0; x < 3; x++) //Matrix Addition (update)
    {
        for (int y = 0; y < 3; y++)
        {
            DCM_Matrix[x][y] += Temporary_Matrix[x][y];
        }
    }
}

void Euler_angles(void)
{
    pitch = -asin(DCM_Matrix[2][0]);
    roll = atan2(DCM_Matrix[2][1], DCM_Matrix[2][2]);
    yaw = atan2(DCM_Matrix[1][0], DCM_Matrix[0][0]);
}

void I2C_Init()
{
    Wire.begin();
}

void Gyro_Init()
{
    m_gyro.init();
    m_gyro.writeReg(L3G_CTRL_REG4, 0x20); // 2000 dps full scale
    m_gyro.writeReg(L3G_CTRL_REG1, 0x0F); // normal power mode, all axes enabled, 100 Hz
}

void Read_Gyro()
{
    m_gyro.read();

    AN[0] = m_gyro.g.x;
    AN[1] = m_gyro.g.y;
    AN[2] = m_gyro.g.z;
    gyro_x = SENSOR_SIGN[0] * (AN[0] - AN_OFFSET[0]);
    gyro_y = SENSOR_SIGN[1] * (AN[1] - AN_OFFSET[1]);
    gyro_z = SENSOR_SIGN[2] * (AN[2] - AN_OFFSET[2]);
}

void Accel_Init()
{
    m_compass.init();
    m_compass.enableDefault();
    switch (m_compass.getDeviceType())
    {
    case LSM303::device_D:
        m_compass.writeReg(LSM303::CTRL2, 0x18); // 8 g full scale: AFS = 011
        break;
    case LSM303::device_DLHC:
        m_compass.writeReg(LSM303::CTRL_REG4_A, 0x28); // 8 g full scale: FS = 10; high resolution output mode
        break;
    default: // DLM, DLH
        m_compass.writeReg(LSM303::CTRL_REG4_A, 0x30); // 8 g full scale: FS = 11
    }
}

// Reads x,y and z accelerometer registers
void Read_Accel()
{
    m_compass.readAcc();

    AN[3] = m_compass.a.x >> 4; // shift left 4 bits to use 12-bit representation (1 g = 256)
    AN[4] = m_compass.a.y >> 4;
    AN[5] = m_compass.a.z >> 4;
    accel_x = SENSOR_SIGN[3] * (AN[3] - AN_OFFSET[3]);
    accel_y = SENSOR_SIGN[4] * (AN[4] - AN_OFFSET[4]);
    accel_z = SENSOR_SIGN[5] * (AN[5] - AN_OFFSET[5]);
}

void Compass_Init()
{
    // doesn't need to do anything because Accel_Init() should have already called m_compass.enableDefault()
}

void Read_Compass()
{
    m_compass.readMag();

    magnetom_x = SENSOR_SIGN[6] * m_compass.m.x;
    magnetom_y = SENSOR_SIGN[7] * m_compass.m.y;
    magnetom_z = SENSOR_SIGN[8] * m_compass.m.z;
}

void printdata(void)
{
    Serial.print("!");

#if PRINT_EULER == 1
    Serial.print("ANG:");
    Serial.print(ToDeg(roll));
    Serial.print(",");
    Serial.print(ToDeg(pitch));
    Serial.print(",");
    Serial.print(ToDeg(yaw));
#endif
#if PRINT_ANALOGS==1
    Serial.print(",AN:");
    Serial.print(AN[0]);  //(int)read_adc(0)
    Serial.print(",");
    Serial.print(AN[1]);
    Serial.print(",");
    Serial.print(AN[2]);
    Serial.print(",");
    Serial.print(AN[3]);
    Serial.print (",");
    Serial.print(AN[4]);
    Serial.print (",");
    Serial.print(AN[5]);
    Serial.print(",");
    Serial.print(c_magnetom_x);
    Serial.print (",");
    Serial.print(c_magnetom_y);
    Serial.print (",");
    Serial.print(c_magnetom_z);
#endif
    /*#if PRINT_DCM == 1
    Serial.print (",DCM:");
    Serial.print(convert_to_dec(DCM_Matrix[0][0]));
    Serial.print (",");
    Serial.print(convert_to_dec(DCM_Matrix[0][1]));
    Serial.print (",");
    Serial.print(convert_to_dec(DCM_Matrix[0][2]));
    Serial.print (",");
    Serial.print(convert_to_dec(DCM_Matrix[1][0]));
    Serial.print (",");
    Serial.print(convert_to_dec(DCM_Matrix[1][1]));
    Serial.print (",");
    Serial.print(convert_to_dec(DCM_Matrix[1][2]));
    Serial.print (",");
    Serial.print(convert_to_dec(DCM_Matrix[2][0]));
    Serial.print (",");
    Serial.print(convert_to_dec(DCM_Matrix[2][1]));
    Serial.print (",");
    Serial.print(convert_to_dec(DCM_Matrix[2][2]));
    #endif*/
    Serial.println();

}

long convert_to_dec(float x)
{
    return x * 10000000;
}

float Vector_Dot_Product(float vector1[3], float vector2[3])
{
    float op = 0;

    for (int c = 0; c < 3; c++)
    {
        op += vector1[c] * vector2[c];
    }

    return op;
}

//Computes the cross product of two vectors
void Vector_Cross_Product(float vectorOut[3], float v1[3], float v2[3])
{
    vectorOut[0] = (v1[1] * v2[2]) - (v1[2] * v2[1]);
    vectorOut[1] = (v1[2] * v2[0]) - (v1[0] * v2[2]);
    vectorOut[2] = (v1[0] * v2[1]) - (v1[1] * v2[0]);
}

//Multiply the vector by a scalar.
void Vector_Scale(float vectorOut[3], float vectorIn[3], float scale2)
{
    for (int c = 0; c < 3; c++)
    {
        vectorOut[c] = vectorIn[c] * scale2;
    }
}

void Vector_Add(float vectorOut[3], float vectorIn1[3], float vectorIn2[3])
{
    for (int c = 0; c < 3; c++)
    {
        vectorOut[c] = vectorIn1[c] + vectorIn2[c];
    }
}

void Matrix_Multiply(float a[3][3], float b[3][3], float mat[3][3])
{
    float op[3];
    for (int x = 0; x < 3; x++)
    {
        for (int y = 0; y < 3; y++)
        {
            for (int w = 0; w < 3; w++)
            {
                op[w] = a[x][w] * b[w][y];
            }
            mat[x][y] = 0;
            mat[x][y] = op[0] + op[1] + op[2];

            float test = mat[x][y];
        }
    }
}
