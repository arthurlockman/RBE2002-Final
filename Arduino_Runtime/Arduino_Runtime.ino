#include "Robotmap.h"

// #define DEBUG //Comment out to disable debug messages.
// #define TESTING //Comment out to disable testing.
// #define IMU //Comment out to disable IMU
// #define OPENIMU //Use OpenIMU Library
// #define DRIVE_PID //Comment out to disable Drive PID

Servo             m_north, m_west, m_south, m_east;
int               m_loopState = 0;
int               m_gyroZero = 0;
boolean           enabled = false;
boolean           m_stopped = true;
boolean           m_navigate = false;
NavigationState   m_navigationState = kNavigationStart;
NavigationState   m_prevNavState = kNavigationStart;
int               m_navigationCurrentWall = 1;
int               m_navigationCurrentDir = 1;
Encoder           m_encoderNorth(kNorthEncoderA, kNorthEncoderB);
Encoder           m_encoderWest(kWestEncoderA, kWestEncoderB);
SingleEncoder     m_encoderEast(kEastEncoderA, kSingleEncTicksPerRev);
SingleEncoder     m_encoderSouth(kSouthEncoderA, kSingleEncTicksPerRev);
Ultrasonic        m_rangeNorth(kNorthRangeOut);
Ultrasonic        m_rangeWest(kWestRangeOut);
Ultrasonic        m_rangeSouth(kSouthRangeOut);
Ultrasonic        m_rangeEast(kEastRangeOut);
LightSensor       m_lightNorth(kLightSensorNorth, kLightSensorThresh);
LightSensor       m_lightWest(kLightSensorWest, kLightSensorThresh);
LightSensor       m_lightSouth(kLightSensorSouth, kLightSensorThresh);
LightSensor       m_lightEast(kLightSensorEast, kLightSensorThresh);
FlameSensor       m_flameNorth(kFlameSensorNorth);
FlameSensor       m_flameWest(kFlameSensorWest);
FlameSensor       m_flameSouth(kFlameSensorSouth);
FlameSensor       m_flameEast(kFlameSensorEast);
int               m_randomSeed = 0;
int               printCounter = 0;
int               m_homeCounter = 0;
bool              m_homedShort = false;

void setup()
{
    Serial.begin(115200);
    initializeMotors();
    testCode();

    // Attach interrupt for speed-only encoders
    attachInterrupt(m_encoderEast.interruptPin, updateEastEncoder, CHANGE);
    attachInterrupt(m_encoderSouth.interruptPin, updateSouthEncoder, CHANGE);
    // attachInterrupt(m_encoderNorth.interruptPin, updateNorthEncoder, CHANGE);
    // attachInterrupt(m_encoderWest.interruptPin, updateWestEncoder, CHANGE);

    // Initialize fan control
    pinMode(kFanNorth, OUTPUT);
    pinMode(kFanWest, OUTPUT);
    pinMode(kFanSouth, OUTPUT);
    pinMode(kFanEast, OUTPUT);
    setFans(0);
}

void loop()
{
    while (Serial.available() > 0)
    {
        String command = Serial.readStringUntil('\n');
        // Serial.println(command);
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
            m_navigate = false;
            m_navigationState = kNavigationStart;
        }
        else if (command == "sp")
        {
            spin(1);
        }
        else if (command == "en")
        {
            enable();
        }
        else if (command == "ds")
        {
            disable();
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
        else if (command == "nav")
        {
            m_navigate = true;
            m_navigationState = kNavigationStart;
            // Serial.println("got nav command");
        }
        else if (command.substring(0, 3) == "imu")
        {
            imuRotation = atof(command.substring(3).c_str());
            // Serial.println(imuRotation);
        }
        else if (command.substring(0, 3) == "ran")
        {
            m_randomSeed = atoi(command.substring(3).c_str());
            randomSeed(m_randomSeed);
        }
    }
    printDebuggingMessages();
    updateDrive();
    navigate();
}

/**
 * @brief Enables the robot
 * @details Sets all the encoders to the origin and locks rotation,
 * also sets enabled to true so drive can occur.
 */
void enable()
{
    printToConsole("Robot Enabled: ");
    m_encoderNorth.write(0);
    m_encoderWest.write(0);
    m_encoderSouth.write(0);
    m_encoderEast.write(0);
    enabled = true;
    lockRotation();
}

/**
 * @brief Disables the robot.
 * @details Sets enabled to fale so drive stops.
 */
void disable()
{
    printToConsole("Robot disabled");
    enabled = false;
}

/**
 * @brief Navigate field.
 * @details Handles field navigation.
 */
void navigate()
{
    if (m_navigate)
    {
        int candleSide = candleVisible();
        switch (m_navigationState)
        {
        case kNavigationStart:
            m_navigationCurrentWall = (m_rangeEast.distance() < m_rangeWest.distance()) ? 4 : 2;
            Serial.println(m_navigationCurrentWall);
            m_navigationCurrentDir  = (m_navigationCurrentWall == 4) ? 0 : 1;
            Serial.println(m_navigationCurrentDir);
            changeNavState(kNavigationFollowWall);
            m_navStack.push((FollowCommand)
            {
                m_navigationCurrentWall,
                ((m_navigationCurrentDir == 1) ? 0 : 1)
            });
            break;
        case kNavigationFollowWall:
            if (candleSide != -1)
            {
                changeNavState(kNavigationHomeOnCandle);
            }
            else if (followWall(m_navigationCurrentWall, m_navigationCurrentDir))
            {
                changeNavState(kNavigationDecideNext);
            }
            break;
        case kNavigationDecideNext:
            Serial.println("finding next");
            if (!(m_lightNorth.read() || m_lightWest.read() || m_lightSouth.read() || m_lightEast.read()))
            {
                switch (m_navigationCurrentWall)
                {
                case 1: //north
                    switch (m_navigationCurrentDir)
                    {
                    case 1: //east
                        m_navigationCurrentDir = getLargestFrontierLeftRight(4);
                        m_navigationCurrentWall = 4;
                        break;
                    default: //west
                        m_navigationCurrentDir = getLargestFrontierLeftRight(2);
                        m_navigationCurrentWall = 2;
                        break;
                    }
                    break;
                case 2: //west
                    switch (m_navigationCurrentDir)
                    {
                    case 1: //north
                        m_navigationCurrentDir = getLargestFrontierLeftRight(1);
                        m_navigationCurrentWall = 1;
                        break;
                    default: //south
                        m_navigationCurrentDir = getLargestFrontierLeftRight(3);
                        m_navigationCurrentWall = 3;
                        break;
                    }
                    break;
                case 3: //south
                    switch (m_navigationCurrentDir)
                    {
                    case 1: //west
                        m_navigationCurrentDir = getLargestFrontierLeftRight(2);
                        m_navigationCurrentWall = 2;
                        break;
                    default: //east
                        m_navigationCurrentDir = getLargestFrontierLeftRight(4);
                        m_navigationCurrentWall = 4;
                        break;
                    }
                    break;
                case 4: //east
                    switch (m_navigationCurrentDir)
                    {
                    case 1: //south
                        m_navigationCurrentWall = 3;
                        m_navigationCurrentDir = getLargestFrontierLeftRight(3);
                        break;
                    default: //north
                        m_navigationCurrentWall = 1;
                        m_navigationCurrentDir = getLargestFrontierLeftRight(1);
                        break;
                    }
                    break;
                }
            }
            else     //Handle case where light sensors are tripped
            {
                switch (m_navigationCurrentWall)
                {
                case 1: //north
                    switch (m_navigationCurrentDir)
                    {
                    case 1:
                        m_navigationCurrentWall = 4;
                        m_navigationCurrentDir = 1;
                        break;
                    default:
                        m_navigationCurrentWall = 2;
                        m_navigationCurrentDir = 0;
                        break;
                    }
                    break;
                case 2: //west
                    switch (m_navigationCurrentDir)
                    {
                    case 1:
                        m_navigationCurrentWall = 1;
                        m_navigationCurrentDir = 1;
                        break;
                    default:
                        m_navigationCurrentWall = 3;
                        m_navigationCurrentDir = 0;
                        break;
                    }
                    break;
                case 3: //south
                    switch (m_navigationCurrentDir)
                    {
                    case 1:
                        m_navigationCurrentWall = 2;
                        m_navigationCurrentDir = 1;
                        break;
                    default:
                        m_navigationCurrentWall = 4;
                        m_navigationCurrentDir = 0;
                        break;
                    }
                    break;
                case 4: //east
                    switch (m_navigationCurrentDir)
                    {
                    case 1:
                        m_navigationCurrentWall = 3;
                        m_navigationCurrentDir = 1;
                        break;
                    default:
                        m_navigationCurrentWall = 1;
                        m_navigationCurrentDir = 0;
                        break;
                    }
                    break;
                }
            }
            changeNavState(kNavigationFollowWall);
            m_navStack.push((FollowCommand)
            {
                m_navigationCurrentWall,
                ((m_navigationCurrentDir == 1) ? 0 : 1)
            });
            break;
        case kNavigationHomeOnCandle:
            if (homeOnCandle(candleSide))
            {
                stopDrive();
                changeNavState(kNavigtationApproachCandle);
                Serial.println("flfo");
            }
            break;
        case kNavigtationApproachCandle:
            Serial.println("Approaching candle");
            Serial.println((candleSide + 1));
            if (m_homedShort || driveDistance((candleSide + 1), 300))
            {
                changeNavState(kNavigationExtinguishFlame);
                writeDisplacement(candleSide);
                writeDisplacement(candleSide);
                stopDrive();
            }
            break;
        case kNavigationExtinguishFlame:
            stopDrive();
            if (candleSide == -1 && printCounter == 4000)
            {
                Serial.println("flex");
                changeNavState(kNavigationReturnToWall);
                FollowCommand _cmd = m_navStack.pop();
                m_navigationCurrentWall = _cmd.side;
                m_navigationCurrentDir  = _cmd.direction;
            }
            else if (candleSide != -1)
            {
                printCounter = 0;
            }
            else
            {
                printCounter++;
            }
            break;
        case kNavigationReturnToWall:
            if (returnToWall(m_navigationCurrentWall))
            {
                changeNavState(kNavigationReturnHome);
            }
            break;
        case kNavigationReturnHome: //playback moves
            if (abs(getDisplacementX()) < 6.0 && abs(getDisplacementY()) < 6.0)
            {
                changeNavState(kNavigationDone);
            }
            else if (followWall(m_navigationCurrentWall, m_navigationCurrentDir))
            {
                if (!m_navStack.isEmpty())
                {
                    FollowCommand _cmd = m_navStack.pop();
                    m_navigationCurrentWall = _cmd.side;
                    m_navigationCurrentDir  = _cmd.direction;
                }
                else
                {
                    changeNavState(kNavigationDone);
                    disable();
                }
            }
            break;
        case kNavigationDone:
            stopDrive();
            setFans(0);
            break;
        }
    }
}

/**
 * @brief Return to a wall.
 * @details Returns to the wall in the direction specified by 
 * the parameter wall.
 * 
 * @param wall The wall to return to.,
 * @return When the wall has been reached.
 */
bool returnToWall(int wall)
{
    switch (wall)
    {
    case 1:
        if (m_rangeNorth.distance() < 8.0 || m_lightNorth.read())
        {
            stopDrive();
            return true;
        }
        drive(0);
        break;
    case 2:
        if (m_rangeWest.distance() < 8.0 || m_lightWest.read())
        {
            stopDrive();
            return true;
        }
        drive(270);
        break;
    case 3:
        if (m_rangeSouth.distance() < 8.0 || m_lightSouth.read())
        {
            stopDrive();
            return true;
        }
        drive(180);
        break;
    case 4:
        if (m_rangeEast.distance() < 8.0 || m_lightEast.read())
        {
            stopDrive();
            return true;
        }
        drive(90);
        break;
    }
    return false;
}

/**
 * @brief Prints the displacement to the console.
 * @details Prints the displacement to the console, correcting
 * for the position of the candle relative to the robot center.
 * 
 * @param candleSide The side that the candle is on.
 */
void writeDisplacement(int candleSide)
{
    float dispx = getDisplacementX();
    float dispy = getDisplacementY();
    switch (candleSide)
    {
    case 0:
        dispx += 9.0;
        break;
    case 1:
        dispy += 9.0;
        break;
    case 2:
        dispx -= 9.0;
        break;
    case 3:
        dispy -= 9.0;
        break;
    }
    String out;
    out += "dsp";
    out += (int)dispx;
    out += ",";
    out += (int)dispy;
    out += ",";
    out += (int)getFlameHeight(candleSide);
    Serial.println(out);
}

/**
 * @brief Switch the navigation state.
 * @details Switches the nav state, storing the previous.
 * 
 * @param navState The state to switch to.
 */
void changeNavState(NavigationState navState)
{
    m_prevNavState = m_navigationState;
    m_navigationState = navState;
}

/**
 * @brief Follow a wall on specified side.
 * @details Follows a wall on a specified side in a specified direction.
 * 
 * @param side The side to follow on
 * @param dir The direction, 1 indicating right.
 * 
 * @return When the wall has ended (corner).
 */
int followWall(int side, int dir)
{
    int   driveDir;
    float wallDist;
    float wall2Dist;
    bool  wallLight;
    bool  wall2Light;
    switch (side)
    {
    case 1:
        wallDist = m_rangeNorth.distance();
        wallLight = m_lightNorth.read();
        wall2Dist = (dir == 1) ? m_rangeEast.distance() : m_rangeWest.distance();
        wall2Light = (dir == 1) ? m_lightEast.read() : m_lightWest.read();
        if (wall2Dist < kWallMaxdist || wall2Light)
        {
            stopDrive();
            Serial.println("stopping");
            return 1;
        }
        else if (wallLight && dir == 1)
        {
            drive(135);
        }
        else if (wallLight && dir != 1)
        {
            drive(225);
        }
        else if (wallDist > kWallMaxdist && dir == 1)
        {
            drive(45);
        }
        else if (wallDist > kWallMaxdist && dir != 1)
        {
            drive(315);
        }
        else if (wallDist < kWallMinDist && dir == 1)
        {
            drive(135);
        }
        else if (wallDist < kWallMinDist && dir != 1)
        {
            drive(225);
        }
        else if (dir == 1)
        {
            drive(90);
        }
        else if (dir != 1)
        {
            drive(270);
        }
        break;
    case 2:
        wallDist = m_rangeWest.distance();
        wall2Dist = (dir == 1) ? m_rangeNorth.distance() : m_rangeSouth.distance();
        wallLight = m_lightWest.read();
        wall2Light = (dir == 1) ? m_lightNorth.read() : m_lightSouth.read();
        if (wall2Dist < kWallMaxdist || wall2Light)
        {
            stopDrive();
            return 1;
        }
        else if (wallLight && dir == 1)
        {
            drive(45);
        }
        else if (wallLight && dir != 1)
        {
            drive(135);
        }
        else if (wallDist > kWallMaxdist && dir == 1)
        {
            drive(315);
        }
        else if (wallDist > kWallMaxdist && dir != 1)
        {
            drive(225);
        }
        else if (wallDist < kWallMinDist && dir == 1)
        {
            drive(45);
        }
        else if (wallDist < kWallMinDist && dir != 1)
        {
            drive(135);
        }
        else if (dir == 1)
        {
            drive(0);
        }
        else if (dir != 1)
        {
            drive(180);
        }
        break;
    case 3:
        wallDist = m_rangeSouth.distance();
        wall2Dist = (dir == 1) ? m_rangeWest.distance() : m_rangeEast.distance();
        wallLight = m_lightSouth.read();
        wall2Light = (dir == 1) ? m_lightWest.read() : m_lightEast.read();
        if (wall2Dist < kWallMaxdist || wall2Light)
        {
            stopDrive();
            return 1;
        }
        else if (wallLight && dir == 1)
        {
            drive(315);
        }
        else if (wallLight && dir != 1)
        {
            drive(45);
        }
        else if (wallDist > kWallMaxdist && dir == 1)
        {
            drive(225);
        }
        else if (wallDist > kWallMaxdist && dir != 1)
        {
            drive(135);
        }
        else if (wallDist < kWallMinDist && dir == 1)
        {
            drive(315);
        }
        else if (wallDist < kWallMinDist && dir != 1)
        {
            drive(45);
        }
        else if (dir == 1)
        {
            drive(270);
        }
        else if (dir != 1)
        {
            drive(90);
        }
        break;
    case 4:
        wallDist = m_rangeEast.distance();
        wall2Dist = (dir == 1) ? m_rangeSouth.distance() : m_rangeNorth.distance();
        wallLight = m_lightEast.read();
        wall2Light = (dir == 1) ? m_lightSouth.read() : m_lightNorth.read();
        if (wall2Dist < kWallMaxdist || wall2Light)
        {
            stopDrive();
            return 1;
        }
        else if (wallLight && dir == 1)
        {
            drive(225);
        }
        else if (wallLight && dir != 1)
        {
            drive(315);
        }
        else if (wallDist > kWallMaxdist && dir == 1)
        {
            drive(135);
        }
        else if (wallDist > kWallMaxdist && dir != 1)
        {
            drive(45);
        }
        else if (wallDist < kWallMinDist && dir == 1)
        {
            drive(225);
        }
        else if (wallDist < kWallMinDist && dir != 1)
        {
            drive(315);
        }
        else if (dir == 1)
        {
            drive(180);
        }
        else if (dir != 1)
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

/**
 * @brief Returns the robot's current orientation.
 * @return A float, the orientation of the robot.
 */
float getCurrentOrientation()
{
    return imuRotation;
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
}

/**
 * @brief Handles PID drive update.
 * @details This method updates the motor speeds according to
 * the values calculated in the PID service routine.
 */
void updateDrive()
{
    float headingError = imuRotation - startOrientation;
    int northSetpointAdj = northSetpoint + (int)(headingError * kCompassCorrectionP);
    int westSetpointAdj  = westSetpoint - (int)(headingError * kCompassCorrectionP);
    int southSetpointAdj = southSetpoint - (int)(headingError * kCompassCorrectionP);
    int eastSetpointAdj  = eastSetpoint + (int)(headingError * kCompassCorrectionP);

    //Set directions for encoders
    if (northSetpointAdj < 90) southDirection = 1;
    else southDirection = 0;
    if (westSetpointAdj < 90) eastDirection = 0;
    else eastDirection = 1;

    if (!m_stopped)
    {
        m_north.write(180 - ((northSetpointAdj < 0) ? northSetpointAdj + 180 : northSetpointAdj));
        m_south.write(((southSetpointAdj < 0) ? southSetpointAdj + 180 : southSetpointAdj));
        m_east.write(180 - ((eastSetpointAdj < 0) ? eastSetpointAdj + 180 : eastSetpointAdj));
        m_west.write(((westSetpointAdj < 0) ? westSetpointAdj + 180 : westSetpointAdj));
    }
    else
    {
        m_north.write(90);
        m_west.write(90);
        m_east.write(90);
        m_south.write(90);
    }
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
    float nowHeading = imuRotation;
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
 * @brief Locks robot rotation
 * @details Locks rotation by setting the starting orientation
 * to the current IMU rotation.
 */
void lockRotation()
{
    startOrientation = imuRotation;
    // Serial.println(startOrientation);
}

/**
 * @brief Get the largest frontier.
 * 
 * @return The direction of the largest frontier for movement.
 */
int getLargestFrontier()
{
    return greatestIndex(4.0, m_rangeNorth.distance(), m_rangeWest.distance(),
                         m_rangeEast.distance(), m_rangeSouth.distance());
}

/**
 * @brief Returns the largest frontier left/right of current wall.
 * @details Returns the best possible frontier to the left/right of
 * the current wall.
 * 
 * @param currentSide The wall that the robot is trying to switch
 * to tracking on.
 * @return An int, 1 indicating right.
 */
int getLargestFrontierLeftRight(int currentSide)
{
    float northDist = m_rangeNorth.distance();
    float westDist = m_rangeWest.distance();
    float southDist = m_rangeSouth.distance();
    float eastDist = m_rangeEast.distance();

    if (((northDist < 10.0) + (westDist < 10.0)
            + (southDist < 10.0) + (eastDist < 10.0)) > 1)
    {
        Serial.println("On corner.");
        Serial.println(m_navigationCurrentWall);
        Serial.println(m_navigationCurrentDir);
        Serial.println(currentSide);
        switch (currentSide)
        {
        case 1: //north
            if (m_navigationCurrentWall == 2)
            {
                return 1;
            }
            else if (m_navigationCurrentWall == 4)
            {
                return 0;
            }
            else
            {
                return greatestIndex(2.0, eastDist, westDist);
            }
            break;
        case 2: //west
            if (m_navigationCurrentWall == 3)
            {
                return 1;
            }
            else if (m_navigationCurrentWall == 1)
            {
                return 0;
            }
            else
            {
                return greatestIndex(2.0, northDist, southDist);
            }
            break;
        case 3: //south
            if (m_navigationCurrentWall == 4)
            {
                return 1;
            }
            else if (m_navigationCurrentWall == 2)
            {
                return 0;
            }
            else
            {
                return greatestIndex(2.0, westDist, eastDist);
            }
            break;
        case 4: //east
            if (m_navigationCurrentWall == 1)
            {
                return 1;
            }
            else if (m_navigationCurrentWall == 3)
            {
                return 0;
            }
            else
            {
                return greatestIndex(2.0, northDist, southDist);
            }
            break;
        default:
            return 1;
        }
    }
    else
    {
        Serial.println("On wall.");
        Serial.println(m_navigationCurrentWall);
        Serial.println(m_navigationCurrentDir);
        Serial.println(currentSide);
        // return random(0, 2);
        switch (currentSide)
        {
        case 1:
            if (m_navigationCurrentWall == 4)
            {
                return 1;
            }
            else if (m_navigationCurrentWall == 2)
            {
                return 0;
            }
            else if (m_navigationCurrentWall == 1)
            {
                return 1;
            }
            break;
        case 2:
            if (m_navigationCurrentWall == 1)
            {
                return 1;
            }
            else if (m_navigationCurrentWall == 3)
            {
                return 0;
            }
            else if (m_navigationCurrentWall == 2)
            {
                return 1;
            }
            break;
        case 3:
            if (m_navigationCurrentWall == 2)
            {
                return 1;
            }
            else if (m_navigationCurrentWall == 4)
            {
                return 0;
            }
            else if (m_navigationCurrentWall == 3)
            {
                return 1;
            }
            break;
        case 4:
            if (m_navigationCurrentWall == 3)
            {
                return 1;
            }
            else if (m_navigationCurrentWall == 1)
            {
                return 0;
            }
            else if (m_navigationCurrentWall == 4)
            {
                return 1;
            }
            break;
        default:
            return 1;
        }
    }
}

/**
 * @brief Gets a heading of the flame.
 * @details Calculates a heading of the flame.
 * @return The heading in degrees from north.
 */
float getFlameHeading()
{
    float noDist = m_flameNorth.distance();
    noDist = (noDist == -1 || noDist == -2 || noDist > 18.0) ? 0 : noDist;
    float weDist = m_flameWest.distance();
    weDist = (weDist == -1 || weDist == -2 || weDist > 18.0) ? 0 : weDist;
    float soDist = m_flameSouth.distance();
    soDist = (soDist == -1 || soDist == -2 || soDist > 18.0) ? 0 : soDist;
    float eaDist = m_flameEast.distance();
    eaDist = (eaDist == -1 || eaDist == -2 || eaDist > 18.0) ? 0 : eaDist;
    float noBias = (eaDist >= weDist) ? 90.0 : 450.0;

    float numerator = (noBias * noDist) + (180.0 * eaDist) +
                      (270.0 * soDist) + (360.0 * weDist);
    float denominator = noDist + eaDist + weDist + soDist;
    return (numerator / denominator) - 90.0;
}

/**
 * @brief Returns the side that the candle is visible on.
 * @details Returns a direction, -1 if not visible.
 * @return An int, the direction of the flame.
 */
int candleVisible()
{
    int minimumSide = -1;
    int minimum = 4000;
    float northDist = m_flameNorth.distance();
    float westDist = m_flameWest.distance();
    float southDist = m_flameSouth.distance();
    float eastDist = m_flameEast.distance();
    if (northDist > 0.0 && northDist < minimum && northDist < 24.0)
    {
        minimum = northDist;
        minimumSide = 0;
    }
    if (westDist > 0.0 && westDist < minimum && westDist < 24.0)
    {
        minimum = westDist;
        minimumSide = 1;
    }
    if (southDist > 0.0 && southDist < minimum && southDist < 24.0)
    {
        minimum = southDist;
        minimumSide = 2;
    }
    if (eastDist > 0.0 && eastDist < minimum && eastDist < 24.0)
    {
        minimum = eastDist;
        minimumSide = 3;
    }
    return minimumSide;
}

/**
 * @brief Homes in on candle.
 * 
 * @param d The direction to home in.
 * @return When the homing has completed.
 */
bool homeOnCandle(int d)
{
    static int previousd = d;
    static int homeState = 2;
    static bool hasSeen = true;
    static int previousDirection = ((d == 0) ? (((m_navigationCurrentWall == 3 && m_navigationCurrentDir == 1) || m_navigationCurrentWall == 4) ? 270 : 90) :
                                    ((d == 1) ? (((m_navigationCurrentWall == 4 && m_navigationCurrentDir == 1) || m_navigationCurrentWall == 1) ? 180 : 0) :
                                     ((d == 2) ? (((m_navigationCurrentWall == 1 && m_navigationCurrentDir == 1) || m_navigationCurrentWall == 2) ? 90 : 270) :
                                      ((d == 3) ? (((m_navigationCurrentWall == 2 && m_navigationCurrentDir == 1) || m_navigationCurrentWall == 3) ? 0 : 180) : 0))));
    if (d == -1)
    {
        d = previousd;
        if (hasSeen)
        {
            previousDirection = (previousDirection + 180) % 360;
        }
        hasSeen = false;
    }
    else
    {
        previousd = d;
        hasSeen = true;
    }
    Ultrasonic *candleSensor = (d == 0) ? &m_rangeNorth :
                               ((d == 1) ? &m_rangeWest :
                                ((d == 2) ? &m_rangeSouth : &m_rangeEast));

    static float startLoc;
    static float highVal = candleSensor->distance();
    static float previous = candleSensor->distance();
    float sensorValue = candleSensor->distance();
    static bool startedShort = (sensorValue < 8.0) ? true : false;
    m_homedShort = startedShort;
    switch (homeState)
    {
    case 1://finds candle
        if (sensorValue - highVal < -5)
        {
            homeState++;
            startLoc = (d == 0 || d == 2) ? getDisplacementY() : getDisplacementX();
        }
        break;
    case 2://starts driving towards candle
        previousDirection = (360 - d * 90) % 360;
        if (sensorValue < 8)
        {
            setFans(d + 1);
            return true;
        }
        else if (sensorValue - previous > 10)
        {
            homeState++;
            highVal = sensorValue;
            previousDirection = (360 - d * 90 + 90) % 360;
        }
        break;
    case 3: //recenters on flame
        if (sensorValue - highVal < -5)
        {
            homeState = 2;
            startLoc = (d == 0 || d == 2) ? getDisplacementX() : getDisplacementY();
        }
        else if (((d == 0 || d == 2) ? getDisplacementX() : getDisplacementY()) > startLoc + 5)
        {
            previousDirection = previousDirection + 180 % 360;
            homeState++;
        }
        break;
    case 4:
        if (sensorValue - highVal < -5)
        {
            homeState = 2;
            startLoc = (d == 0 || d == 2) ? getDisplacementX() : getDisplacementY();
        }
        break;
    }

    previous = sensorValue;
    drive(previousDirection);
    return false;
    // static int previousDirection = ((d == 0) ? ((m_navigationCurrentDir == 1) ? 45 : 315) :
    //                                 ((d == 1) ? ((m_navigationCurrentDir == 1) ? 315 : 225) :
    //                                  ((d == 2) ? ((m_navigationCurrentDir == 1) ? 225 : 135) :
    //                                   ((d == 3) ? ((m_navigationCurrentDir == 1) ? 135 : 45) : 0))));
    // static int previous = ((d == 0) ? m_rangeNorth.distance() :
    //                        ((d == 1) ? m_rangeWest.distance() :
    //                         ((d == 2) ? m_rangeSouth.distance() : m_rangeEast.distance())));
    // static int switched = 0;
    // static int lastD = 0;
    // int sensorValue;

    // if (switched == 0 && d == -1)
    // {
    //     switch (d)
    //     {
    //     case 0:
    //         previousDirection = (previousDirection == 45) ? 315 : 45;
    //         break;
    //     case 1:
    //         previousDirection = (previousDirection == 315) ? 225 : 315;
    //         break;
    //     case 2:
    //         previousDirection = (previousDirection == 225) ? 135 : 225;
    //         break;
    //     case 3:
    //         previousDirection = (previousDirection == 135) ? 45 : 135;
    //         break;
    //     }
    //     switched = 1;
    // }
    // else
    // {
    //     switched = 0;
    //     lastD = d;
    //     switch (d)
    //     {
    //     case 0://north
    //         sensorValue = m_rangeNorth.distance();
    //         break;
    //     case 1: //west
    //         sensorValue = m_rangeWest.distance();
    //         break;
    //     case 2: //south
    //         sensorValue = m_rangeSouth.distance();
    //         break;
    //     case 3: //east
    //         sensorValue = m_rangeEast.distance();
    //         break;
    //     }
    //     if (sensorValue < 8)
    //     {
    //         stopDrive();
    //         setFans(d + 1);
    //         return true;
    //     }
    //     else
    //     {
    //         if (abs(sensorValue - previous) > 10)
    //         {
    //             Serial.println("switched");
    //             m_homeCounter = 0;
    //             switch (d)
    //             {
    //             case 0:
    //                 previousDirection = (previousDirection == 45) ? 315 : 45;
    //                 break;
    //             case 1:
    //                 previousDirection = (previousDirection == 315) ? 225 : 315;
    //                 break;
    //             case 2:
    //                 previousDirection = (previousDirection == 225) ? 135 : 225;
    //                 break;
    //             case 3:
    //                 previousDirection = (previousDirection == 135) ? 45 : 135;
    //                 break;
    //             }
    //         }
    //         else
    //         {
    //             m_homeCounter++;
    //         }
    //     }
    // }
    // previous = sensorValue;
    // drive(previousDirection);
    // return false;
}

/**
 * @brief Gets X displacement.
 * @return The displacement.
 */
float getDisplacementX()
{
    return -((float)m_encoderWest.read() / 360.0) * 2.75 * M_PI;
}

/**
 * @brief Gets Y displacement.
 * @return The displacement.
 */
float getDisplacementY()
{
    return ((float)m_encoderNorth.read() / 360.0) * 2.75 * M_PI;
}

/**
 * @brief Drives a distance.
 * 
 * @param dir Direction to drive.
 * @param dTime Length of time to drive for.
 * 
 * @return When the drive command has completed.
 */
bool driveDistance(int dir, int dTime)
{
    static long initialTime = millis();
    static int initialDir = dir;
    if (millis() - initialTime > dTime)
    {
        stopDrive();
        return true;
    }
    else
    {
        switch (initialDir)
        {
        case 1:
            drive(0);
            break;
        case 2:
            drive(270);
            break;
        case 3:
            drive(180);
            break;
        case 4:
            drive(90);
            break;
        }
    }
    return false;
    // static float initialDist = (dir == 1 || dir == 3) ? getDisplacementX() : getDisplacementY();
    // switch (dir)
    // {
    // case 1: //north
    //     if (abs(getDisplacementX() - initialDist) >= distance)
    //     {
    //         stopDrive();
    //         return true;
    //     }
    //     else
    //     {
    //         drive(0);
    //         return false;
    //     }
    //     break;
    // case 2: //west
    //     if (abs(getDisplacementY() - initialDist) >= distance)
    //     {
    //         stopDrive();
    //         return true;
    //     }
    //     else
    //     {
    //         drive(270);
    //         return false;
    //     }
    //     break;
    // case 3: //south
    //     if (abs(getDisplacementX() - initialDist) >= distance)
    //     {
    //         stopDrive();
    //         return true;
    //     }
    //     else
    //     {
    //         drive(180);
    //         return false;
    //     }
    //     break;
    // case 4: //east
    //     if (abs(getDisplacementY() - initialDist) >= distance)
    //     {
    //         stopDrive();
    //         return true;
    //     }
    //     else
    //     {
    //         drive(90);
    //         return false;
    //     }
    //     break;
    // }
}

/**
 * @brief Get the Z height of the flame.
 * 
 * @param d The flame direction
 * @return The Z height, in inches.
 */
float getFlameHeight(int d)
{
    float sensorVal;
    switch (d)
    {
    case 0:
        sensorVal = m_flameNorth.read();
        break;
    case 1:
        sensorVal = m_flameWest.read();
        break;
    case 2:
        sensorVal = m_flameSouth.read();
        break;
    case 3:
        sensorVal = m_flameEast.read();
        break;
    }
    if (sensorVal < 20)
    {
        sensorVal = 11;
    }
    else if (sensorVal < 30)
    {
        sensorVal = 9.5;
    }
    else
    {
        sensorVal = 7.5;
    }
    return sensorVal;
}
