
#include "Common.h"
#include "SensorState.h"

SensorState::SensorState(TalonXXI *pRobot)
{
    mainRobot = pRobot;

    //shooter
    shooterFalcon = NULL;

    //shrouding
    shroud = new frc::Encoder(SHROUD_ANGLE_ENCODER_ONE, SHROUD_ANGLE_ENCODER_TWO, false);
    shroud->SetDistancePerPulse(ANGLE_DISTANCE_PER_PULSE);

    //turret
    turretAnalog = new frc::AnalogInput(TURRET_ANALOG);
    turretGyro = new frc::AnalogGyro(TURRET_GYRO_ANALOG);

    /*//climber
    //climbGyro = new frc::ADXRS450_Gyro(frc::SPI::kOnboardCS0);
    //climber
    climbDeliver = new frc::DutyCycleEncoder(CLIMBER_DUTY_CYCLE);
    climbDeliver->SetDistancePerRotation(CLIMB_EXTEND_DISTANCE_PER_PULE);
    winch = new frc::Encoder(WINCH_ENCODER_ONE, WINCH_ENCODER_TWO, false);
    winch->SetDistancePerPulse(WINCH_DISTANCE_PER_PULSE);
    
   
    //color wheel
    m_colorMatcher.AddColorMatch(kBlueTarget);
    m_colorMatcher.AddColorMatch(kGreenTarget);
    m_colorMatcher.AddColorMatch(kRedTarget);
    m_colorMatcher.AddColorMatch(kYellowTarget);
    */
    //colorString = "";

    //drivetrain
    leftDriveFalcon = NULL;
    rightDriveFalcon = NULL;

#ifdef USE_GYRO
    driveGyro = new frc::ADXRS450_Gyro(frc::SPI::kOnboardCS0);
#endif

    //deathstar
    deathStar = new frc::DutyCycleEncoder(DIGIN_DEATH_STAR_DUTY_CYCLE);
    deathStar->SetDistancePerRotation(DEATH_STAR_DISTANCE_PER_ROTATION);

    cellSensor0 = new frc::DigitalInput(DIGIN_DEATH_STAR_0);
    cellSensor1 = new frc::DigitalInput(DIGIN_DEATH_STAR_1);
    cellSensor2 = new frc::DigitalInput(DIGIN_DEATH_STAR_2);
    cellSensor3 = new frc::DigitalInput(DIGIN_DEATH_STAR_3);
    cellSensor4 = new frc::DigitalInput(DIGIN_DEATH_STAR_4);

    //drivetrain
    leftWheelDisDrive = 0.0;
    rightWheelDisDrive = 0.0;
    averageWheelDisDrive = 0.0;
    leftEncoderOffset = 0.0;
    rightEncoderOffset = 0.0;
    leftRawReading = 0.0;
    rightRawReading = 0.0;
    LocalReset();
}

void SensorState::LocalReset()
{
    // DO NOT RESET ENCODERS HERE! User can request as neede
    //ResetShootingEncoder();
    //ResetTurretEncoder();
    //ResetClimbEncoder();
    //ResetDriveEncoders();

    loopCount = 0;

    //shooter
    shooterCurrentPos = 0.0;
    shooterPosRadians = 0.0;
    shooterRadPerSec = 0.0;
    shooterOldPosRadians = 0.0;

    //turret
    turretAnalogAngleRaw = turretAnalog->GetVoltage() * TURRET_DEGREES_PER_VOLT;
    turretAnalogAngleMod = turretAnalogAngleRaw + TURRET_ANALOG_OFFSET;
    if (turretAnalogAngleMod >= TURRET_DEGREE_RANGE)
    {
        turretAnalogAngleMod = turretAnalogAngleMod - TURRET_DEGREE_RANGE;
    }
    else if (turretAnalogAngleMod < 0.0)
    {
        turretAnalogAngleMod = turretAnalogAngleMod + TURRET_DEGREE_RANGE;
    }
    turretAnalogAnglez = turretAnalogAngleMod;
    turretAnalogVel = 0.0;
    turretGyroAngle = turretGyro->GetAngle();
    turretGyroAnglez = turretGyroAngle;
    turretGyroVel = 0.0;

    //shrouding
    shroudingAngleRadians = shroud->GetDistance();
    shroudingRadPerSec = 0.0;
    shroudingAngleRadiansz = shroudingAngleRadians;

    /*
    //climber
    climbDis = climbDeliver->GetDistance() - CLIMB_EXTEND_OFFSET;
    currentClimbVel = 0.0;
    oldClimbDis = climbDis;
    winchDist = winch->GetDistance();

    confidence = 0.0;

    oldDriveDis = 0.0;
    currentDriveVel = 0.0;

    gyroReadingDRV = driveGyro->GetAngle();
    oldGyroReadingDRV = gyroReadingDRV;
    gyroVelDRV = 0.0;
    */

    //deathstar
    deathStarAngleDegrees = deathStar->GetDistance() - DEATH_STAR_OFFSET;
    deathStarAngleDegreesz = deathStarAngleDegrees;
    deathStarDegreesPerSec = 0.0;
}

void SensorState::StartingConfig()
{
}

void SensorState::StopAll()
{
    LocalReset();
}

void SensorState::UpdateDash()
{
    //shooter
    // frc::SmartDashboard::PutNumber("Shooter Position:", shooterCurrentPos);
    //frc::SmartDashboard::PutNumber("Shooter Radians:", shooterPosRadians);
    //frc::SmartDashboard::PutNumber("shooterCurVel", shooterRadPerSec);

    //shrouding
    frc::SmartDashboard::PutNumber("shroudCurPos", shroudingAngleRadians);
    // frc::SmartDashboard::PutNumber("Shrouding Rad Per Sec:", shroudingRadPerSec);

    //turret
    //frc::SmartDashboard::PutNumber("Turret Analog Angle Raw", turretAnalogAngleRaw);
    frc::SmartDashboard::PutNumber("Turret Analog Angle Mod", turretAnalogAngleMod);
    // frc::SmartDashboard::PutNumber("Turret Analog Vel", turretAnalogVel);
    //frc::SmartDashboard::PutNumber("Turret Gyro Angle", turretGyroAngle);
    frc::SmartDashboard::PutNumber("Turret Gyro Vel", turretGyroVel);

    /*  frc::SmartDashboard::PutNumber("Turret Angle:", turretCurrentAngle);
    frc::SmartDashboard::PutNumber("Turret Radians:", turretAngleRadians);
    frc::SmartDashboard::PutNumber("Turret Rad Per Sec:", turretRadPerSec);
    frc::SmartDashboard::PutNumber("Turret Degrees Per Sec:", turretDegreesPerSec);*/

    /*
    //climber
    frc::SmartDashboard::PutNumber("Climb Extend Pos:", climbDis);
  //  frc::SmartDashboard::PutNumber("Climber Velocity:", currentClimbVel);
   // frc::SmartDashboard::PutNumber("Winch Distance:", winchDist);

    //color wheel
    frc::SmartDashboard::PutNumber("Red", detectedColor.red);
    frc::SmartDashboard::PutNumber("Green", detectedColor.green);
    frc::SmartDashboard::PutNumber("Blue", detectedColor.blue);
    frc::SmartDashboard::PutNumber("Confidence", confidence);
    */
    //frc::SmartDashboard::PutString("color seen:", colorString);

    //drivetrain
    frc::SmartDashboard::PutNumber("Drive Left Drive Dis:", leftWheelDisDrive);
    frc::SmartDashboard::PutNumber("Drive Right Drive Dis:", rightWheelDisDrive);
    frc::SmartDashboard::PutNumber("Drive Average Drive Dis:", averageWheelDisDrive);
    frc::SmartDashboard::PutNumber("Drive Velocity:", currentDriveVel);

    frc::SmartDashboard::PutNumber("Drive Gyro Velocity:", gyroVelDRV);
    frc::SmartDashboard::PutNumber("Drive Gyro Reading:", gyroReadingDRV);
}

void SensorState::Analyze()
{
    mainRobot->xstartsurv = frc::GetTime();
    loopCount++;
    //shooter
    if (shooterFalcon != NULL)
    {

        shooterCurrentPos = ((double)shooterFalcon->GetSelectedSensorPosition());
        shooterPosRadians = shooterCurrentPos * 2 * PI / 2048;
        shooterRadPerSec = shooterFalcon->GetSelectedSensorVelocity() * SHOOTER_VEL_K;
        //printf("%d shootervel %f", loopCount, shooterRadPerSec);
        shooterOldPosRadians = shooterPosRadians;
    }

    //shrouding

    shroudingAngleRadians = shroud->GetDistance();
    //printf("%d shroud pos %f", loopCount, shroudingAngleRadians);
    shroudingRadPerSec = (shroudingAngleRadians - shroudingAngleRadiansz) / LOOPTIME; //(shroudingAngleRadians * 1000)/LOOPTIME;
    shroudingAngleRadiansz = shroudingAngleRadians;

    //turret
    turretAnalogAngleRaw = turretAnalog->GetVoltage() * TURRET_DEGREES_PER_VOLT;
    turretAnalogAngleMod = turretAnalogAngleRaw + TURRET_ANALOG_OFFSET;
    if (turretAnalogAngleMod > TURRET_DEGREE_RANGE)
    {
        turretAnalogAngleMod = turretAnalogAngleMod - TURRET_DEGREE_RANGE;
    }
    turretAnalogVel = (turretAnalogAngleMod - turretAnalogAnglez) / LOOPTIME;
    turretAnalogAnglez = turretAnalogAngleMod;
    turretGyroAngle = turretGyro->GetAngle();
    turretGyroVel = TURRET_GYRO_READING_K * (turretGyroAngle - turretGyroAnglez) / LOOPTIME;
    // printf("%d %f %f \n", loopCount, turretGyroVel, turretGyro->GetRate());
    turretGyroAnglez = turretGyroAngle;
    //printf("%d turret-analograw %f analogmod %f gyroangle %f gyrovel %f", loopCount, turretAnalogAngleRaw, turretAnalogAngleMod, turretGyroAngle, turretGyroVel);

    // turretAngleRadians = turretCurrentAngle * PI/180;
    //  turretRadPerSec = (turretAngleRadians * 1000)/LOOPTIME;
    //turretDegreesPerSec = (turretCurrentAngle-turretOldAngle)/LOOPTIME;

    /*
    //climber
    climbDis = climbDeliver->GetDistance() - CLIMB_EXTEND_OFFSET;
    currentClimbVel = (climbDis - oldClimbDis)/LOOPTIME;
    oldClimbDis = climbDis;
    winchDist = winch->GetDistance();
    // printf("%d %f %f \n", loopCount, climbDis, winchDist);
    gyroReadingCLB = climbGyro->GetAngle();
    gyroVelCLB = (gyroReadingCLB - oldGyroReadingCLB)/LOOPTIME;
    oldGyroReadingCLB = gyroReadingCLB:
    
    //color wheel
    mainRobot->xstartcolor = frc::GetTime();
    detectedColor = m_colorSensor.GetColor();
    mainRobot->xmidcolor = frc::GetTime();
    matchedColor = m_colorMatcher.MatchClosestColor(detectedColor, confidence);
    mainRobot->xendcolor = frc::GetTime();
    if (matchedColor == kBlueTarget) 
    {
       colorChar = 'B';
       colorString = "Blue";
    } 
    else if (matchedColor == kRedTarget) 
    {
        colorChar = 'R';
        colorString = "Red";
    } 
    else if (matchedColor == kGreenTarget) 
    {
        colorChar = 'G';
        colorString = "Green";
    } 
    else if (matchedColor == kYellowTarget) 
    {
        colorChar = 'Y';
        colorString = "Yellow";
    } 
    else 
    {
        colorChar = 'X'; //Unknown
        colorString = "Unknown";
    }
   // printf("%c\n", colorChar);
   */

    //drivetrain
    ReadDriveEncoders();

#ifdef USE_GYRO
    gyroReadingDRV = driveGyro->GetAngle();
    gyroVelDRV = (gyroReadingDRV - oldGyroReadingDRV) / LOOPTIME;
    oldGyroReadingDRV = gyroReadingDRV;
    // printf("%d %f %f %f", loopCount, gyroReadingDRV, gyroVelDRV, driveGyro->GetRate());
#endif
    //printf("%d %f %f %f %f \n", loopCount, )
    //deathstar
    deathStarAngleDegrees = deathStar->GetDistance() - DEATH_STAR_OFFSET;
    deathStarDegreesPerSec = (deathStarAngleDegrees - deathStarAngleDegreesz) / LOOPTIME;
    deathStarAngleDegreesz = deathStarAngleDegrees;

    //  printf("%d %f %f %f %f %f \n", loopCount, GetDriveGyroVelocity(), GetDriveGyroVelDirect(), GetVelGyro_Turret(), GetVelGyro_TurretDir(), turretAnalogAngleMod);
    mainRobot->xendsurv = frc::GetTime();
}

//HAVE TO DO THIS TO INIT
void SensorState::ReadDriveEncoders()
{
    /*if (!(mainRobot->isAuto))
    {
        return;
    }*/
    if ((leftDriveFalcon != NULL) && (rightDriveFalcon != NULL))
    {
        leftRawReading = leftDriveFalcon->GetSelectedSensorPosition();
        rightRawReading = rightDriveFalcon->GetSelectedSensorPosition();
        leftWheelDisDrive = (leftRawReading - leftEncoderOffset) * DRIVE_DISTANCE_PER_PULSE;
        rightWheelDisDrive = -1.0 * (rightRawReading - rightEncoderOffset) * DRIVE_DISTANCE_PER_PULSE;
        averageWheelDisDrive = (rightWheelDisDrive + leftWheelDisDrive) / 2;

        currentDriveVel = (averageWheelDisDrive - oldDriveDis) / LOOPTIME;
        //printf("%d %f %f %f \n", loopCount, leftWheelDisDrive, rightWheelDisDrive, gyroReadingDRV);
        oldDriveDis = averageWheelDisDrive;
    }
}

void SensorState::ResetWinchEncoder()
{
}

void SensorState::ResetClimbExtendEncoder()
{
}

void SensorState::ResetShootingEncoder()
{
    // shooterEncoderOffset = shooterCurrentPos;
}

void SensorState::ResetShroudingEncoder()
{
    shroud->Reset();
}

void SensorState::ResetTurretEncoder()
{
    //turret->Reset();
}

/*void SensorState::ResetDeathStarEncoder()
{
    deathStar->Reset();
}

void SensorState::ResetClimbEncoder()
{
    climb->Reset();
}*/

void SensorState::ResetDriveEncoders()
{
    leftEncoderOffset = leftRawReading;
    rightEncoderOffset = rightRawReading;
    //printf("%d %f %f %f %f resetdrive \n", loopCount, leftWheelDisDrive, leftEncoderOffset, rightWheelDisDrive, rightEncoderOffset);
}

//shooter
void SensorState::InitShooterFalcon(ctre::phoenix::motorcontrol::can::WPI_TalonFX *shooterMotor)
{
    shooterFalcon = shooterMotor;
    shooterFalcon->SetStatusFramePeriod(StatusFrame::Status_2_Feedback0_, 1);
}

//drivetrain
void SensorState::InitDriveFalcons(ctre::phoenix::motorcontrol::can::WPI_TalonFX *leftMotor, ctre::phoenix::motorcontrol::can::WPI_TalonFX *rightMotor)
{
    leftDriveFalcon = leftMotor;
    leftDriveFalcon->SetStatusFramePeriod(StatusFrame::Status_2_Feedback0_, 20);
    rightDriveFalcon = rightMotor;
    rightDriveFalcon->SetStatusFramePeriod(StatusFrame::Status_2_Feedback0_, 20);
}

//deathstar
bool SensorState::GetCellSensor(int num)
{
    switch (num)
    {
    case 1:
        return !cellSensor0->Get();
    case 2:
        return !cellSensor1->Get();
    case 3:
        return !cellSensor2->Get();
    case 4:
        return !cellSensor3->Get();
    case 5:
        return !cellSensor4->Get();
    default:
        return !cellSensor0->Get();
    }
}
