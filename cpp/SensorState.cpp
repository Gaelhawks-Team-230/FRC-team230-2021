
#include "Common.h"
#include "SensorState.h"


SensorState::SensorState(TalonXXI* pRobot)
{
    mainRobot = pRobot;

    //shooter
    shooterFalcon = NULL;
    shooterEncoderOffset = 0.0;

    //shrouding
    shroud = new frc::Encoder(SHROUD_ANGLE_ENCODER_ONE, SHROUD_ANGLE_ENCODER_TWO, false);
    shroud->SetDistancePerPulse(ANGLE_DISTANCE_PER_PULSE);

    //turret
   /* turret = new frc::Encoder(TURRET_ANGLE_ENCODER_ONE, TURRET_ANGLE_ENCODER_TWO, false);
    turret->SetDistancePerPulse(SPIN_DISTANCE_PER_PULSE);*/
    turretAnalog = new frc::AnalogInput(TURRET_ANALOG);
    turretGyro = new frc::AnalogGyro(TURRET_GYRO_ANALOG);
   // turretGyro = new frc::ADXRS450_Gyro(frc::SPI::kOnboardCS0);

    //climber
    //climbGyro = new frc::ADXRS450_Gyro(frc::SPI::kOnboardCS0);
    climbDeliver = new frc::DutyCycleEncoder(CLIMBER_DUTY_CYCLE);
    winch = new frc::Encoder(WINCH_ENCODER_ONE, WINCH_ENCODER_TWO, false);
    winch->SetDistancePerPulse(WINCH_DISTANCE_PER_PULSE);

   
    //color wheel
    m_colorMatcher.AddColorMatch(kBlueTarget);
    m_colorMatcher.AddColorMatch(kGreenTarget);
    m_colorMatcher.AddColorMatch(kRedTarget);
    m_colorMatcher.AddColorMatch(kYellowTarget);

    //drivetrain
    leftDriveFalcon = NULL;
    rightDriveFalcon = NULL;
    
#ifdef USE_GYRO
    driveGyro = new frc::ADXRS450_Gyro(frc::SPI::kOnboardCS0);
#endif

    //deathstar
    deathStar = new frc::DutyCycleEncoder(DIGIN_DEATH_STAR_DUTY_CYCLE);
    deathStar->SetDistancePerRotation(DEATH_STAR_DISTANCE_PER_ROTATION);

    cellSensor1 = new frc::DigitalInput(DIGIN_DEATH_STAR_1);
    cellSensor2 = new frc::DigitalInput(DIGIN_DEATH_STAR_2);
    cellSensor3 = new frc::DigitalInput(DIGIN_DEATH_STAR_3);
    cellSensor4 = new frc::DigitalInput(DIGIN_DEATH_STAR_4);
    cellSensor5 = new frc::DigitalInput(DIGIN_DEATH_STAR_5);
    
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
    shooterRPM = 0.0;

    //turret
    turretAnalogAngleRaw = turretAnalog->GetVoltage()*TURRET_DEGREES_PER_VOLT;
    turretAnalogAngleMod = turretAnalogAngleRaw + TURRET_ANALOG_OFFSET;
    if(turretAnalogAngleMod > TURRET_DEGREE_RANGE)
    {
        turretAnalogAngleMod = turretAnalogAngleMod - TURRET_DEGREE_RANGE;
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
    shroudingCurrentAngle = 0.0;
    shroudingDegreesPerSec = 0.0;
    shroudingOldAngle = 0.0;

    //climber
    climbDis = 0.0;
    oldClimbDis = 0.0;
    currentClimbVel = 0.0;
    winchDist = 0.0;

    gyroReadingCLB = 0.0;
    oldGyroReadingCLB = 0.0;
    gyroVelCLB = 0.0;
        
    confidence = 0.0;

    oldDriveDis = 0.0;
    currentDriveVel = 0.0;

    gyroReadingDRV = driveGyro->GetAngle();
    oldGyroReadingDRV = gyroReadingDRV;
    gyroVelDRV = 0.0;
    
    //deathstar
    deathStarCurrentAngle = 0.0;
    deathStarAngleRadians = 0.0;
    deathStarAngleDegrees = deathStar->GetDistance() - DEATH_STAR_OFFSET;
    printf("sensorstate local reset %f \n", deathStarAngleDegrees);
    deathStarAngleDegreesz = deathStarAngleDegrees;
    deathStarRadPerSec = 0.0;
    deathStarDegreesPerSec = 0.0;
    deathStarOldAngle = 0.0;
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
    frc::SmartDashboard::PutNumber("Shooter Position:", shooterCurrentPos);
    frc::SmartDashboard::PutNumber("Shooter Radians:", shooterPosRadians);
    frc::SmartDashboard::PutNumber("Shooter Rad Per Sec:", shooterRadPerSec);
    frc::SmartDashboard::PutNumber("Shooter RPM:", shooterRPM);

    //shrouding
    frc::SmartDashboard::PutNumber("Shrouding Angle:", shroudingCurrentAngle);
    frc::SmartDashboard::PutNumber("Shrouding Radians:", shroudingAngleRadians);
    frc::SmartDashboard::PutNumber("Shrouding Rad Per Sec:", shroudingRadPerSec);
    frc::SmartDashboard::PutNumber("Shrouding Degrees Per Sec:", shroudingDegreesPerSec);

    //turret
    frc::SmartDashboard::PutNumber("Turret Analog Angle Raw", turretAnalogAngleRaw);
    frc::SmartDashboard::PutNumber("Turret Analog Angle Mod", turretAnalogAngleMod);
    frc::SmartDashboard::PutNumber("Turret Analog Vel", turretAnalogVel);
    frc::SmartDashboard::PutNumber("Turret Gyro Angle", turretGyroAngle);
    frc::SmartDashboard::PutNumber("Turret Gyro Vel", turretGyroVel);
    
  /*  frc::SmartDashboard::PutNumber("Turret Angle:", turretCurrentAngle);
    frc::SmartDashboard::PutNumber("Turret Radians:", turretAngleRadians);
    frc::SmartDashboard::PutNumber("Turret Rad Per Sec:", turretRadPerSec);
    frc::SmartDashboard::PutNumber("Turret Degrees Per Sec:", turretDegreesPerSec);*/

    //climber
    frc::SmartDashboard::PutNumber("Climber Distance:", climbDis);
    frc::SmartDashboard::PutNumber("Climber Velocity:", currentClimbVel);
    frc::SmartDashboard::PutNumber("Winch Distance:", winchDist);
    //frc::SmartDashboard::PutNumber("Climber Gyro Reading:", gyroReadingCLB);
    //frc::SmartDashboard::PutNumber("Climber Velocity:", gyroVelCLB);

    //color wheel
    frc::SmartDashboard::PutNumber("Red", detectedColor.red);
    frc::SmartDashboard::PutNumber("Green", detectedColor.green);
    frc::SmartDashboard::PutNumber("Blue", detectedColor.blue);
    frc::SmartDashboard::PutNumber("Confidence", confidence);

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
    loopCount++;
    //shooter
    if (shooterFalcon != NULL)
    { 
        
        shooterCurrentPos = ((double) shooterFalcon->GetSelectedSensorPosition()) - shooterEncoderOffset;
        shooterPosRadians = shooterCurrentPos*2*PI/2048;
        //shooterRadPerSec = (shooterPosRadians - shooterOldPosRadians)/LOOPTIME;
        shooterRadPerSec = shooterFalcon->GetSelectedSensorVelocity()*10.0*2.0*PI/2048;
        //printf("%d shootervel %f", loopCount, shooterRadPerSec);
        shooterOldPosRadians = shooterPosRadians;
    }

    //shrouding
    
    //shroudingCurrentAngle = shroud->GetDistance();
    //shroudingAngleRadians = shroudingCurrentAngle *2*PI/2048;
    shroudingAngleRadians = shroud->GetDistance();
    //printf("%d shroud pos %f", loopCount, shroudingAngleRadians);
    shroudingRadPerSec = (shroudingAngleRadians - shroudingAngleRadiansz)/LOOPTIME;  //(shroudingAngleRadians * 1000)/LOOPTIME;
    shroudingAngleRadiansz = shroudingAngleRadians;
    //shroudingDegreesPerSec = (shroudingCurrentAngle-shroudingOldAngle)/LOOPTIME;
    
    //turret
    //turretCurrentAngle = turret->GetRate();
    //turretCurrentAngle = turret->GetVoltage();
   // turretAnalogAngle = turretAnalog->GetVoltage()*TURRET_DEGREES_PER_VOLT;
    turretAnalogAngleRaw = turretAnalog->GetVoltage()*TURRET_DEGREES_PER_VOLT;
    turretAnalogAngleMod = turretAnalogAngleRaw + TURRET_ANALOG_OFFSET;
    if(turretAnalogAngleMod > TURRET_DEGREE_RANGE)
    {
        turretAnalogAngleMod = turretAnalogAngleMod - TURRET_DEGREE_RANGE;
    }
    turretAnalogVel = (turretAnalogAngleMod - turretAnalogAnglez)/LOOPTIME;
    turretAnalogAnglez = turretAnalogAngleMod;
    turretGyroAngle = turretGyro->GetAngle();
    turretGyroVel = TURRET_GYRO_READING_K*(turretGyroAngle - turretGyroAnglez)/LOOPTIME;
   // printf("%d %f %f \n", loopCount, turretGyroVel, turretGyro->GetRate());
    turretGyroAnglez = turretGyroAngle;
    //printf("%d turret-analograw %f analogmod %f gyroangle %f gyrovel %f", loopCount, turretAnalogAngleRaw, turretAnalogAngleMod, turretGyroAngle, turretGyroVel);
   
   // turretAngleRadians = turretCurrentAngle * PI/180;
  //  turretRadPerSec = (turretAngleRadians * 1000)/LOOPTIME;
    //turretDegreesPerSec = (turretCurrentAngle-turretOldAngle)/LOOPTIME;

    //climber
    climbDis = climbDeliver->GetDistance();
    currentClimbVel = (climbDis - oldClimbDis)/LOOPTIME;
    oldClimbDis = climbDis;
    winchDist = winch->GetDistance();
    // printf("%d %f %f \n", loopCount, climbDis, winchDist);
    /* gyroReadingCLB = climbGyro->GetAngle();
    gyroVelCLB = (gyroReadingCLB - oldGyroReadingCLB)/LOOPTIME;
    oldGyroReadingCLB = gyroReadingCLB;*/

    //color wheel
    detectedColor = m_colorSensor.GetColor();
    matchedColor = m_colorMatcher.MatchClosestColor(detectedColor, confidence);
    if (matchedColor == kBlueTarget) 
    {
       colorChar = 'B';
    } 
    else if (matchedColor == kRedTarget) 
    {
        colorChar = 'R';
    } 
    else if (matchedColor == kGreenTarget) 
    {
        colorChar = 'G';
    } 
    else if (matchedColor == kYellowTarget) 
    {
        colorChar = 'Y';
    } 
    else 
    {
        colorChar = 'X'; //Unknown
    }
   // printf("%c\n", colorChar);

    //drivetrain
    ReadDriveEncoders();

#ifdef USE_GYRO
   

    gyroReadingDRV = driveGyro->GetAngle();
    gyroVelDRV = (gyroReadingDRV - oldGyroReadingDRV)/LOOPTIME;
    oldGyroReadingDRV = gyroReadingDRV;
   // printf("%d %f %f %f", loopCount, gyroReadingDRV, gyroVelDRV, driveGyro->GetRate());
#endif
    //printf("%d %f %f %f %f \n", loopCount, )
    //deathstar
    deathStarAngleDegrees = deathStar->GetDistance() - DEATH_STAR_OFFSET;
    deathStarDegreesPerSec = (deathStarAngleDegrees-deathStarAngleDegreesz)/LOOPTIME;
    deathStarAngleDegreesz = deathStarAngleDegrees;
    //deathStarAngleRadians = deathStarCurrentAngle * PI/180;
    //deathStarRadPerSec = (deathStarAngleRadians * 1000)/LOOPTIME;

  //  printf("%d %f %f %f %f %f \n", loopCount, GetDriveGyroVelocity(), GetDriveGyroVelDirect(), GetVelGyro_Turret(), GetVelGyro_TurretDir(), turretAnalogAngleMod);
    
}

//HAVE TO DO THIS TO INIT
void SensorState::ReadDriveEncoders()
{
    
    if ((leftDriveFalcon != NULL) && (rightDriveFalcon != NULL))
    {
        leftRawReading = leftDriveFalcon->GetSelectedSensorPosition();
        rightRawReading = rightDriveFalcon->GetSelectedSensorPosition();
        leftWheelDisDrive = (leftRawReading - leftEncoderOffset) * DRIVE_DISTANCE_PER_PULSE;
        rightWheelDisDrive = -1.0* (rightRawReading - rightEncoderOffset) * DRIVE_DISTANCE_PER_PULSE; 
        averageWheelDisDrive = (rightWheelDisDrive + leftWheelDisDrive)/2;

        currentDriveVel = (averageWheelDisDrive - oldDriveDis)/LOOPTIME;
        oldDriveDis = averageWheelDisDrive;
        //printf("%d %f %f %f %f %f %f readdrive \n", loopCount, leftRawReading, leftEncoderOffset, leftWheelDisDrive, rightRawReading, rightEncoderOffset, rightWheelDisDrive);
    }
}

void SensorState::ResetShootingEncoder()
{
    shooterEncoderOffset = shooterCurrentPos;
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
void SensorState::InitDriveFalcons (ctre::phoenix::motorcontrol::can::WPI_TalonFX *leftMotor, ctre::phoenix::motorcontrol::can::WPI_TalonFX *rightMotor)
{
    leftDriveFalcon = leftMotor;
    leftDriveFalcon->SetStatusFramePeriod(StatusFrame::Status_2_Feedback0_, 20);
    rightDriveFalcon = rightMotor;
    rightDriveFalcon->SetStatusFramePeriod(StatusFrame::Status_2_Feedback0_, 20);
}


//deathstar
bool SensorState::GetCellSensor(int num)
{
    switch(num)
    {
        case 1:
            return !cellSensor1->Get();
        case 2:
            return !cellSensor2->Get();
        case 3:
            return !cellSensor3->Get();
        case 4:
            return !cellSensor4->Get();
        case 5:
            return !cellSensor5->Get();
        default:
            return !cellSensor1->Get();
    }
}
