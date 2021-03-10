#include "TalonXXI_main.h"
#include "Common.h"
#include "frc/smartdashboard/Smartdashboard.h" 
#include "ShooterTurret.h"

ShooterTurret::ShooterTurret(TalonXXI* pRobot)
{
    mainRobot = pRobot;
    turretMotor = new frc::VictorSP(PWM_TURRET);
    localSurveillance = mainRobot->surveillance;
    turretAnalogOffsetZero = 0.0;
    turretAnalogPosOffset = 0.0;
    currentTiltAdjust = 0.0;

    LocalReset();
}

void ShooterTurret::LocalReset()
{
    loopCount = 0;
    turretCmd = 0.0;
   
    turretAnalogPos = localSurveillance->GetAngleAnalog_Turret() - turretAnalogPosOffset;//
    turretAnalogVel = localSurveillance->GetVelAnalog_Turret();
    turretGyroPos = localSurveillance->GetAngleGyro_Turret();
    turretGyroVel = localSurveillance->GetVelGyro_Turret();
    turretGoalPos = turretAnalogPos;
    turretPCmd = turretAnalogPos;
    turretPosError = 0.0;

    robotBaseVel = localSurveillance->GetDriveGyroVelocity();
    robotBaseVelz = robotBaseVel;
    robotBaseAccel = 0.0;
 
    turretVelCmd = 0.0;
    turretVelErr = 0.0;
    turretErrInt = 0.0;
    isTurretAtGoal = true;
    isTurretOverride = false;
    isTurretManual = false;

    isLimelightTracking = false;
 
    seesTarget = mainRobot->limelight->SeesShooterTarget();
    targetPos = mainRobot->limelight->GetTargetHOffset();

    turretMotor->Set(turretCmd); 
}

void ShooterTurret::StartingConfig()
{
    GiveGoalAngle(TURRET_FACE_BACK_POS);
}

void ShooterTurret::StopAll()
{
    LocalReset();
}

//Control System always running unless is in override
void ShooterTurret::TurretControl()
{    
    if(isLimelightTracking)
    {
        if(seesTarget)
        {
            turretVelCmd = (targetPos + TURRET_HOR_TAR_BIAS) * TURRET_K_TRACKING;
            turretVelCmd = TalonXXI::Limit(-TURRET_VEL_CMD_LIMIT, TURRET_VEL_CMD_LIMIT, turretVelCmd);
        }
        else
        {
            turretVelCmd = mainRobot->userInput->GetGamepadAxis(TURRET_AXIS) * TURRET_K_VEL_JOYSTICK;
        }
        turretVelErr = turretVelCmd - turretGyroVel;

        turretErrInt = turretErrInt + (turretVelErr*LOOPTIME);
        turretErrInt = TalonXXI::Limit(TURRET_MIN_VELOCITY_ERROR_INT, TURRET_MAX_VELOCITY_ERROR_INT, turretErrInt);
        turretCmd = TURRET_K_BANDWIDTH/TURRET_K * (TURRET_TAU * turretVelErr + turretErrInt);
        turretCmd = turretCmd - (robotBaseAccel*TURRET_TAU + robotBaseVel)/TURRET_K;
    }
    else
    {
        if(fabs(mainRobot->userInput->GetGamepadAxis(TURRET_AXIS)) > TURRET_NO_TRACK_JOYSTICK_DEADBAND)
        {
            //turretPCmd = turretPCmd + mainRobot->userInput->GetGamepadAxis(0) * 0.5;
            turretGoalPos = turretGoalPos + mainRobot->userInput->GetGamepadAxis(TURRET_AXIS) * TURRET_NO_TRACK_JOYSTICK_K;
            turretGoalPos = TalonXXI::Limit(TURRET_MIN_ANGLE, TURRET_MAX_ANGLE, turretGoalPos);
        }
        turretPCmd = turretPCmd + TalonXXI::Limit(-TURRET_MAX_DELTA_POS, TURRET_MAX_DELTA_POS, turretGoalPos - turretPCmd);//NEW
        turretPosError = turretPCmd - turretAnalogPos;
        turretPosError = TalonXXI::Limit(-TURRET_NO_TRACK_POS_ERR, TURRET_NO_TRACK_POS_ERR, turretPosError);
        turretCmd = turretPosError * TURRET_KP_NO_TRACKING;
        //turretCmd = turretCmd + 0.01*TalonXXI::Sign(turretCmd);
        //turretCmd = sqrt(fabs(turretCmd))*TalonXXI::Sign(turretCmd);
    }
    if(turretAnalogPos < TURRET_MIN_ANGLE || turretAnalogPos > TURRET_MAX_ANGLE)
    {
        if(turretAnalogPos > TURRET_MID_DEADBAND)
        {
            turretCmd = -TURRET_IN_DEADBAND_CMD;
        }
        else
        {
            turretCmd = TURRET_IN_DEADBAND_CMD;
        }
    }
    
}

void ShooterTurret::UpdateDash()
{
    frc::SmartDashboard::PutNumber("turretCmd", turretCmd);
    frc::SmartDashboard::PutNumber("turretGoalPos", turretGoalPos);
    //frc::SmartDashboard::PutBoolean("isTurretManual", isTurretManual);
}

void ShooterTurret::Service()
{
    loopCount++;
    turretAnalogPos = localSurveillance->GetAngleAnalog_Turret() - turretAnalogPosOffset;
    turretAnalogVel = localSurveillance->GetVelAnalog_Turret();
    seesTarget = mainRobot->limelight->SeesShooterTarget();
    targetPos = mainRobot->limelight->GetTargetHOffset();
    turretGyroPos = localSurveillance->GetAngleGyro_Turret();
    turretGyroVel = localSurveillance->GetVelGyro_Turret();
    robotBaseVel = localSurveillance->GetDriveGyroVelocity();
    robotBaseAccel = (robotBaseVel - robotBaseVelz)/LOOPTIME;

#ifndef TEST_MODE
    TurretControl();
#endif
   // printf("%d %f %f %f %f %f \n", loopCount, turretCmd, turretAnalogPos, turretGoalPos, turretAnalogPosOffset, turretAnalogOffsetZero);
    turretCmd = TalonXXI::Limit(TURRET_MIN_CMD, TURRET_MAX_CMD, turretCmd);
    turretMotor->Set(turretCmd);
    robotBaseVelz = robotBaseVel;
}

void ShooterTurret::SetTargetingValues()
{
    isLimelightTracking = true; 
    mainRobot->limelight->TurnOnLED();  
}

bool ShooterTurret::IsLimelightTracking()
{
    return isLimelightTracking;
}

void ShooterTurret::StopTargeting()
{
    if(isLimelightTracking)
    {
         isLimelightTracking = false;
         mainRobot->limelight->TurnOffLED();
    //turretPCmd = turretAnalogPos;
        turretGoalPos = turretAnalogPos;
    }
}

void ShooterTurret::ResetTurretDialAdjust(double reading)
{
    turretAnalogOffsetZero = reading;
}

void ShooterTurret::TurretDialAdjust(double reading)
{
    currentTiltAdjust = reading;
    turretAnalogPosOffset = TURRET_ADJUST_K*(currentTiltAdjust - turretAnalogOffsetZero);
}

void ShooterTurret::TurretStop()
{
    LocalReset();
}

//TURRET BASIC FUNCTIONS

bool ShooterTurret::IsInTrackingRange()
{
    if(fabs(turretAnalogPos - turretGoalPos) < LIMELIGHT_START_TRACKING_RANGE)
    {
        return true;
    }
    return false;
}

bool ShooterTurret::IsReadyToShoot()
{
    //if(isLimelightTracking && (fabs(turretAnalogPos - turretGoalPos) < LIMELIGHT_AT_TARGET_RANGE))
    if(isLimelightTracking && targetPos < LIMELIGHT_AT_TARGET_RANGE)
    {
        return true;
    }
    return false;
}

void ShooterTurret::GiveGoalAngle(double inputAngle)
{
   // turretPCmd = inputAngle;
    turretGoalPos = inputAngle;
    isLimelightTracking = false;
    isTurretManual = false;
}

void ShooterTurret::TurnOnTurretOverride()
{
    isTurretOverride = true;
}

void ShooterTurret::TurnOffTurretOverride()
{
    isTurretOverride = false;
    LocalReset();
}

double ShooterTurret::GiveTurretPos()
{
    //turretCurPos = localSurveillance->GetAngle_Turret();
    turretAnalogPos = localSurveillance->GetAngleAnalog_Turret();
    return turretAnalogPos;
}

bool ShooterTurret::IsTurretAtGoal()
{
    return (fabs(turretAnalogPos - turretGoalPos) < TURRET_AT_GOAL_ALLOWANCE);
}

bool ShooterTurret::IsTurretOverride()
{
    return isTurretOverride;
}

double ShooterTurret::GiveTurretGoal()
{
    return turretGoalPos;
}

#ifdef TEST_MODE
void ShooterTurret::Testing(double input)
{
    if(fabs(input) > 0.1)
    {
        turretCmd = input;
    }
    else
    {
        turretCmd = 0.0;
    }
    turretCmd = TalonXXI::Limit(-0.3, 0.3, turretCmd);
}
#endif