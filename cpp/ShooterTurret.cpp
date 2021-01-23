#include "TalonXXI_main.h"
#include "Common.h"
#include "frc/smartdashboard/Smartdashboard.h" 
#include "ShooterTurret.h"

ShooterTurret::ShooterTurret(TalonXXI* pRobot)
{
    mainRobot = pRobot;
    turretMotor = new frc::VictorSP(PWM_TURRET);
    localSurveillance = mainRobot->surveillance;
      
    LocalReset();
    //isTurretCalibrating = true;
}

void ShooterTurret::LocalReset()
{
    loopCount = 0;
    turretCmd = 0.0;
   
    turretAnalogPos = localSurveillance->GetAngleAnalog_Turret();//
    turretAnalogVel = localSurveillance->GetVelAnalog_Turret();
    turretGyroPos = localSurveillance->GetAngleGyro_Turret();
    turretGyroVel = localSurveillance->GetVelGyro_Turret();
    turretGoalPos = turretAnalogPos;
    turretPCmd = turretAnalogPos;
    turretPosError = 0.0;

    robotBaseVel = localSurveillance->GetDriveGyroVelocity();
    robotBaseVelz = robotBaseVel;
    robotBaseAccel = 0.0;
    //turretAnalogPosz = turretAnalogPos;
    //turretAnalogVel = (turretAnalogPos - turretAnalogPosz)/LOOPTIME;
    
   // turretCurPos = turretGyro->GetAngle();
    //turretPosz = turretCurPos;
    
    //turretCurVel = 0.0;
    //turretCurVel = localSurveillance->GetAngleRadPerSec_Turret();
    
    turretVelCmd = 0.0;
    turretVelErr = 0.0;
    turretErrInt = 0.0;
    isTurretAtGoal = true;
    isTurretOverride = false;
    isTurretManual = false;

    isLimelightTracking = false;
 
    seesTarget = mainRobot->limelight->SeesShooterTarget();
    targetPos = mainRobot->limelight->GetTargetHOffset();
   // targetPos = 0.0;

    turretMotor->Set(turretCmd);
 
}

void ShooterTurret::StartingConfig()
{

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
            turretVelCmd = targetPos*TURRET_K_TRACKING;
            turretVelCmd = TalonXXI::Limit(-TURRET_VEL_CMD_LIMIT, TURRET_VEL_CMD_LIMIT, turretVelCmd);
        }
        else
        {
            turretVelCmd = mainRobot->userInput->GetGamepadAxis(TURRET_AXIS) * TURRET_K_VEL_JOYSTICK;
        }
        //turretVelCmd = 0.0;
        turretVelErr = turretVelCmd - turretGyroVel;

        turretErrInt = turretErrInt + (turretVelErr*LOOPTIME);
        turretErrInt = TalonXXI::Limit(TURRET_MIN_VELOCITY_ERROR_INT, TURRET_MAX_VELOCITY_ERROR_INT, turretErrInt);
        turretCmd = TURRET_K_BANDWIDTH/TURRET_K * (TURRET_TAU * turretVelErr + turretErrInt);
        turretCmd = turretCmd - (robotBaseAccel*TURRET_TAU + robotBaseVel)/TURRET_K;
    }
    else
    {
        //printf("robot frame ");
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
    if(turretAnalogPos < TURRET_MIN_ANGLE && turretAnalogPos > TURRET_MAX_ANGLE)
    {
        if(turretAnalogPos > TURRET_MID_DEADBAND)
        {
            turretCmd = TURRET_IN_DEADBAND_CMD;
        }
        else
        {
            turretCmd = -TURRET_IN_DEADBAND_CMD;
        }
    }
    
}

void ShooterTurret::UpdateDash()
{
    //frc::SmartDashboard::PutBoolean("isTurretCalibrating", isTurretCalibrating);
    frc::SmartDashboard::PutNumber("turretCmd", turretCmd);
    frc::SmartDashboard::PutNumber("turretGoalPos", turretGoalPos);
    frc::SmartDashboard::PutBoolean("isTurretManual", isTurretManual);
    frc::SmartDashboard::PutBoolean("isTurretOverride", isTurretOverride);
}

void ShooterTurret::Service()
{
    loopCount++;
    turretAnalogPos = localSurveillance->GetAngleAnalog_Turret();
    turretAnalogVel = localSurveillance->GetVelAnalog_Turret();
//#ifndef TEST_DRIVE
   //seesTarget = mainRobot->limelight->GetTargetVisibility();
   seesTarget = mainRobot->limelight->SeesShooterTarget();
   targetPos = mainRobot->limelight->GetTargetHOffset();
//#endif
   turretGyroPos = localSurveillance->GetAngleGyro_Turret();
   turretGyroVel = localSurveillance->GetVelGyro_Turret();
   robotBaseVel = localSurveillance->GetDriveGyroVelocity();
   robotBaseAccel = (robotBaseVel - robotBaseVelz)/LOOPTIME;

    //if(!isTurretOverride)
    //{
#ifndef TEST_MODE
    TurretControl();
#endif
    //printf("%d %f %f %f %f \n", loopCount, turretCmd, turretGoalPos, turretAnalogPos, turretPosError);
    //}
    //printf("%d %f %f \n", loopCount, turretCmd, turretAnalogPos);
    //printf("%d %f %f %f %f %f %f %f %f %f \n", loopCount, turretCmd, turretGyroVel, turretVelCmd, turretPCmd,turretAnalogPos, turretPosError, seesTarget, targetPos);
    turretCmd = TalonXXI::Limit(TURRET_MIN_CMD, TURRET_MAX_CMD, turretCmd);
    turretMotor->Set(turretCmd);
    robotBaseVelz = robotBaseVel;
}

void ShooterTurret::SetTargetingValues()
{
    isLimelightTracking = true; 
    mainRobot->limelight->TurnOnLED();  
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

/*void ShooterTurret::Recalibrate()
{
    isTurretCalibrating = true;
}*/

void ShooterTurret::TurretStop()
{
    LocalReset();
}

void ShooterTurret::ManualTurretMove(double axisReading)
{
    
    //turretCmd = axisReading*1.0;
    turretVelCmd = axisReading;
    /*if(fabs(axisReading) > TURRET_MANUAL_DEADBAND)
    { 
        if(isTurretOverride)
        {
            turretCmd = axisReading * TURRET_OVERRIDE_CONSTANT;
        }
        else
        {
            if(!isTurretManual)
            {
                turretGoalPos = turretCurPos;
                isTurretManual = true;
            }
            turretGoalPos = turretGoalPos + axisReading*TURRET_MANUAL_DELTA;
        }
    }*/
}

//TURRET BASIC FUNCTIONS

bool ShooterTurret::IsInTrackingRange()
{
    if(isLimelightTracking && (fabs(turretAnalogPos - turretGoalPos) < LIMELIGHT_START_TRACKING_RANGE))
    {
        return true;
    }
    return false;
}

bool ShooterTurret::IsReadyToShoot()
{
    if(isLimelightTracking && (fabs(turretAnalogPos - turretGoalPos) < LIMELIGHT_AT_TARGET_RANGE))
    {
        return true;
    }
    return false;
}

void ShooterTurret::GiveGoalAngle(double inputAngle)
{
   // turretPCmd = inputAngle;
    turretGoalPos = inputAngle;
   // turretGoalPos = 180.0;
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

/*bool ShooterTurret::IsTurretCalibrating()
{
    return isTurretCalibrating;
}*/

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

bool ShooterTurret::TurretAtMax()
{
    return (turretAnalogPos > (TURRET_MAX_ANGLE - TURRET_AT_GOAL_ALLOWANCE));
}

bool ShooterTurret::TurretAtMin()
{
    return (turretAnalogPos < (TURRET_MIN_ANGLE + TURRET_AT_GOAL_ALLOWANCE));
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