//#ifdef BALLSHOOTER_H_
#include "TalonXXI_main.h"
#include "Common.h"
#include "frc/smartdashboard/Smartdashboard.h" 
#include "BallShooter.h"
#include "ctre/phoenix/motorcontrol/TalonFXSensorCollection.h"
#include "ctre/phoenix/motorcontrol/can/WPI_TalonFX.h"
#include "ctre/phoenix/motorcontrol/StatorCurrentLimitConfiguration.h"


BallShooter::BallShooter(TalonXXI* pRobot)
{
    mainRobot = pRobot;
    localSurveillance = mainRobot->surveillance;
   // shooterMotor = new ctre::phoenix::motorcontrol::can::WPI_TalonFX(7); //- prototype
    shooterMotor = new ctre::phoenix::motorcontrol::can::WPI_TalonFX(CAN_SHOOTER); //Real Shooter

    localSurveillance->InitShooterFalcon(shooterMotor);
    shooterMotor->SetNeutralMode(NeutralMode::Brake);
    shooterSupplyCurrent.currentLimit = SHOOTER_SUPPLY_CUR_LIMIT;
    shooterSupplyCurrent.enable = true;
    shooterMotor->ConfigSupplyCurrentLimit(shooterSupplyCurrent);
    shooterStatorCurrent.currentLimit = SHOOTER_STATOR_CUR_LIMIT;
    shooterStatorCurrent.enable = true;
    shooterMotor->ConfigStatorCurrentLimit(shooterStatorCurrent);
    
    shroudMotor = new frc::PWMTalonSRX(PWM_SHROUDING);

#ifndef TEST_MODE
    mainRobot->userInput->SetGamepadAxisParameter(TURRET_AXIS, 0.1, 0.7);
    mainRobot->userInput->SetGamepadAxisParameter(SHROUD_AXIS, 0.3, 0.7);
#endif

    LocalReset();
    isShroudCalibrating = true;
    shroudCalState = 0;
}

//Sets all local variables
void BallShooter::LocalReset()
{
    ShooterReset();
    ShroudReset();    
    seesTarget = false;
    targetYPos = 0.0;
    loopCount = 0;
}

void BallShooter::ShooterReset()
{
    shooterCmd = 0.0;
    shooterCurPos = localSurveillance->GetPosRadians_Shooter();
    shooterCurVel = localSurveillance->GetPosRadPerSec_Shooter();
    shooterCurPosz = shooterCurPos;
    shooterCurPoszz = shooterCurPos;
    shooterCurVelz = shooterCurVel;
    shooterCurVelzz = shooterCurVel;
    shooterCurVelzzz = shooterCurVel;
    shooterVelOutput = shooterCurVel;
    shooterVelOutputz = shooterVelOutput;
    shooterVelOutputzz = shooterVelOutputz;
    shooterGoalVel = 0.0;
    shooterVelCmd = 0.0;
    shooterErr = 0.0;
    shooterErrInt = 0.0;
    shooterErrIntz = shooterErrInt;
    shooterErrIntzz = shooterErrInt;
    shooterErrIntzzz = shooterErrInt;
    shooterErrIntzzzz = shooterErrInt;

    errCount = 0;

    isShooterAtGoal = true;
    shooterMotor->Set(ControlMode::PercentOutput,0.0);
}

void BallShooter::ShroudReset()
{
    shroudCmd = 0.0;
    shroudCurPos = localSurveillance->GetAngleRadians_Shrouding();
    shroudCurPosz = shroudCurPos;
    shroudGoalPos = shroudCurPos;
    shroudPCmd = shroudCurPos;
    shroudPosError = 0.0;
    shroudCurVel = 0.0;
    shroudVelCmd = 0.0;
    shroudVelErr = 0.0;
    shroudErrInt = 0.0;
    ticSum = 0;
    isShroudAtGoal = true;
    isShroudManual = false;
    shroudMotor->Set(shroudCmd);
}

void BallShooter::StartingConfig()
{
    GiveShooterGoalVel(0.0);
    GiveShroudGoalAngle(SHROUD_MIN_ANGLE);
}

void BallShooter::StopAll()
{
    LocalReset();
}

void BallShooter::UpdateDash()
{
    //Shooter
    //frc::SmartDashboard::PutNumber("shooterCmd", shooterCmd);
    frc::SmartDashboard::PutNumber("shooterCurVel", shooterCurVel);
    frc::SmartDashboard::PutNumber("shootergoalVel", shooterGoalVel);
    //double shooterCurrentSupply = shooterMotor->GetSupplyCurrent();
    //double shooterCurrentOutput = shooterMotor->GetStatorCurrent();
    //frc::SmartDashboard::PutNumber("Shooter Supply current", shooterCurrentSupply);
    //frc::SmartDashboard::PutNumber("Shooter Output Current", shooterCurrentOutput);
    //Shroud
    //frc::SmartDashboard::PutBoolean("isShroudCalibrating", isShroudCalibrating);
    frc::SmartDashboard::PutNumber("shroudCmd", shroudCmd);
   // frc::SmartDashboard::PutNumber("shroudCurPos", shroudCurPos);
    frc::SmartDashboard::PutNumber("shroudGoalPos", shroudGoalPos);
    //frc::SmartDashboard::PutBoolean("isShroudManual", isShroudManual);
}

void BallShooter::Service()
{
    loopCount++;
    seesTarget = mainRobot->limelight->SeesShooterTarget();
    targetYPos = mainRobot->limelight->GetTargetVOffset();
 
    shooterCurPos = localSurveillance->GetPosRadians_Shooter();
    shooterCurVel = localSurveillance->GetPosRadPerSec_Shooter();
    shooterVelOutput = SHOOTER_LEAD_FILTER_C*shooterVelOutputz + SHOOTER_LEAD_FILTER_A*shooterCurVel + SHOOTER_LEAD_FILTER_B*shooterCurVelz;
    
    shroudCurPos = localSurveillance->GetAngleRadians_Shrouding();
    shroudCurVel = (shroudCurPos - shroudCurPosz)/LOOPTIME;
       
#ifndef TEST_MODE
    ShooterControl();
    if(isShroudCalibrating)
    {
        ShroudCalibrate();
    }
    else
    {
        ShroudControl();
    }
#endif
   
    shooterCmd = TalonXXI::Limit(SHOOTER_MIN_CMD, SHOOTER_MAX_CMD, shooterCmd);
    shroudCmd = TalonXXI::Limit(SHROUD_MIN_CMD, SHROUD_MAX_CMD, shroudCmd);
    shooterMotor->Set(ControlMode::PercentOutput, (shooterCmd));
    shroudMotor->Set(shroudCmd);
    shroudCurPosz = shroudCurPos;
}

void BallShooter::ShooterControl()
{
   // shooterCurVel = TalonXXI::Limit(SHOOTER_MIN_VELOCITY, SHOOTER_MAX_VELOCITY, shooterVelCmd);
    shooterVelCmd = shooterVelCmd + TalonXXI::Limit(-1*SHOOTER_MAX_DELTA_VEL, SHOOTER_MAX_DELTA_VEL, shooterGoalVel - shooterVelCmd);
    shooterErr = shooterVelCmd - shooterVelOutput;
    if((shooterVelOutput - shooterVelOutputzz) < SHOOTER_DELTA_VEL)
    {
        errCount = SHOOTER_HOLD_TIME;
        shooterErrInt = shooterErrIntzzzz;
    }
    if(errCount > 0)
    {
        errCount--;
        shooterErrInt = shooterErrIntz = shooterErrIntzz = shooterErrIntzzz = shooterErrIntzzzz;
    }
    else 
    {
        shooterErrIntzzzz = shooterErrIntzzz;
        shooterErrIntzzz = shooterErrIntzz;
        shooterErrIntzz = shooterErrIntz;
        shooterErrIntz = shooterErrInt;
        shooterErrInt = shooterErrInt + shooterErr*LOOPTIME;
    }
    shooterErrInt = TalonXXI::Limit(SHOOTER_MIN_VELOCITY_ERROR_INT, SHOOTER_MAX_VELOCITY_ERROR_INT, shooterErrInt);
    if(errCount > 0)
    {
        shooterCmd = SHOOTER_K_BANDWIDTH/SHOOTER_K * (SHOOTER_TAU * SHOOTER_BANDWIDTH_MULTIPLIER * shooterErr + shooterErrInt);
    }
    else
    {
        shooterCmd = SHOOTER_K_BANDWIDTH/SHOOTER_K * (SHOOTER_TAU * shooterErr + shooterErrInt);
    
    }
   
    shooterCurPoszz = shooterCurPosz;
    shooterCurPosz = shooterCurPos;
    shooterCurVelzzz = shooterCurVelzz;
    shooterCurVelzz = shooterCurVelz;
    shooterCurVelz = shooterCurVel;
    shooterVelOutputzz = shooterVelOutputz;
    shooterVelOutputz = shooterVelOutput;
}

void BallShooter::ShroudControl()
{
    shroudGoalPos = TalonXXI::Limit(SHROUD_MIN_ANGLE, SHROUD_MAX_ANGLE, shroudGoalPos);
    /*shroudPCmd = shroudPCmd + TalonXXI::Limit(-1*SHROUD_MAX_DELTA_POS, SHROUD_MAX_DELTA_POS, shroudGoalPos - shroudPCmd);
    shroudPosError = shroudPCmd - shroudCurPos;
    //shroudCmd = shroudPosError * SHROUD_TEST_KP;
    //shroudCurVel = TalonXXI::Limit(SHROUD_MIN_VELOCITY, SHROUD_MAX_VELOCITY, shroudCurVel);
    shroudVelCmd = SHROUD_KP * shroudPosError;
    shroudVelErr = shroudVelCmd - shroudCurVel;
    shroudErrInt = shroudErrInt + (shroudVelErr*LOOPTIME);
    shroudErrInt = TalonXXI::Limit(SHROUD_MIN_VELOCITY_ERROR_INT, SHROUD_MAX_VELOCITY_ERROR_INT, shroudErrInt);
    shroudCmd = SHROUD_K_BANDWIDTH/SHROUD_K * (SHROUD_TAU * shroudVelErr + shroudErrInt);
    //shroudCmd = sqrt(fabs(shroudCmd))*TalonXXI::Sign(shroudCmd);
   // shroudCmd = shroudCmd + 0.1*TalonXXI::Sign(shroudCmd);*/
   shroudPosError = shroudGoalPos - shroudCurPos;
   shroudCmd = shroudPosError*SHROUD_TIC_BANDWIDTH/SHROUD_K;
   if(fabs(shroudCmd) > SHROUD_TIC_THRESHOLD)
   {
       ticSum = 0;
   }
   else
   {
       if(ticSum <= 0)
       {
           ticSum = SHROUD_TIC_COUNT;
       }
       else
       {
           ticSum--;
           //if(fabs(shroudCmd) > ticSum*SHROUD_TIC_THRESHOLD/(SHROUD_TIC_COUNT + 1))
           if(fabs(shroudCmd) > (ticSum+0.2)*SHROUD_TIC_THRESHOLD/(SHROUD_TIC_COUNT - 0.5))
           {
                shroudCmd = TalonXXI::Sign(shroudCmd)*SHROUD_TIC_THRESHOLD;
           }
           else
           {
               shroudCmd = 0.0;
           }
            
       }
   }
    shroudCmd = shroudCmd + SHROUD_CMD_OFFSET;
    //printf("%d %f \n", loopCount, shroudPosError);
}

bool BallShooter::IsShroudCalibrating()
{
    return isShroudCalibrating;
}

void BallShooter::SetTargeting()
{
    SetShooterVelTargeting();
    SetShroudAngleTargeting();
}

void BallShooter::StopTargeting()
{
    GiveShooterGoalVel(0.0);
}

void BallShooter::SetShooterVelTargeting()
{
    if(targetYPos > LIMELIGHT_Y_LIMIT_LOW && targetYPos < LIMELIGHT_Y_LIMIT_HIGH)
    {
        GiveShooterGoalVel(SHOOTER_ZONE_VEL);
    }
    /*if(targetYPos > LIMELIGHT_Y_INITIATION_LINE_LOW && targetYPos < LIMELIGHT_Y_INITIATIOn_LINE_HIGH)
    {
        GiveShooterGoalVel(SHOOTER_INITIATION_LINE_VEL);
    }
    if(targetYPos > LIMELIGHT_Y_FRONT_TRENCH_LOW && targetYPos < LIMELIGHT_Y_FRONT_TRENCH_HIGH)
    {
        GiveShooterGoalVel(SHOOTER_FRONT_TRENCH_VEL);
    }
    if(targetYPos > LIMELIGHT_Y_COLOR_WHEEL_LOW && targetYPos < LIMELIGHT_Y_COLOR_WHEEL_HIGH)
    {
        GiveShooterGoalVel(SHOOTER_COLOR_WHEEL_VEL);
    }
    if(targetYPos > LIMELIGHT_Y_OTHER_LINE_LOW && targetYPos < LIMELIGHT_Y_OTHER_LINE_HIGH)
    {
        GiveShooterGoalVel(SHOOTER_FAR_LINE_VEL);
    }*/
}

void BallShooter::SetShroudAngleTargeting()
{
    if(targetYPos > LIMELIGHT_Y_LIMIT_LOW && targetYPos < LIMELIGHT_Y_MID)
    {
        targetYPos = TalonXXI::Limit(LIMELIGHT_Y_LIMIT_LOW, LIMELIGHT_Y_MID, targetYPos);
        shroudGoalAngle = SHROUD_LOW_ANGLE + (SHROUD_LOW_ANGLE - SHROUD_MID_ANGLE) * (targetYPos - LIMELIGHT_Y_LOW) / (LIMELIGHT_Y_LOW - LIMELIGHT_Y_MID);
        GiveShroudGoalAngle(shroudGoalAngle);
    }
    if(targetYPos > LIMELIGHT_Y_MID && targetYPos < LIMELIGHT_Y_LIMIT_HIGH)
    {
        targetYPos = TalonXXI::Limit(LIMELIGHT_Y_MID, LIMELIGHT_Y_LIMIT_HIGH, targetYPos);
        shroudGoalAngle = SHROUD_MID_ANGLE + (SHROUD_MID_ANGLE - SHROUD_HIGH_ANGLE) * (targetYPos - LIMELIGHT_Y_MID) / (LIMELIGHT_Y_MID - LIMELIGHT_Y_HIGH);
        GiveShroudGoalAngle(shroudGoalAngle);
    }

    /*if(targetYPos > LIMELIGHT_Y_INITIATION_LINE_LOW && targetYPos < LIMELIGHT_Y_INITIATIOn_LINE_HIGH)
    {
        GiveShroudGoalAngle(SHROUD_INITIATION_LINE_ANGLE);
    }
    if(targetYPos > LIMELIGHT_Y_FRONT_TRENCH_LOW && targetYPos < LIMELIGHT_Y_FRONT_TRENCH_HIGH)
    {
        GiveShroudGoalAngle(SHROUD_FRONT_TRENCH_ANGLE);
    }
    if(targetYPos > LIMELIGHT_Y_COLOR_WHEEL_LOW && targetYPos < LIMELIGHT_Y_COLOR_WHEEL_HIGH)
    {
        GiveShroudGoalAngle(SHROUD_BEHIND_COLOR_WHEEL_ANGLE);
    }
    if(targetYPos > LIMELIGHT_Y_OTHER_LINE_LOW && targetYPos < LIMELIGHT_Y_OTHER_LINE_HIGH)
    {
        GiveShroudGoalAngle(SHROUD_OTHER_LINE_ANGLE);
    }*/
}

void BallShooter::ManualShooterAdjust(double shooterAxisReading)
{
    shooterCmd = shooterAxisReading;
    /*if(fabs(shooterAxisReading) > SHOOTER_MANUAL_DEADBAND)
    { 
        if(!isShooterManual)
        {
            shooterGoalVel = shooterCurVel;
            isShooterManual = true;
        }
        shooterGoalVel = shooterGoalVel + shooterAxisReading*SHOOTER_MANUAL_DELTA_VEL;
    }*/
}

void BallShooter::ManualShroudAdjust(double shroudAxisReading)
{
   // shroudCurPos = localSurveillance->GetAngle_Shrouding();
    if(fabs(shroudAxisReading) > SHROUD_MANUAL_DEADBAND)
    { 
        if(!isShroudManual)
        {
            shroudGoalPos = shroudCurPos;
            isShroudManual = true;
        }
        shroudGoalPos = shroudGoalPos + shroudAxisReading*SHROUD_MANUAL_DELTA;
    }
}

void BallShooter::GiveShooterGoalVel(double inGoalVel)
{
    shooterGoalVel = inGoalVel;
    //shooterVelCmd = inGoalVel;
   //shooterCmd = inGoalVel;
}

void BallShooter::GiveShroudGoalAngle(double inGoalAngle)
{
    shroudGoalPos = inGoalAngle;
    /*if(inGoalAngle>0.1)
    {
        shroudGoalPos = shroudGoalPos + 0.02;
    }
    else if(inGoalAngle<-0.1)
    {
        shroudGoalPos = shroudGoalPos - 0.02;
    }*/
    //shroudCmd = inGoalAngle;
    isShroudManual = false;
}

void BallShooter::Recalibrate()
{
   if(!isShroudCalibrating)
   {
        isShroudCalibrating = true;
        shroudCalState = 0;
   }
}

bool BallShooter::ReadyToShoot()
{
    if(IsShooterAtGoal() && IsShroudAtGoal())
    {
        return true;
    }
    return false;
}

void BallShooter::ShooterStop()
{
    ShooterReset();
}

void BallShooter::ShroudStop()
{
    ShroudReset();
}
//SHOOTER BASIC FUNCTIONS
bool BallShooter::IsShooterAtGoal()
{
    bool isAtGoal = ((fabs(shooterCurVel - shooterGoalVel) < SHOOTER_AT_GOAL_ALLOWANCE) && shooterGoalVel != 0.0);
    return isAtGoal;
}

double BallShooter::GetShooterGoalVel()
{
    return shooterGoalVel;
}

double BallShooter::GetShooterVel()
{
    return shooterCurVel;
}

//SHROUD BASIC FUNCTIONS
bool BallShooter::IsShroudAtGoal()
{
    return (fabs((shroudCurPos - shroudGoalPos) < SHROUD_AT_GOAL_ALLOWANCE));
}

double BallShooter::GetShroudGoal()
{
    return shroudGoalPos;
}

double BallShooter::GetShroudPos()
{
    return shroudCurPos;
}

void BallShooter::ShroudCalibrate()
{
    switch(shroudCalState)
    {
        case 0:
            shroudCmd = SHROUD_CALIBRATE_COMMAND;
            if(shroudCurPos != shroudCurPosz)
            {
                shroudCalState++;
            }
            break;

        case 1:
            if(shroudCurPos == shroudCurPosz)
            {
                shroudCmd = 0.0;
                mainRobot->surveillance->ResetShroudingEncoder();
                shroudCalState++;
            }
            break;

        case 2:
            ShroudReset();
            isShroudCalibrating = false;
            shroudCalState = 0;
            break;
    }
}

#ifdef TEST_MODE
void BallShooter::TestingShooter(double input)
{
    if(fabs(input) > 0.1)
    {
        shooterCmd = input*0.3;
    }
    else
    {
        shooterCmd = 0.0;
    }
    shooterCmd = TalonXXI::Limit(-0.3, 0.3, shooterCmd);
}

void BallShooter::TestingShroud(double input)
{
    if(fabs(input) > 0.1)
    {
        shroudCmd = input * 0.3;
    }
    else
    {
        shroudCmd = 0.0;
    }

    shroudCmd = TalonXXI::Limit(-0.3, 0.3, shroudCmd);
}
#endif
