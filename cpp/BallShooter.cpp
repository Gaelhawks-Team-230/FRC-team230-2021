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
    shroudDither = 0.2;
    
    LocalReset();
    isShooterCalibrating = true;
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
    shooterCurVelOwn = shooterCurPos;
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
    isShooterManual = false;
    isShooterOverride = false;
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
    isShroudAtGoal = true;
    isShroudManual = false;
    isShroudOverride = false;
    shroudMotor->Set(shroudCmd);
}

void BallShooter::StartingConfig()
{

}

void BallShooter::StopAll()
{
    LocalReset();
}

void BallShooter::UpdateDash()
{
    //Shooter
    frc::SmartDashboard::PutBoolean("isShooterCalibrating", isShooterCalibrating);
    frc::SmartDashboard::PutNumber("shooterCmd", shooterCmd);
    frc::SmartDashboard::PutNumber("shooterCurVel", shooterCurVel);
    frc::SmartDashboard::PutNumber("shootergoalVel", shooterGoalVel);
    frc::SmartDashboard::PutBoolean("isShooterManual", isShooterManual);
    frc::SmartDashboard::PutBoolean("isShooterOverride", isShooterOverride);
    double shooterCurrentSupply = shooterMotor->GetSupplyCurrent();
    double shooterCurrentOutput = shooterMotor->GetStatorCurrent();
    frc::SmartDashboard::PutNumber("Shooter Supply current", shooterCurrentSupply);
    frc::SmartDashboard::PutNumber("Shooter Output Current", shooterCurrentOutput);
    //Shroud
    frc::SmartDashboard::PutBoolean("isShroudCalibrating", isShroudCalibrating);
    frc::SmartDashboard::PutNumber("shroudCmd", shroudCmd);
    frc::SmartDashboard::PutNumber("shroudCurPos", shroudCurPos);
    frc::SmartDashboard::PutNumber("shroudGoalPos", shroudGoalPos);
    frc::SmartDashboard::PutBoolean("isShroudManual", isShroudManual);
    frc::SmartDashboard::PutBoolean("isShroudOverride", isShroudOverride);   
}

void BallShooter::Service()
{
    loopCount++;
    seesTarget = mainRobot->limelight->SeesShooterTarget();
    targetYPos = mainRobot->limelight->GetTargetVOffset();
 
    shooterCurPos = localSurveillance->GetPosRadians_Shooter();
    shooterCurVel = localSurveillance->GetPosRadPerSec_Shooter();
    shooterVelOutput = SHOOTER_LEAD_FILTER_C*shooterVelOutputz + SHOOTER_LEAD_FILTER_A*shooterCurVel + SHOOTER_LEAD_FILTER_B*shooterCurVelz;
    

  //  double distance = (limelighty - 31.459)/-1.0298;

   // printf("%d %f %f %f %f \n", loopCount, limelightsees, limelightx, limelighty, distance);
    shroudCurPos = localSurveillance->GetAngleRadians_Shrouding();
    shroudCurVel = (shroudCurPos - shroudCurPosz)/LOOPTIME;
    
    //printf("%d %f %f %f %f \n", loopCount, shroudCmd, shroudGoalPos ,shroudPCmd, shroudCurPos);
   
#ifndef TEST_MODE
   // if(!isShooterCalibrating)
   // {
        ShooterControl();
  //  }
    if(isShroudCalibrating)
    {
        ShroudCalibrate();
    }
    else
    {
        ShroudControl();
    }
#endif
    //printf("%d %d %f %f %f \n", loopCount, shroudCalState, shroudCmd, shroudCurPos, shroudCurPosz);
   // printf("%d %f %f \n", loopCount, shooterCmd, shooterCurVel);
    //printf("%d %f %f %f \n", loopCount, shroudCmd, shroudCurPos, shroudPosError*2048.0/(2.0*PI));
    //shooterCmd = TalonXXI::Limit(SHOOTER_MIN_CMD, SHOOTER_MAX_CMD, shooterCmd);
    shroudCmd = TalonXXI::Limit(SHROUD_MIN_CMD, SHROUD_MAX_CMD, shroudCmd);
    //double shroudPos = localSurveillance->GetAngleRadians_Shrouding();
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
   shroudCmd = shroudPosError * 20.0/SHROUD_K;
    static double threshold = 0.15; //0.15
    static int ticsum = 0;
    static int ticcount = 5; //5
   if(fabs(shroudCmd) > threshold)
   {
       ticsum = 0;
   }
   else
   {
       if(ticsum <= 0)
       {
           ticsum = ticcount;
       }
       else
       {
           if(fabs(shroudCmd) > ticsum*threshold/(ticcount + 1))
           {
                shroudCmd = TalonXXI::Sign(shroudCmd)*threshold;
           }
           else
           {
               shroudCmd = 0.0;
           }
            ticsum--;
       }
   }
    shroudCmd = shroudCmd + 0.05;
    //printf("%d %f \n", loopCount, shroudPosError);
}

bool BallShooter::IsShooterCalibrating()
{
    return isShooterCalibrating;
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

void BallShooter::SetShooterVelTargeting()
{
    if(fabs(targetYPos - SHROUD_LIMELIGHT_Y_INITIATION_LINE) < SHROUD_LIMELIGHT_Y_RANGE_CLOSE)
    {
        GiveShooterGoalVel(SHOOTER_CLOSE_VEL);
    }
    else if(fabs(targetYPos - SHROUD_LIMELIGHT_Y_MID) < SHROUD_LIMELIGHT_Y_RANGE_MID)
    {
        GiveShooterGoalVel(SHOOTER_MID_VEL);
    }
    else
    {
        GiveShooterGoalVel(SHOOTER_FAR_VEL);
    }
}

void BallShooter::SetShroudAngleTargeting()
{
    //Function that determines shroud angle based on target reading
    /*if(targetYPos > SHROUD_LIMELIGHT_Y_BEHIND_WHEEL && targetYPos < SHROUD_LIMELIGHT_Y_FRONT_WHEEL)
    {
        GiveShroudGoalAngle(SHROUD_BEHIND_COLOR_WHEEL_ANGLE);
    }*/
    /*double slope = (SHROUD_BEHIND_COLOR_WHEEL_ANGLE - SHROUD_INITIATION_LINE_ANGLE)/(SHROUD_LIMELIGHT_Y_FAR - SHROUD_LIMELIGHT_Y_INITIATION_LINE - 2*SHROUD_LIMELIGHT_Y_RANGE);
    double shroudAngle;
    if(seesTarget)
    {
        shroudAngle = slope*targetYPos - slope*(SHROUD_LIMELIGHT_Y_INITIATION_LINE - SHROUD_LIMELIGHT_Y_RANGE) + SHROUD_LIMELIGHT_Y_RANGE;
    }
    else
    {
        shroudAngle = SHROUD_BEHIND_COLOR_WHEEL_ANGLE;
    }*/
    if(fabs(targetYPos - SHROUD_LIMELIGHT_Y_INITIATION_LINE) < SHROUD_LIMELIGHT_Y_RANGE_CLOSE)
    {
        GiveShroudGoalAngle(SHROUD_INITIATION_LINE_ANGLE);
    }
    else if(fabs(targetYPos - SHROUD_LIMELIGHT_Y_MID) < SHROUD_LIMELIGHT_Y_RANGE_MID)
    {
        GiveShroudGoalAngle(SHROUD_FRONT_TRENCH_ANGLE);
    }
    else
    {
        GiveShroudGoalAngle(SHROUD_BEHIND_COLOR_WHEEL_ANGLE);
    }
    /*if(fabs(targetYPos - SHROUD_LIMELIGHT_Y_FAR) < SHROUD_LIMELIGHT_Y_RANGE_FAR)
    {
        GiveShroudGoalAngle(SHROUD_BEHIND_COLOR_WHEEL_ANGLE);
    }*/
     

   // printf("%f %f %f \n", targetYPos, shroudGoalPos, shroudAngle);
}

void BallShooter::ManualShooterAdjust(double shooterAxisReading)
{
    shooterCmd = shooterAxisReading;
    /*if(fabs(shooterAxisReading) > SHOOTER_MANUAL_DEADBAND)
    { 
        if(isShooterOverride)
        {
            shooterCmd = shooterCmd + shooterAxisReading*SHOOTER_OVERRIDE_CONSTANT;
        }
        else
        {
            if(!isShooterManual)
            {
                shooterGoalVel = shooterCurVel;
                isShooterManual = true;
            }
            shooterGoalVel = shooterGoalVel + shooterAxisReading*SHOOTER_MANUAL_DELTA_VEL;
        }
    }*/
}

void BallShooter::ManualShroudAdjust(double shroudAxisReading)
{
    shroudCurPos = localSurveillance->GetAngle_Shrouding();

    if(fabs(shroudAxisReading) > SHROUD_MANUAL_DEADBAND)
    { 
        if(isShroudOverride)
        {
            shroudCmd = shroudAxisReading * SHROUD_OVERRIDE_CONSTANT;
        }
        else
        {
            if(!isShroudManual)
            {
                shroudGoalPos = shroudCurPos;
                isShroudManual = true;
            }
            shroudGoalPos = shroudGoalPos + shroudAxisReading*SHROUD_MANUAL_DELTA;
        }
    }
}

void BallShooter::GiveShooterGoalVel(double inGoalVel)
{
    shooterGoalVel = inGoalVel;
    //shooterVelCmd = inGoalVel;
   //shooterCmd = inGoalVel;
    isShooterManual = false;
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
   // isShooterCalibrating = true;
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
void BallShooter::TurnOnShooterOverride()
{
    isShooterOverride = true;
    shooterCmd = SHOOTER_OVERRIDE_CMD;
}

void BallShooter::TurnOffShooterOverride()
{
    isShooterOverride = false;
    ShooterReset();
}

bool BallShooter::IsShooterAtGoal()
{
    return (fabs((shooterCurVel - shooterGoalVel) < SHOOTER_AT_GOAL_ALLOWANCE));
}

bool BallShooter::IsShooterOverride()
{
    return isShooterOverride;
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
void BallShooter::TurnOnShroudOverride()
{
    isShroudOverride = true;
}

void BallShooter::TurnOffShroudOverride()
{
    isShroudOverride = false;
    ShroudReset();
}

bool BallShooter::IsShroudAtGoal()
{
    return (fabs((shroudCurPos - shroudGoalPos) < SHROUD_AT_GOAL_ALLOWANCE));
}

bool BallShooter::IsShroudOverride()
{
    return isShroudOverride;
}

double BallShooter::GetShroudGoal()
{
    return shroudGoalPos;
}

double BallShooter::GetShroudPos()
{
    return shroudCurPos;
}

bool BallShooter::IsShroudAtMax()
{
    return (shroudCurPos > (SHROUD_MAX_ANGLE - SHROUD_AT_GOAL_ALLOWANCE));
}

bool BallShooter::IsShroudAtMin()
{
    return (shroudCurPos < (SHROUD_MIN_ANGLE + SHROUD_AT_GOAL_ALLOWANCE));
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
//#endif
