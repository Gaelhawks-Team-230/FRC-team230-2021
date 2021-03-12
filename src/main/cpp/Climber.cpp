/*
#include "Common.h"
#include "Climber.h"

Climber::Climber(TalonXXI* pRobot)
{
    mainRobot = pRobot;
    climbExtendMotor = new frc::VictorSP(PWM_CLIMB_EXTEND);
    climbWinchMotor = new frc::VictorSP(PWM_CLIMB_WINCH);
    localSurveillance = mainRobot->surveillance;
    
    LocalReset();
}

//Sets all local variables
void Climber::LocalReset()
{
    extendPos = localSurveillance->GetClimbExtendDis();
    goalPos = extendPos;
    winchPos = localSurveillance->GetClimbWinchDis();
    extendMotorCmd = 0.0;
    winchMotorCmd = 0.0;
    isClimbing = false;
    isManualMove = false;
    loopCount = 0;
}

//Holds how the variables should be set when the robot starts
void Climber::StartingConfig()
{

}

//Deploys solenoid piston
void Climber::DeployHook()
{
}

//Turns climb motor off
void Climber::StopClimb()
{
   // motorCmd= 0.0;
    //mode= IDLE; ???
}

//Checks if robot is climbing
bool Climber::IsClimbing()
{
    if(extendPos > IS_CLIMBING_THRESHOLD)
    {
        isClimbing = true;
    }
    return isClimbing;
}

void Climber::StopAll()
{
    LocalReset();
}

void Climber::UpdateDash()
{
    //frc::SmartDashboard::PutNumber("Climb Extend Goalpos: ", goalPos);
    //frc::SmartDashboard::PutNumber("Climb Extend Cmd: ", extendMotorCmd);
    //frc::SmartDashboard::PutNumber("Climb Winch Cmd: ", winchMotorCmd);
}

void Climber::Service()
{
    loopCount++;
    extendPos = localSurveillance->GetClimbExtendDis();
    winchPos = localSurveillance->GetClimbWinchDis();
#ifndef TEST_MODE
   IsClimbing();
   //extendErr = goalPos - extendPos;
 //  extendMotorCmd = extendErr * CLIMBER_EXTEND_K;
   extendMotorCmd = TalonXXI::Limit(-CLIMBER_MAX_CMD, CLIMBER_MAX_CMD, extendMotorCmd);
#endif
    goalPos = TalonXXI::Limit(MIN_CLIMB_EXTEND_DIS, MAX_CLIMB_EXTEND_DIS, goalPos);
  //  printf("%d %f %f %f %f %f \n", loopCount, extendMotorCmd, extendPos, goalPos, winchMotorCmd, winchPos);
    climbExtendMotor->Set(extendMotorCmd);
    climbWinchMotor->Set(winchMotorCmd);
}

void Climber::MoveExtender(double input)
{
    /*if(input != 0.0 && !isManualMove)
    {
        goalPos = extendPos;
        isManualMove = true;
    }
    if(input != 0.0)
    {
        //goalPos = goalPos + TalonXXI::Sign(input)*CLIMBER_MANUAL_DELTA;
        extendMotorCmd = TalonXXI::Sign(input)*CLIMBER_MAX_CMD;
    }
    else
    {
        extendMotorCmd = 0.0;
    }
}

void Climber::MoveWinch(double input)
{
    winchMotorCmd = input;
}

void Climber::GiveGoalPos(double inGoalPos)
{
    goalPos = inGoalPos;
    isManualMove = false;
}

#ifdef TEST_MODE
void Climber::ExtendTesting(double input)
{
    if(fabs(input) > 0.1)
    {
        extendMotorCmd = input;
    }
    else
    {
        extendMotorCmd = 0.0;
    }
    extendMotorCmd = TalonXXI::Limit(-0.6, 0.6, extendMotorCmd);
}

void Climber::WinchTesting(double input)
{
    if(fabs(input) > 0.1)
    {
        winchMotorCmd = input;
    }
    else
    {
        winchMotorCmd = 0.0;
    }
    winchMotorCmd = TalonXXI::Limit(-0.6, 0.6, winchMotorCmd);
}
#endif
*/