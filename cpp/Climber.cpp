/*
#include "Common.h"
#include "Climber.h"

Climber::Climber(TalonXXI* pRobot)
{
    mainRobot = pRobot;
    //climbPiston = new frc::Solenoid(PCM_CLIMB_PISTON);
    climbExtendMotor = new frc::VictorSP(PWM_CLIMB_EXTEND);
    climbWinchMotor = new frc::VictorSP(PWM_CLIMB_WINCH);
    localSurveillance = mainRobot->surveillance;
    
    LocalReset();
}

//Sets all local variables
void Climber::LocalReset()
{
    winchDistance= 0.0;
    extendMotorCmd = 0.0;
    winchMotorCmd = 0.0;
    pistonCmd= CLIMB_PISTON_OFF;
    mode= IDLE;
}

//Holds how the variables should be set when the robot starts
void Climber::StartingConfig()
{
    // bring down piston?
    //motorCmd= WINCH_RETRACT; or maybe Climb(); //wind up winch
}

//Deploys solenoid piston
void Climber::DeployHook()
{
    pistonCmd= CLIMB_PISTON_EXTEND;
    mode= HOOK_DEPLOYED;
}

//Turns climb motor on
void Climber::SetClimb()
{
    //motorCmd= WINCH_RETRACT;
    mode= IS_CLIMBING;
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
    if(mode==IS_CLIMBING)
    {
        return true;
    }
    else
    {
        return false;
    }
    
}

//Checks if the robot is balanced on the hanger
bool Climber::IsBalanced()
{
    if (mode==IS_BALANCED)
    {
        return true;
    }
    else
    {
        return false;
    }
}

//Checks if the climb is done
bool Climber::IsClimbComplete()
{
    if (mode==CLIMB_COMPLETE)
    {
        return true;
    }
    else
    {
        return false;
    }
}

//Reads the encoder value
double Climber::GetWinchDistance()
{
    winchDistance = localSurveillance->GetClimbDist();
    return (winchDistance);
}

void Climber::StopAll()
{
    LocalReset();
  //  climbMotor->Set(motorCmd);

}

//Anything that you want printed on the dashboard for testing or during match. 
//Any information which may be important. 
void Climber::UpdateDash()
{
    frc::SmartDashboard::PutNumber("Mode", mode);
    frc::SmartDashboard::PutNumber("Winch Distance", winchDistance);
    //frc::SmartDashboard::PutNumber("Motor Cmd", motorCmd);
    frc::SmartDashboard::PutNumber("Piston Cmd", pistonCmd);
}

//Called every loop (used for timing related stuff)
void Climber::Service()
{
    winchDistance = localSurveillance->GetClimbDist();
#ifndef TEST_MODE
   /* switch (mode)
    {
        case IDLE:
            break;
    
        case HOOK_DEPLOYED:
            pistonCmd= CLIMB_PISTON_EXTEND;
            break;

        case IS_CLIMBING:
            // call control system to calculate motor command

            // check for climb done
            if (winchDistance>= MAX_WINCH_DISTANCE)
            {
                motorCmd= 0.0;
                mode= CLIMB_COMPLETE;
            }

            break;

        case CLIMB_COMPLETE:
            // test if it is balanced... and do stuff to make it balance
            break;

        case IS_BALANCED:
            // done... time to hang out
            //maintain balance until the game is over
            break;

        
    }*/
/*
#endif
   // printf("%f %f \n", extendMotorCmd, winchMotorCmd);
    climbExtendMotor->Set(extendMotorCmd);
    climbWinchMotor->Set(winchMotorCmd);
#ifdef USING_SOLENOID
    //climbPiston->Set(pistonCmd);
#endif
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
    extendMotorCmd = TalonXXI::Limit(-0.3, 0.3, extendMotorCmd);
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
    winchMotorCmd = TalonXXI::Limit(-0.3, 0.3, winchMotorCmd);
}
#endif
*/