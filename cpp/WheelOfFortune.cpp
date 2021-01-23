//#define NEEDS_SENSOR

#include "TalonXXI_main.h"
#include "WheelOfFortune.h"

//#include <frc/VictorSP.h>


WheelOfFortune::WheelOfFortune(TalonXXI* pRobot) //constructor 
{
    mainRobot = pRobot;
    localSurveillance = pRobot->surveillance;
    //create motor object
    spinnyMotor = new frc::VictorSP(PWM_COLOR_WHEEL); 

    LocalReset();
}

//Sets all local variables
void WheelOfFortune::LocalReset()
{
    //goalColor = 0; //set color to color not on wheel to start? or void color You will be setting the real color you get from Diya's class after
    operationMode = IDLE_MODE;
    motorCommand = 0.0;
    colorCount = 0;
    curColor = ' ';
    lastColor = ' ';
    goalColor = ' ';
}

//Holds how the variables should be set when the robot starts
void WheelOfFortune::StartingConfig()
{
  //probably nothing here for now
}


void WheelOfFortune::StopAll()
{
    motorCommand = 0.0;
    operationMode = IDLE_MODE;
}

bool WheelOfFortune::LookForColor(char searchColor, int numSearches)
{
    bool complete = false;
    lastColor = curColor;
    curColor = localSurveillance->GetColorChar();

    motorCommand = 0.5;
    if(curColor != 'X')
    {
        if (curColor != lastColor)  
            return complete;
        if(curColor == searchColor)
        {
            colorCount++;
        }
        if(colorCount >= numSearches)
        {
            motorCommand = 0.0;
            colorCount = 0;
            complete = true;
        }
    }
    return complete;
}

void WheelOfFortune::GetMatchColor()
{
    std::string gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
    if(gameData.length() > 0)
    {
        switch(gameData[0])
        {
            case 'B': //If sensor wants BLUE we should aim for RED
                goalColor = 'R';
                break;
            case 'R': //If sensor wants RED we should aim for BLUE
                goalColor = 'B';
                break;
            case 'G': //If sensor wants GREEN we should aim for YELLOW
                goalColor = 'Y';
                break;
            case 'Y': //If sensor wants YELLOW we should aim for GREEN
                goalColor = 'G';
                break;
            default:
                printf("Getting the color from the field has gone wrong!");
                break;
        }
    }

}



//this function sets the goal distance in degrees
//stage 2
void WheelOfFortune::SetAngleDistanceConfigSpin(void)
{
    //double rotationDegreesSpinnyWheel = (spinnyToColorRatio * AMOUNT_OF_STAGE2_SPINS) * 360; //360 = degrees
    operationMode = SPIN_MODE;
    //goalAngle = rotationDegreesSpinnyWheel;
    //start angle is going to be current angle 
}
//stage 3
void WheelOfFortune::SetAngleDistanceConfigMatch(void)
{
    operationMode = MATCH_MODE;
}


//Anything that you want printed on the dashboard for testing or during match. 
//Any information which may be important. 

void WheelOfFortune::UpdateDash() //not sure what is here
{
    frc::SmartDashboard::PutNumber("What Mode?:", operationMode); 
    frc::SmartDashboard::PutNumber("Goal Color:", goalColor);
// frc::SmartDashboard::PutNumber("Current Color:", ShownVariable);
}
    
//Called every loop (used for timing related stuff)
void WheelOfFortune::Service()
{
//switch statement for operation modes
#ifndef TEST_MODE
    switch (operationMode)
    {
        case IDLE_MODE:
            motorCommand = 0.0; 
            break;
        case SPIN_MODE:
            if(LookForColor(COLOR_FOR_SPIN, 7))
            {
                operationMode = IDLE_MODE;
            }
            break;
        case MATCH_MODE:
            GetMatchColor();
            if(goalColor != ' ')
            {
                if(LookForColor(goalColor, 1))
                {
                    operationMode = IDLE_MODE;
                }
            }
        
            /*if ((currentColor) == goalColor)
            {
                motorCommand = 0.0;
                operationMode = IDLE_MODE;
            }
            else 
            {
               motorCommand = SPIN_MOTOR_COMMAND;
            }
            break;
            */
    } 
#endif
    spinnyMotor->Set(motorCommand);
    
}

#ifdef TEST_MODE
void WheelOfFortune::Testing(double input)
{
    if(fabs(input) > 0.1)
    {
        motorCommand = input;
    }
    else
    {
        motorCommand = 0.0;
    }
    motorCommand = TalonXXI::Limit(-0.3, 0.3, motorCommand);
}
#endif
