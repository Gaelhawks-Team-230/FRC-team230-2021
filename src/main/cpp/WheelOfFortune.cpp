
/*//#define NEEDS_SENSOR

#include "TalonXXI_main.h"
#include "WheelOfFortune.h"

//#include <frc/VictorSP.h>


WheelOfFortune::WheelOfFortune(TalonXXI* pRobot) //constructor 
{
    mainRobot = pRobot;
    localSurveillance = pRobot->surveillance;
    //create motor object
    spinnyMotor = new frc::VictorSP(PWM_COLOR_WHEEL); 
    mainRobot->userInput->SetGamepadAxisParameter(COLOR_WHEEL_AXIS, 0.8, 1.0);
    LocalReset();
}

void WheelOfFortune::LocalReset()
{
    //goalColor = 0; //set color to color not on wheel to start? or void color You will be setting the real color you get from Diya's class after
    operationMode = WHEEL_IDLE_MODE;
    motorCommand = 0.0;
    colorCount = 0;
    curColor = ' ';
    lastColor = ' ';
    goalColor = ' ';
    isCompleteSpin = false;
    isManualSpin = false;
}

void WheelOfFortune::StartingConfig()
{
  //probably nothing here for now
}


void WheelOfFortune::StopAll()
{
    motorCommand = 0.0;
    operationMode = WHEEL_IDLE_MODE;
}

bool WheelOfFortune::LookForColor(char searchColor, int numSearches)
{
    bool complete = false;
    mainRobot->addedDrive = COLOR_WHEEL_ADDED_DRIVE;
    motorCommand = SPIN_ROTATE_CMD;
    if(curColor != 'X')
    {
        if (curColor == lastColor)  
            return complete;
        if(curColor == searchColor)
        {
            colorCount++;
        }
        if(colorCount >= numSearches)
        {
            motorCommand = 0.0;
           // colorCount = 0;
            complete = true;
        }
    }
    if(complete)
    {
        mainRobot->addedDrive = 0.0;
        isCompleteSpin = true;
    }
    else
    {
        isCompleteSpin = false;
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

//stage 2
void WheelOfFortune::StartWheelRotate()
{
    if(operationMode != SPIN_MODE)
    {
        colorCount = 0;
        operationMode = SPIN_MODE;
    }
}

//stage 3
void WheelOfFortune::StartColorMatch()
{
    operationMode = MATCH_MODE;
}

void WheelOfFortune::SetIdleMode()
{
    if(operationMode != WHEEL_IDLE_MODE)
    {
        operationMode = WHEEL_IDLE_MODE;
    }
}

void WheelOfFortune::UpdateDash() //not sure what is here
{
    //frc::SmartDashboard::PutNumber("What Mode?:", operationMode); 
    //frc::SmartDashboard::PutChar("Goal Color:", goalColor);
    frc::SmartDashboard::PutNumber("Color count: ", colorCount);
    frc::SmartDashboard::PutBoolean("is color complete ", isCompleteSpin);
// frc::SmartDashboard::PutNumber("Current Color:", ShownVariable);
}
    
//Called every loop (used for timing related stuff)
void WheelOfFortune::Service()
{
    lastColor = curColor;
    curColor = localSurveillance->GetColorChar();
//switch statement for operation modes
#ifndef TEST_MODE
    switch (operationMode)
    {
        case WHEEL_IDLE_MODE:
            if(!isManualSpin)
            {
                motorCommand = 0.0; 
            }
           // colorCount = 0;
            mainRobot->addedDrive = 0.0;
            break;

        case SPIN_MODE:
            isManualSpin = false;
            if(LookForColor(COLOR_FOR_SPIN, 7))
            {
                operationMode = WHEEL_IDLE_MODE;
            }
            break;

        case MATCH_MODE:
            isManualSpin = false;
            GetMatchColor();
            if(goalColor != ' ')
            {
                if(LookForColor(goalColor, 1))
                {
                    operationMode = WHEEL_IDLE_MODE;
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
            
    } 
#endif 

    spinnyMotor->Set(motorCommand);
}

void WheelOfFortune::ManualWheelMove(double input)
{
    if(operationMode == WHEEL_IDLE_MODE)
    {
        if(fabs(input) > MANUAL_WHEEL_DEADBAND)
        {
            isManualSpin = true;
            motorCommand = TalonXXI::Sign(input)*SPIN_ROTATE_CMD;
        }
        else
        {
            motorCommand = 0.0;
            isManualSpin = false;
        }
    }
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
    motorCommand = TalonXXI::Limit(-0.8, 0.8, motorCommand);
}
#endif
*/
