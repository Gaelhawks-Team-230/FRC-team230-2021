/*
* JoystickState.cpp
*
* Created on: Jan 11, 2020
*		Author: Ben DeMartino
*/
#include "JoystickState.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include "stdio.h"
/*This program is made for all the different buttons and axies on the controllers*/
JoystickState::JoystickState(TalonXXI* pRobot)
{
    mainRobot = pRobot;
    flightcontroller = new frc::Joystick(0);
	gamepad = new frc::Joystick(1);

    gamepadBtnCount = 0;
    flightcontrollerBtnCount = 0;
    gamepadAxisCount = 0;
    flightcontrollerAxisCount = 0;

    JoystickCountInitialize();
    
    LocalReset();
}

void JoystickState::JoystickCountInitialize()
{
    gamepadBtnCount = gamepad->GetButtonCount();
    if (gamepadBtnCount > MAX_JOYSTICK_BUTTONS)
    {
        printf("Gamepad has too many buttons! (read %d, clipped to %d)\n", gamepadBtnCount, MAX_JOYSTICK_BUTTONS);
        gamepadBtnCount = MAX_JOYSTICK_BUTTONS;
    }
    flightcontrollerBtnCount = flightcontroller->GetButtonCount();
    if (flightcontrollerBtnCount > MAX_JOYSTICK_BUTTONS)
    {
        printf("Flight Controller has too many buttons!(read %d, clipped to %d)\n", flightcontrollerBtnCount, MAX_JOYSTICK_BUTTONS);
        flightcontrollerBtnCount = MAX_JOYSTICK_BUTTONS;
    }
    gamepadAxisCount = gamepad->GetAxisCount();
    if (gamepadAxisCount > MAX_JOYSTICK_AXES)
    {
        printf("Gamepad has too many axes! (read %d, clipped to %d)\n", gamepadAxisCount, MAX_JOYSTICK_AXES);
        gamepadAxisCount = MAX_JOYSTICK_AXES;
    }
    flightcontrollerAxisCount = flightcontroller->GetAxisCount();
    if (flightcontrollerAxisCount > MAX_JOYSTICK_AXES)
    {
        printf("Flight Controller has too many axes! (read %d, clipped to %d)\n", flightcontrollerAxisCount, MAX_JOYSTICK_AXES);
        flightcontrollerAxisCount = MAX_JOYSTICK_AXES;
    }
    int axis;
    for(axis = 0; axis < gamepadAxisCount; axis++)
    {
        gamepadDeadband[axis] = 0.0;
        gamepadShaping[axis] = 0.0;
    }
    for(axis = 0; axis < flightcontrollerAxisCount; axis++)
    {
        flightcontrollerDeadband[axis] = 0.0;
        flightcontrollerShaping[axis] = 0.0;
    }

    gamepadPOVCount = gamepad->GetPOVCount();
    if (gamepadPOVCount < 1)
    {
        printf("Gamepad does not have a POV!\n");
    }
    printf("GamepadBtnCount %d, FlightBtnCount %d, GamepadAxisCount %d, FlightAxisCount %d, gamepadPOVCount %d \n", 
           gamepadBtnCount, flightcontrollerBtnCount, gamepadAxisCount, flightcontrollerAxisCount, gamepadPOVCount);
}

void JoystickState::LocalReset()
{
    int btn;
    for(btn = 0; btn < MAX_JOYSTICK_BUTTONS; btn++)
    {
        gamepadBtnState[btn] = kOff;
    }
    for(btn = 0; btn < MAX_JOYSTICK_BUTTONS; btn++)
    {
        flightcontrollerBtnState[btn] = kOff;
    }
    int axis;
    for(axis = 0; axis < MAX_JOYSTICK_AXES; axis++)
    {
        gamepadAxisCmd[axis] = 0.0;
    }
    for(axis = 0; axis < MAX_JOYSTICK_AXES; axis++)
    {
        flightcontrollerAxisCmd[axis] = 0.0;
    }
    DpadUp = kOff;
    DpadRight = kOff;
    DpadDown = kOff;
    DpadLeft = kOff;
}

void JoystickState::SetGamepadAxisParameter(int axisNum, float deadband, float shaping)
{
    if ((axisNum < 0) || (axisNum >= gamepadAxisCount))
    {
        printf("Invalid gamepad axis number %d!\n", axisNum);
    }
    else
    {
        gamepadDeadband[axisNum] = deadband;
        gamepadShaping[axisNum] = shaping;
    }
}

void JoystickState::SetFlightControllerAxisParameter(int axisNum, float deadband, float shaping)
{
    if ((axisNum < 0) || (axisNum >= flightcontrollerAxisCount))
    {
        printf("Invalid flightController axis number %d!\n", axisNum);
    }
    else
    {
        flightcontrollerDeadband[axisNum] = deadband;
        flightcontrollerShaping[axisNum] = shaping;
    }
}

//Holds how the variables should be set when the robot starts
void JoystickState::StartingConfig()
{

}


void JoystickState::StopAll()
{
    LocalReset();
}

//Anything that you want printed on the dashboard for testing or during match. 
//Any information which may be important. 
void JoystickState::UpdateDash()
{
    //frc::SmartDashboard::PutBoolean("GamepadButtonPUSHED", GamepadBtnPushed(1));
    //frc::SmartDashboard::PutNumber("gamepadAxis0", GetGamepadAxis(0));
    //frc::SmartDashboard::PutBoolean("POV right", GetDpadRightPushed());
}

ButtonState JoystickState::GetGamepadButton(int btnNumber)
{
    if ((btnNumber <= 0) || (btnNumber > gamepadBtnCount))
    {
        printf("Invalid gamepad button number %d!\n", btnNumber);
        return (kOff);
    }
    else
    {
        return (gamepadBtnState[btnNumber-1]);
    }
}

ButtonState JoystickState::GetFlightControllerButton(int btnNumber)
{
    if ((btnNumber <= 0) || (btnNumber > flightcontrollerBtnCount))
    {
        printf("Invalid flightcontroller button number %d!\n", btnNumber);
        return (kOff);
    }
    else
    {
        return (flightcontrollerBtnState[btnNumber-1]);
    }
}

bool JoystickState::GamepadBtnPushed(int btnNumber)
{
    if ((btnNumber <= 0) || (btnNumber > gamepadBtnCount))
    {
        printf("Invalid gamepad button number %d!\n", btnNumber);
        return (false);
    }

    if ((gamepadBtnState[btnNumber-1] == kPressing) || (gamepadBtnState[btnNumber-1] == kHeld))
    {
        //printf("BUTTON 1 PUSHED\n");
        return true;
    }
    else
    {
        //printf("BUTTON 1 OFF\n");
        return false;
    }
}

bool JoystickState::FlightCtrlBtnPushed(int btnNumber)
{
    if ((btnNumber <= 0) || (btnNumber > flightcontrollerBtnCount))
    {
        printf("Invalid gamepad button number %d!\n", btnNumber);
        return (false);
    }
    if (flightcontrollerBtnState[btnNumber-1] == kPressing || flightcontrollerBtnState[btnNumber-1] == kHeld)
        return true;
    else
        return false;
}


float JoystickState::GetGamepadAxis(int axisNumber)
{
    if ((axisNumber < 0) || (axisNumber >= gamepadAxisCount))
    {
        printf("Invalid gamepad axis number %d!\n", axisNumber);
        return 0;
    }
    else
    {
        return (gamepadAxisCmd[axisNumber]);
    }
    
}
float JoystickState::GetFlightControllerAxis(int axisNumber)
{
    if ((axisNumber < 0) || (axisNumber >= flightcontrollerAxisCount))
    {
        printf("Invalid flightcontroller axis number %d!\n", axisNumber);
        return 0;
    }
    else 
    {
        return (flightcontrollerAxisCmd[axisNumber]);
    }
}
ButtonState JoystickState::GetDpadUpButton()
{
    return DpadUp;
}
ButtonState JoystickState::GetDpadRightButton()
{
    return DpadRight;
}
ButtonState JoystickState::GetDpadDownButton()
{
    return DpadDown;
}
ButtonState JoystickState::GetDpadLeftButton()
{
    return DpadLeft;
}
bool JoystickState::GetDpadUpPushed()
{
    if (DpadUp == kPressing || DpadUp == kHeld)
        return true;
    else
        return false;
}
bool JoystickState::GetDpadRightPushed()
{
    if (DpadRight == kPressing || DpadRight == kHeld)
        return true;
    else
        return false;
}
bool JoystickState::GetDpadDownPushed()
{
    if (DpadDown == kPressing || DpadDown == kHeld)
        return true;
    else
        return false;
}
bool JoystickState::GetDpadLeftPushed()
{
    if (DpadLeft == kPressing || DpadLeft == kHeld)
        return true;
    else
        return false;
}

ButtonState JoystickState::BtnStateMachine(bool curPressed, ButtonState curState)
{
    ButtonState newState;
    newState = curState;
     if(curPressed)
     {
        switch (curState)
        {
            case kOff:
                newState = kPressing;
                break;
            case kPressing:
                newState = kHeld;
                break;
            case kHeld:
                newState = kHeld;
                break;
            case kReleasing:
                newState = kPressing;
                break;
            default:
                printf("Error! How did we get here?\n");
                break;
        }
     }
     else
     {
         switch (curState)
         {
            case kOff:
                newState = kOff;
                break;
            case kPressing:
                newState = kReleasing;
                break;
            case kHeld:
                newState = kReleasing;
                break;
            case kReleasing:
                newState = kOff;
                break;
            default: 
                printf("Error! How did we get here?\n");
                break;
            
        }
    }
    return (newState);
}

float JoystickState::AxisShaping(float rawReading, float deadband, float shapingSlope)
{
    float modifiedReading;
    if (fabs(rawReading) <= deadband)
    {
        modifiedReading = 0.0;
    }
    else if (rawReading > 0.0)
    {
        modifiedReading = (rawReading - deadband)/(1.0 - deadband);
    }
    else
    {
        modifiedReading = (rawReading + deadband)/(1.0 - deadband);
    }
    //printf("%f %f %f %f ", rawReading, deadband, shapingSlope, modifiedReading);
    modifiedReading = modifiedReading * (1.0 + 0.5 * shapingSlope * (fabs(modifiedReading) - 1.0));
    //printf("  %f \n", modifiedReading);
    return (modifiedReading);
}

//Called every loop (used for timing related stuff)
void JoystickState::Analyze()
{
    if (gamepadBtnCount == 0 || flightcontrollerBtnCount == 0) JoystickCountInitialize();

    int btn;
    //printf("gamepadBtnCount %d: ", gamepadBtnCount);
    for(btn = 0; btn < gamepadBtnCount; btn++)
    {
        gamepadBtnState[btn] = BtnStateMachine(gamepad->GetRawButton(btn + 1), gamepadBtnState[btn]);
      //  printf("%d", btn);
    }
  //  printf("\n");

    for(btn = 0; btn < flightcontrollerBtnCount; btn++)
    {
        flightcontrollerBtnState[btn] = BtnStateMachine(flightcontroller->GetRawButton(btn + 1), flightcontrollerBtnState[btn]); 
    }
    int axis;
    for(axis = 0; axis < gamepadAxisCount; axis++)
    {
        gamepadAxisCmd[axis] = AxisShaping(gamepad->GetRawAxis(axis), gamepadDeadband[axis], gamepadShaping[axis]);
    }
    for(axis = 0; axis < flightcontrollerAxisCount; axis++)
    {
        flightcontrollerAxisCmd[axis] = AxisShaping(flightcontroller->GetRawAxis(axis), flightcontrollerDeadband[axis], flightcontrollerShaping[axis]);
    } 
    if (gamepadPOVCount >= 1)
    {
        int angle = gamepad->GetPOV();   
        DpadUp = BtnStateMachine(angle == Up, DpadUp);
        DpadRight = BtnStateMachine(angle == Right, DpadRight);
        DpadDown = BtnStateMachine(angle == Down, DpadDown);
        DpadLeft = BtnStateMachine(angle == Left, DpadLeft);
    }
}