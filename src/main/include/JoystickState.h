/*
* JoystickState.h
*
* Created on: Jan 11, 2020
*		Author: Ben DeMartino
*/
#ifndef JoystickState_H_
#define JoystickState_H_
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Joystick.h>

class TalonXXI;

//#defines
#define MAX_JOYSTICK_BUTTONS     25
#define MAX_JOYSTICK_AXES        8
#define DRIVE_DEADBAND           0.07
//#define NUM_POV_BUTTONS          4
//example: #define LEVEL_ONE_HEIGHT     (23.0)

enum ButtonState {kOff, kPressing, kHeld, kReleasing}; 
enum Gampad_POV {Up = 0, Right = 90, Down = 180, Left = 270};

class JoystickState
{
    public:
    
    
        //enum ButtonState {kOff, kPressing, kHeld, kReleasing}; 
        /*UP is kOff, P is kPressing, H is kHeld, R is kReleasing
        UP (True) to P, UP (False) to UP, P (True) to H, P (False) to R, 
        H (False) to R, H (true) to H R(True) to P, R (False) to UP */
   

    private:
        TalonXXI *mainRobot;
        frc::Joystick *flightcontroller;
        frc::Joystick *gamepad;

        ButtonState gamepadBtnState [MAX_JOYSTICK_BUTTONS];
        int gamepadBtnCount;
        ButtonState flightcontrollerBtnState [MAX_JOYSTICK_BUTTONS];
        int flightcontrollerBtnCount;

        double gamepadAxisCmd [MAX_JOYSTICK_AXES];
        int gamepadAxisCount;
        double gamepadDeadband[MAX_JOYSTICK_BUTTONS];
        double gamepadShaping[MAX_JOYSTICK_BUTTONS];
        int gamepadPOVCount;
        int flightcontrollerAxisCount;
        double flightcontrollerDeadband[MAX_JOYSTICK_BUTTONS];
        double flightcontrollerAxisCmd [MAX_JOYSTICK_AXES];
        double flightcontrollerShaping[MAX_JOYSTICK_BUTTONS];
        
        ButtonState DpadUp;
        ButtonState DpadRight;
        ButtonState DpadDown;
        ButtonState DpadLeft;

        

    public:
        JoystickState(TalonXXI* pRobot);

        void JoystickCountInitialize(void);
        void LocalReset(void);
        void StartingConfig(void);
        void StopAll(void);
        void UpdateDash(void);
        void Analyze(void);

        ButtonState GetGamepadButton(int btnNumber);
        ButtonState GetFlightControllerButton(int btnNumber);
        bool GamepadBtnPushed(int btnNumber);
        bool FlightCtrlBtnPushed(int btnNumber);

        double GetGamepadAxis(int btnNumber);
        double GetFlightControllerAxis(int btnNumber);
        
        ButtonState GetDpadUpButton();
        ButtonState GetDpadRightButton();
        ButtonState GetDpadDownButton();
        ButtonState GetDpadLeftButton();

        bool GetDpadUpPushed();
        bool GetDpadRightPushed();
        bool GetDpadDownPushed();
        bool GetDpadLeftPushed();

        void SetGamepadAxisParameter(int axisNum, double deadband, double shaping);
        void SetFlightControllerAxisParameter(int axisNum, double deadband, double shaping);
    private:
        ButtonState BtnStateMachine(bool curPressed, ButtonState curState);
        double AxisShaping(double rawReading, double deadband, double shapingSlope);
        
};
#endif /*JoystickState_H_*/