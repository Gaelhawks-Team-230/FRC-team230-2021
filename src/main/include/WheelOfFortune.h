#ifndef WHEELOFFORTUNE_H_
#define WHEELOFFORTUNE_H_

//clude "TalonXXI_main.h"
#include "Common.h"
#include <frc/VictorSP.h>
#include <frc/util/color.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>
#include "SensorState.h"
//#include "rev/ColorSensorV3.h"
//#include "rev/ColorMatch.h"

class TalonXXI;

//#defines
//example: #define LEVEL_ONE_HEIGHT     (23.0)

#define WHEEL_IDLE_MODE (0)
#define SPIN_MODE (1)
#define MATCH_MODE (2)

#define COLOR_RED (3)
#define COLOR_YELLOW (4)
#define COLOR_GREEN (5)
#define COLOR_BLUE (6)

#define COLOR_FOR_SPIN  ('B')

//Diameter is in inches 
#define DIAMETER_OF_COLORWHEEL (32.0)
#define DIAMETER_OF_SPINNY_WHEEL (2.875)
#define AMOUNT_OF_STAGE2_SPINS (3.5)

#define SPIN_ROTATE_CMD     (0.5)
#define MANUAL_WHEEL_DEADBAND    (0.5)

#define COLOR_WHEEL_ADDED_DRIVE     (-0.07)


class WheelOfFortune
{
    private:
       // Create objects needed by this class
		frc::VictorSP *spinnyMotor; //defined motor object 
        frc::Color currentColor;//this is the current color reading from the sensor
        int operationMode;//int that sets operation mode (Distance, Color, or Idle)
        double motorCommand; //double that's a motor command
        TalonXXI *mainRobot;
        SensorState *localSurveillance;

        //declare member variables
        //example: float height;
        int colorCount;
        char curColor;
        char lastColor;
        char goalColor;
        double isCompleteSpin;

        bool isManualSpin;


    public:
        WheelOfFortune(TalonXXI* pRobot);
        //Functions
        void LocalReset(void);
        void StartingConfig(void);
        void StopAll(void);
        void StartWheelRotate(void);
        void StartColorMatch(void);
        void UpdateDash(void);
        void Service(void);

        bool LookForColor(char, int);
        void GetMatchColor(void);
        void SetIdleMode(void);
        void ManualWheelMove(double);
        void Testing(double);
};
#endif /*WheelOfFortune_H_*/