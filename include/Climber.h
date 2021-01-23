#ifndef CLIMBER_H_
#define CLIMBER_H_
#include "TalonXXI_main.h"
#include "SensorState.h"
#include "Common.h"
#include "frc/Solenoid.h"
class TalonXXI;

//#defines
//example: #define LEVEL_ONE_HEIGHT     (23.0)
#define CLIMB_PISTON_EXTEND        (true)

#define CLIMB_PISTON_OFF           (false)
#define WINCH_EXTEND               (1.0)
#define WINCH_RETRACT              (-1.0)
#define MAX_WINCH_DISTANCE         (1.0)
                                            
#define IDLE                       (0)
#define HOOK_DEPLOYED              (1)
#define IS_CLIMBING                (2)
#define CLIMB_COMPLETE             (3)
#define IS_BALANCED                (4)

class Climber
{
    private:
       // Create objects needed by this class
		// example: VictorSP *sampleMotor;
        TalonXXI *mainRobot;
        frc::VictorSP *climbExtendMotor;
        frc::VictorSP *climbWinchMotor;
        //frc::Solenoid *climbPiston;
        SensorState *localSurveillance;

        //declare member variables
        double winchDistance;
        double extendMotorCmd;
        double winchMotorCmd;
        double pistonCmd;
        int mode;


    public:
        Climber(TalonXXI* pRobot);
        //Functions
        void LocalReset(void);
        void StartingConfig(void);
        void DeployHook(void);
        void SetClimb(void);
        void StopClimb(void);
        bool IsClimbing(void);
        bool IsBalanced(void);
        bool IsClimbComplete(void);
        double GetWinchDistance(void);
        void StopAll(void);
        void UpdateDash(void);
        void Service(void);
        void ControlSystem(void);
        void ExtendTesting(double);
        void WinchTesting(double);
};
#endif /*Sample_H_*/