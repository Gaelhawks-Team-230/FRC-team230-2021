
#ifndef CLIMBER_H_
#define CLIMBER_H_
#include "TalonXXI_main.h"
#include "SensorState.h"
#include "Common.h"
#include "frc/Solenoid.h"
class TalonXXI;

//#defines
//example: #define LEVEL_ONE_HEIGHT     (23.0)
#define WINCH_EXTEND               (1.0)
#define WINCH_RETRACT              (-1.0)
#define MAX_WINCH_DISTANCE         (1.0)

#define MIN_CLIMB_EXTEND_DIS        (0.0)
#define MAX_CLIMB_EXTEND_DIS        (48.0)
#define IS_CLIMBING_THRESHOLD       (5.0)
#define CLIMBER_EXTEND_K            (0.1)
#define CLIMBER_MANUAL_DELTA        (20.0*LOOPTIME)
#define CLIMBER_MAX_CMD             (0.6)
#define CLIMB_MID_DISTANCE          (24.0)


#define IDLE                       (0)
#define HOOK_DEPLOYED              (1)
#define IS_CLIMBING                (2)
#define CLIMB_COMPLETE             (3)
#define IS_BALANCED                (4)

class Climber
{
    private:
       // Create objects needed by this class
        TalonXXI *mainRobot;
        frc::VictorSP *climbExtendMotor;
        frc::VictorSP *climbWinchMotor;
        SensorState *localSurveillance;

        //declare member variables
        double extendPos;
        double winchPos;
        double extendMotorCmd;
        double winchMotorCmd;
        double goalPos;
        double extendErr;
        bool isManualMove;
        int loopCount;
        bool isClimbing;

    public:
        Climber(TalonXXI* pRobot);
        //Functions
        void LocalReset(void);
        void StartingConfig(void);
        void DeployHook(void);
        void SetClimb(void);
        void StopClimb(void);
        bool IsClimbing(void);
        void StopAll(void);
        void UpdateDash(void);
        void Service(void);

        void MoveExtender(double);
        void MoveWinch(double);
        void GiveGoalPos(double);

        void ExtendTesting(double);
        void WinchTesting(double);
};
#endif /*Sample_H_*/