#ifndef DEATHSTAR_H_
#define DEATHSTAR_H_

#include <frc/VictorSP.h>
#include <frc/DigitalInput.h>
#include <frc/DoubleSolenoid.h>


#include "Common.h"
#include "SensorState.h"
#include "TalonXXI_main.h"

class TalonXXI;
class SensorState;

//#defines

#define INTAKE_POS      (0)
#define SHOOTING_POS    (2)
#define SLOT_COUNT      (5)

//Service switch
#define IDLE_MODE       (0)
#define INTAKE_MODE     (1)
#define PREP_MODE       (2)
#define SHOOT_MODE      (3)

#define KICK_SPEED      (-1.0)
#define DEATH_STAR_POS_ERR_K        (0.0125)
#define DEATH_STAR_MIN_CMD          (-0.6)
#define DEATH_STAR_MAX_CMD          (0.6)

#define RETRACTED_KICK  (frc::DoubleSolenoid::Value::kReverse) 
#define EXTENDED_KICK   (frc::DoubleSolenoid::Value::kForward) 
#define DISABLE_KICK    (frc::DoubleSolenoid::Value::kOff)

#define ONE_SLOT_ANGLE       (72.0)
#define HALF_SLOT_ANGLE      (36.0)

#define LOADING_TOLERANCE    (10.0)
#define SHOOTING_TOLERANCE   (10.0)

#define SHOOTING_OFFSET      (0.0)
#define INTAKE_OFFSET        (0.0)

#define BALL_SETTLE_DELAY    ((int)(0.1*N1SEC))
#define SHOOT_WAIT_DELAY    ((int)(0.5*N1SEC))

class DeathStar
{
    private:
       // Create objects needed by this class
		// example: VictorSP *sampleMotor;
        frc::VictorSP *starMotor;
        frc::VictorSP *kickMotor;
        frc::DoubleSolenoid *kickSolenoid;

        TalonXXI *mainRobot;
        SensorState *localSurveillance;

        //declare member variables
        int loopCount;
        bool isIntakeMode;
        bool isShootingMode;
        bool hasCell[SLOT_COUNT];
        int cellCount;
        double spinSpeed;
        double kickSpeed;
        frc::DoubleSolenoid::Value kickPos;
        //bool isSpinning;
        double goalPos;
        double currentPos;
        int mode;
        double pcmd;
        double poserr;
        int ballSettleCount;
        int shootWaitCount;
        double tolerance;
        double alignOffset;
        double deathStarCurrent;


    public:
        DeathStar(TalonXXI* pRobot);
        //Functions
        void SetIdle(void);
        void PrepToShoot(void);
        void FireAtWill(void);
        void CheckCells(void);
        void IntakeCells(void);
        bool IsInPosition();
        bool IsEmpty();
        bool IsFull();
        bool HasCellAtShootPos();
        bool HasCellAtIntakePos();
        int GetCellCount();

        void LocalReset(void);
        void StartingConfig(void);
        void StopAll(void);
        void UpdateDash(void);
        void Service(void);
        void ModeService(void);

        void Testing(float);
        void Increasepcmd(void);
        void Decreasepcmd(void);

        void TestRetractKicker(void);
        void TestExtendKicker(void);
        void GetCurrent(void);

    private:
        void SpinStarService(void);
        void SpinOneSlot(void);
        void KickCell(void);

};
#endif /*DeathStar_H_*/