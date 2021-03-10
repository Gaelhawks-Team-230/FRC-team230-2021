#ifndef BALLSHOOTER_H_
#define BALLSHOOTER_H_

//#ifdef BALLSHOOTER_H_
#include "TalonXXI_main.h"
#include "SensorState.h"
#include "Common.h"
#include "frc/smartdashboard/Smartdashboard.h"
#include <frc/VictorSP.h> 
#include <frc/Talon.h>
#include <frc/PWMTalonSRX.h>
#include "ctre/Phoenix.h"
#include "ctre/phoenix/motorcontrol/TalonFXSensorCollection.h"
#include "ctre/phoenix/motorcontrol/can/WPI_TalonFX.h"
#include "ctre/phoenix/motorcontrol/StatorCurrentLimitConfiguration.h"
#include "math.h"

class TalonXXI;

#define SHOOTER_MANUAL_DEADBAND         (0.1)
#define SHOOTER_AT_GOAL_ALLOWANCE       (2.0)

#define SHOOTER_K_BANDWIDTH                      (8.0)
#define SHOOTER_K                                (630.0)
#define SHOOTER_TAU                              (0.7)
#define SHOOTER_MIN_VELOCITY                     (-1000.0)
#define SHOOTER_MAX_VELOCITY                     (1000.0)
#define SHOOTER_MAX_VELOCITY_ERROR_INT           (fabs(SHOOTER_K/SHOOTER_K_BANDWIDTH))
#define SHOOTER_MIN_VELOCITY_ERROR_INT           (-1.0 * SHOOTER_MAX_VELOCITY_ERROR_INT)
#define SHOOTER_MIN_CMD                          (-1.0)
#define SHOOTER_MAX_CMD                          (1.0)
#define SHOOTER_ACCEL                         (150.0)
#define SHOOTER_MAX_DELTA_VEL                 (SHOOTER_ACCEL * LOOPTIME)
#define SHOOTER_MANUAL_DELTA_VEL                  (0.99 * SHOOTER_MAX_DELTA_VEL)

#define SHOOTER_LEAD_FILTER_A                   (2.0)
#define SHOOTER_LEAD_FILTER_B                   (-1.2)
#define SHOOTER_LEAD_FILTER_C                   (0.2)
#define SHOOTER_BANDWIDTH_MULTIPLIER            (1.0)
#define SHOOTER_HOLD_TIME                       (0.5*N1SEC)
#define SHOOTER_DELTA_VEL                       (-15.0)

#define SHOOTER_SUPPLY_CUR_LIMIT                (60.0)
#define SHOOTER_STATOR_CUR_LIMIT                (60.0)

/*#define SHOOTER_AUTO_VEL                        (300.0)
#define SHROUD_AUTO_ANGLE                       (0.05)*/ //DONT WANT TO USE

#define SHROUD_MANUAL_DEADBAND          (0.2)
#define SHROUD_MIN_ANGLE                (0.02)//(0.075)
/*#ifdef PRACTICE_BOT
#define SHROUD_MAX_ANGLE                (0.55)
#else
#define SHROUD_MAX_ANGLE                (0.50)
#endif*/
#define SHROUD_AT_GOAL_ALLOWANCE        (0.5)
#define SHROUD_MIN_CMD                  (-0.5)//(-0.3)//(-1.0)
#define SHROUD_MAX_CMD                  (0.5)//(0.3)//(1.0)

#define SHROUD_TEST_KP                          (10.0)
#define SHROUD_KP                               (4.0)//(3.0)//6mm(4.0)
#define SHROUD_K_BANDWIDTH                      (12.0)//(25.0))//(8.0)//6mm(15.0)
/*#ifdef PRACTICE_BOT
#define SHROUD_K                                (6.0)//(1.0)//(6.0)//
#else
#define SHROUD_K                                (2.0)//(1.0)//(6.0)//
#endif*/
#define SHROUD_TAU                              (0.01)//6mm(0.02)//(0.15)//
#define SHROUD_MIN_VELOCITY                     (-100.0)
#define SHROUD_MAX_VELOCITY                     (100.0)
#define SHROUD_MAX_VELOCITY_ERROR_INT           (fabs(SHROUD_K/SHROUD_K_BANDWIDTH))
#define SHROUD_MIN_VELOCITY_ERROR_INT           (-1.0 * SHROUD_MAX_VELOCITY_ERROR_INT)
#define SHROUD_SPEED                         (0.2)
#define SHROUD_MAX_DELTA_POS                 (SHROUD_SPEED * LOOPTIME)
#define SHROUD_MANUAL_DELTA                  (0.99 * SHROUD_MAX_DELTA_POS)
/*#ifdef PRACTICE_BOT
#define SHROUD_CALIBRATE_COMMAND              (-0.3)//(-0.2)
#else
#define SHROUD_CALIBRATE_COMMAND              (-0.4)//(-0.2)
#endif

#ifdef PRACTICE_BOT
#define SHROUD_TIC_THRESHOLD                (0.15)
#else
#define SHROUD_TIC_THRESHOLD                (0.2)
#endif*/
#define SHROUD_TIC_COUNT                    (5)
//#define SHROUD_CMD_OFFSET                   (0.05)
/*#ifdef PRACTICE_BOT
#define SHROUD_TIC_BANDWIDTH                (20.0)
#else
#define SHROUD_TIC_BANDWIDTH                (30.0)
#endif
//TARGETTING
#ifdef PRACTICE_BOT
#define SHROUD_INITIATION_LINE_ANGLE        (0.25)
#define SHROUD_FRONT_TRENCH_ANGLE           (SHROUD_BEHIND_COLOR_WHEEL_ANGLE)
#define SHROUD_BEHIND_COLOR_WHEEL_ANGLE     (0.466056)//(0.39939)
#define SHROUD_OTHER_LINE_ANGLE             (0.43)
#else
#define SHROUD_INITIATION_LINE_ANGLE        (0.25)
#define SHROUD_BEHIND_COLOR_WHEEL_ANGLE     (23.0*PI/180.0)//(0.39939)
#define SHROUD_FRONT_TRENCH_ANGLE           (SHROUD_BEHIND_COLOR_WHEEL_ANGLE)
#define SHROUD_OTHER_LINE_ANGLE             (0.43)
#endif*/


#define SHOOTER_INITIATION_LINE_VEL             (300.0)
#define SHOOTER_FRONT_TRENCH_VEL                (500.0)
#define SHOOTER_COLOR_WHEEL_VEL                 (500.0)
#define SHOOTER_FAR_LINE_VEL                    (500.0)

/*#ifdef PRACTICE_BOT
#define LIMELIGHT_Y_INITIATION_LINE_LOW             (10.0)
#define LIMELIGHT_Y_INITIATIOn_LINE_HIGH            (24.0)
#define LIMELIGHT_Y_FRONT_TRENCH_LOW                (4.0)
#define LIMELIGHT_Y_FRONT_TRENCH_HIGH                (10.0)
#define LIMELIGHT_Y_COLOR_WHEEL_LOW                 (1.2)
#define LIMELIGHT_Y_COLOR_WHEEL_HIGH                (4.0)
#define LIMELIGHT_Y_OTHER_LINE_LOW                  (0.3)
#define LIMELIGHT_Y_OTHER_LINE_HIGH                 (1.2)
#else
#define LIMELIGHT_Y_INITIATION_LINE_LOW             (15.0)
#define LIMELIGHT_Y_INITIATIOn_LINE_HIGH            (24.0)
#define LIMELIGHT_Y_FRONT_TRENCH_LOW                (8.5)
#define LIMELIGHT_Y_FRONT_TRENCH_HIGH                (15.0)
#define LIMELIGHT_Y_COLOR_WHEEL_LOW                 (6.0)
#define LIMELIGHT_Y_COLOR_WHEEL_HIGH                (8.5)
#define LIMELIGHT_Y_OTHER_LINE_LOW                  (4.0)
#define LIMELIGHT_Y_OTHER_LINE_HIGH                 (6.0)
#endif
*/
class BallShooter
{
    private:
    
        TalonXXI *mainRobot;
        frc::PWMTalonSRX *shroudMotor;
        ctre::phoenix::motorcontrol::can::WPI_TalonFX *shooterMotor;
        ctre::phoenix::motorcontrol::SupplyCurrentLimitConfiguration shooterSupplyCurrent;
        ctre::phoenix::motorcontrol::StatorCurrentLimitConfiguration shooterStatorCurrent;

        SensorState *localSurveillance;

        bool isShroudCalibrating;
        int loopCount;   
        double shooterCmd;
        double shooterCurPos;
        double shooterCurPosz;
        double shooterCurPoszz;
        double shooterCurVel;
        double shooterCurVelz;
        double shooterCurVelzz;
        double shooterCurVelzzz;
        double shooterGoalVel;

        int errCount;

        double shooterVelOutput;
        double shooterVelOutputz;
        double shooterVelOutputzz;

        double shooterVelCmd;
        double shooterErr;
        double shooterErrInt;
        double shooterErrIntz;
        double shooterErrIntzz;
        double shooterErrIntzzz;
        double shooterErrIntzzzz;
        bool isShooterAtGoal;
       // double shooterCurrent;

        double shroudCmd;
        double shroudCurPos;
        double shroudCurPosz;
        double shroudGoalPos;
        double shroudPCmd;
        double shroudPosError;
        double shroudCurVel;
        double shroudVelCmd;
        double shroudVelErr;
        double shroudErrInt;
        bool isShroudAtGoal;
        bool isShroudManual;
        int shroudCalState;
        int ticSum;

        double shroudGoalAngle;

        bool seesTarget;
        double targetYPos;

    public:

        BallShooter(TalonXXI* pRobot);
        //Functions
        void LocalReset(void);
        void ShooterReset(void);
        void ShroudReset(void);
        void StartingConfig(void);
        void StopAll(void);
        void UpdateDash(void);
        void Service(void);
        bool IsShooterCalibrating(void);
        bool IsShroudCalibrating(void);

        bool IsShooterAtGoal(void);
        double GetShooterGoalVel(void);
        double GetShooterVel(void);
        void ShooterControl(void);

        bool IsShroudAtGoal(void);
        double GetShroudGoal(void);
        double GetShroudPos(void);
        void ShroudControl(void);
        void ShroudCalibrate(void);
        
        void SetTargeting(void);
        void StopTargeting(void);
        bool ReadyToShoot(void);

        void SetShooterVelTargeting(void);
        void SetShroudAngleTargeting(void);
        void ManualShooterAdjust(double);
        void ManualShroudAdjust(double);
        void GiveShroudGoalAngle(double);
        void GiveShooterGoalVel(double);
        void Recalibrate(void);
        void ShooterStop(void);
        void ShroudStop(void);

        void TestingShooter(double);
        void TestingShroud(double);

      //  ctre::phoenix::motorcontrol::can::WPI_TalonFX *ReturnShooterObject() {return (ctre::phoenix::motorcontrol::can::WPI_TalonFX)(shooterMotor);}
    };

#endif /*Sample_H_*/