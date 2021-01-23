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

#define SHOOTER_OVERRIDE_CMD            (0.5)
#define SHOOTER_OVERRIDE_CONSTANT       (0.1)
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

#define SHOOTER_AUTO_VEL                        (500.0)
#define SHROUD_AUTO_ANGLE                       (0.05)

#define SHOOTER_FAR_VEL                         (500.0)
#define SHOOTER_MID_VEL                         (500.0)
#define SHOOTER_CLOSE_VEL                       (300.0)

#define SHROUD_MANUAL_DEADBAND          (0.1)
#define SHROUD_OVERRIDE_CONSTANT        (0.5)
#define SHROUD_MIN_ANGLE                (0.075)
#define SHROUD_MAX_ANGLE                (30.0)
#define SHROUD_AT_GOAL_ALLOWANCE        (0.5)
#define SHROUD_MIN_CMD                  (-0.5)//(-0.3)//(-1.0)
#define SHROUD_MAX_CMD                  (0.5)//(0.3)//(1.0)

#define SHROUD_TEST_KP                          (10.0)
#define SHROUD_KP                               (4.0)//(3.0)//6mm(4.0)
#define SHROUD_K_BANDWIDTH                      (12.0)//(25.0))//(8.0)//6mm(15.0)
#define SHROUD_K                                (6.0)//(1.0)//(6.0)//
#define SHROUD_TAU                              (0.01)//6mm(0.02)//(0.15)//
#define SHROUD_MIN_VELOCITY                     (-100.0)
#define SHROUD_MAX_VELOCITY                     (100.0)
#define SHROUD_MAX_VELOCITY_ERROR_INT           (fabs(SHROUD_K/SHROUD_K_BANDWIDTH))
#define SHROUD_MIN_VELOCITY_ERROR_INT           (-1.0 * SHROUD_MAX_VELOCITY_ERROR_INT)
#define SHROUD_SPEED                         (0.2)
#define SHROUD_MAX_DELTA_POS                 (SHROUD_SPEED * LOOPTIME)
#define SHROUD_MANUAL_DELTA                  (0.99 * SHROUD_MAX_DELTA_POS)
#define SHROUD_CALIBRATE_COMMAND              (-0.1)

#define SHROUD_BEHIND_COLOR_WHEEL_ANGLE     (0.39939)
#define SHROUD_INITIATION_LINE_ANGLE        (0.15)
#define SHROUD_FRONT_TRENCH_ANGLE           (0.28)

#define SHROUD_LIMELIGHT_Y_FRONT_WHEEL            (3.4)
#define SHROUD_LIMELIGHT_Y_BEHIND_WHEEL           (2.0)


#define SHROUD_LIMELIGHT_Y_FAR                    (2.7)
#define SHROUD_LIMELIGHT_Y_MID                      (10.0)
#define SHROUD_LIMELIGHT_Y_INITIATION_LINE        (16.2)
#define SHROUD_LIMELIGHT_Y_RANGE_FAR                  (2.0)
#define SHROUD_LIMELIGHT_Y_RANGE_MID                (3.0)
#define SHROUD_LIMELIGHT_Y_RANGE_CLOSE              (4.0)

class BallShooter
{
    private:
    
        TalonXXI *mainRobot;
        frc::PWMTalonSRX *shroudMotor;
        ctre::phoenix::motorcontrol::can::WPI_TalonFX *shooterMotor;
        ctre::phoenix::motorcontrol::SupplyCurrentLimitConfiguration shooterSupplyCurrent;
        ctre::phoenix::motorcontrol::StatorCurrentLimitConfiguration shooterStatorCurrent;

        SensorState *localSurveillance;

        bool isShooterCalibrating;
        bool isShroudCalibrating;
        int loopCount;   
        double shooterCmd;
        double shooterCurPos;
        double shooterCurPosz;
        double shooterCurPoszz;
        double shooterCurVel;
        double shooterCurVelOwn;
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
        bool isShooterManual;
        bool isShooterOverride;
       // double shooterCurrent;

        double shroudCmd;
        double shroudCurPos;
        double shroudCurPosz;
        double shroudDither;
        double shroudGoalPos;
        double shroudPCmd;
        double shroudPosError;
        double shroudCurVel;
        double shroudVelCmd;
        double shroudVelErr;
        double shroudErrInt;
        bool isShroudAtGoal;
        bool isShroudManual;
        bool isShroudOverride;
        int shroudCalState;

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

        void TurnOnShooterOverride(void);
        void TurnOffShooterOverride(void);
        bool IsShooterAtGoal(void);
        bool IsShooterOverride(void);
        double GetShooterGoalVel(void);
        double GetShooterVel(void);
        void ShooterControl(void);

        void TurnOnShroudOverride(void);
        void TurnOffShroudOverride(void);
        bool IsShroudAtGoal(void);
        bool IsShroudOverride(void);
        double GetShroudGoal(void);
        double GetShroudPos(void);
        bool IsShroudAtMax(void);
        bool IsShroudAtMin(void);
        void ShroudControl(void);
        void ShroudCalibrate(void);
        void SetTargeting(void);
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