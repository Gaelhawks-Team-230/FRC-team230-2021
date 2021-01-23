#ifndef SHOOTERTURRET_H_
#define SHOOTERTURRET_H_
#include "TalonXXI_main.h"
#include "SensorState.h"
#include "LimeLight.h"
#include "Common.h"
#include "frc/smartdashboard/Smartdashboard.h"
#include <frc/VictorSP.h> 
#include <frc/AnalogPotentiometer.h>

#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"

class TalonXXI;

#define TURRET_OVERRIDE_CONSTANT                (1.0)
#define TURRET_MANUAL_DEADBAND                  (0.1)
#define TURRET_MIN_ANGLE                        (30.0)
#define TURRET_MAX_ANGLE                        (250.0)
#define TURRET_MID_DEADBAND                     ((TURRET_MIN_ANGLE + TURRET_MAX_ANGLE)/2 + 180.0)
#define TURRET_AT_GOAL_ALLOWANCE                (1.0)
#define TURRET_MIN_CMD                          (-1.0)
#define TURRET_MAX_CMD                          (1.0)

#define TURRET_KP                               (5.0)
#define TURRET_K_BANDWIDTH                      (10.0)
#define TURRET_K                                (206.67)//(204.0)//
#define TURRET_TAU                              (0.045)//(0.08)//
#define TURRET_MIN_VELOCITY                     (-100.0)
#define TURRET_MAX_VELOCITY                     (100.0)
#define TURRET_MAX_VELOCITY_ERROR_INT           (fabs(TURRET_K/TURRET_K_BANDWIDTH))
#define TURRET_MIN_VELOCITY_ERROR_INT           (-1.0 * TURRET_MAX_VELOCITY_ERROR_INT)
#define TURRET_SPEED                         (100.0)
#define TURRET_MAX_DELTA_POS                 (TURRET_SPEED * LOOPTIME)
#define TURRET_MANUAL_DELTA                  (0.99 * TURRET_MAX_DELTA_POS)

#define TURRET_K_TRACKING                     (3.0)
#define TURRET_K_VEL_JOYSTICK                 (30.0)
#define TURRET_KP_NO_TRACKING                 (0.01)
#define TURRET_FACE_BACK_POS                      (55.0)
#define TURRET_COLOR_WHEEL_POS                (234.0)
#define TURRET_TRACKING_POS                   (200.0) 
#define LIMELIGHT_AT_TARGET_RANGE             (1.0)
#define LIMELIGHT_START_TRACKING_RANGE        (20.0)

#define TURRET_NO_TRACK_POS_ERR               (40.0)//(20.0)
#define TURRET_NO_TRACK_JOYSTICK_K            (2.0)//(0.5)
#define TURRET_NO_TRACK_JOYSTICK_DEADBAND     (0.1)
#define TURRET_VEL_CMD_LIMIT                  (30.0)
#define TURRET_IN_DEADBAND_CMD                (0.2)

#define TURRET_CENTER_AUTO_POS                (234.0)
#define TURRET_TRENCH_AUTO_POS                (234.0)
#define TURRET_FEEDER_AUTO_POS                (234.0)
#define TURRET_TRENCH_FRONT_POS               (200.0)

class ShooterTurret
{
    private:
        TalonXXI *mainRobot;
        frc::VictorSP *turretMotor;
        SensorState *localSurveillance;
        //LimelightCamera *localLimelight;
        frc::AnalogInput *turret;
      //  frc::ADXRS450_Gyro *robotGyro;
        //frc::ADXRS450_Gyro *turretGyro;
      //  std::shared_ptr<NetworkTable> table;

        int loopCount;
        //bool isTurretCalibrating;
        double turretCmd;
        double turretGyroPos;
        double turretGyroVel;
        double turretPosz;

        double turretAnalogPos;
        double turretAnalogPosz;
        double turretAnalogVel;
        bool isLimelightTracking;

        double turretGoalPos;
        double turretPCmd;
        double turretPosError;
        double turretCurVel;
        double turretVelCmd;
        double turretVelErr;
        double turretErrInt;
        bool isTurretAtGoal;
        bool isTurretOverride;
        bool isTurretManual;

        double robotBaseVel;
        double robotBaseVelz;
        double robotBaseAccel;

        bool seesTarget;
        double targetPos;

    public:
        ShooterTurret(TalonXXI* pRobot);

        void LocalReset(void);
        void StartingConfig(void);
        void StopAll(void);
        void UpdateDash(void);
        void Service(void);

        //bool IsTurretCalibrating(void);
        void TurnOnTurretOverride(void);
        void TurnOffTurretOverride(void);
        bool IsTurretAtGoal(void);
        bool IsTurretOverride(void);
        double GiveTurretGoal(void);
        double GiveTurretPos(void);
        bool TurretAtMax(void);
        bool TurretAtMin(void);
        void TurretControl(void);

        void SetTargetingValues(void);
        void StopTargeting(void);
        void ManualTurretMove(double);
        void GiveGoalAngle(double);
        //void Recalibrate(void);
        void TurretStop(void);

        bool IsInTrackingRange(void);
        bool IsReadyToShoot(void);

        void Testing(double);
};
#endif /*Sample_H_*/