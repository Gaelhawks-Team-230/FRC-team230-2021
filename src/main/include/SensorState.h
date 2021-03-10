#ifndef SENSORSTATE_H_
#define SENSORSTATE_H_

#include "TalonXXI_main.h"
#include "Common.h"
#include "LimeLight.h"
#include "frc/smartdashboard/Smartdashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
//#include <frc/util/color.h>
//#include "rev/ColorSensorV3.h"
//#include "rev/ColorMatch.h"
#include <frc/Encoder.h>
#include <frc/DigitalInput.h>
#include <frc/ADXRS450_Gyro.h>
#include <frc/AnalogInput.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/AnalogGyro.h>

//shooter
#define SHOOTER_ENCODER_PULSE_COUNT (1024.0)
#define SHOOTER_DISTANCE_PER_PULSE (1.0)
#define SHOOTER_VEL_K (10.0 * 2.0 * PI / 2048)

//shrouding
#define ANGLE_ENCODER_PULSE_COUNT (1024.0)
#define ANGLE_DISTANCE_PER_PULSE (2.0 * PI / 2048) //(1.0)

//turret
/*#ifdef PRACTICE_BOT
#define TURRET_DEGREES_PER_VOLT     (75.0217)//(72.0)
#else
#define TURRET_DEGREES_PER_VOLT     (72.72)//(73.0)
#endif
#ifdef PRACTICE_BOT
#define TURRET_ANALOG_OFFSET        (0.0)//(-43.0)//(181.0)
#else
#define TURRET_ANALOG_OFFSET        (0.0)//(-254.0)
#endif*/
#define SPIN_ENCODER_PULSE_COUNT (1024.0)
#define SPIN_DISTANCE_PER_PULSE (1.0)
#define TURRET_DEGREE_RANGE (360.0)
#define TURRET_GYRO_READING_K (-1.05)

/*
//climber
//#define CLIMBER_ENCODER_PULSE_COUNT            (1024.0)
#define CLIMB_EXTEND_DISTANCE_PER_PULE       (-24.0) //SENSOR DELTA TWO is 4 ft
#define CLIMB_EXTEND_OFFSET                  (4.0)//(-43.95)
#define WINCH_DISTANCE_PER_PULSE             (1.0)

//color wheel
#define COLOR_ENCODER_PULSE_COUNT    (1024.0)
#define COLOR_DISTANCE_PER_PULSE     (1.0)
*/

//drivetrain
#define WHEEL_CIRCUMFERENCE (6.0 * PI) //inches
#define DRIVE_ENCODER_PULSE_COUNT (2048.0)
#define DRIVE_GEAR_RATIO (8.45)
#define DRIVE_DISTANCE_PER_PULSE (WHEEL_CIRCUMFERENCE / (DRIVE_ENCODER_PULSE_COUNT * DRIVE_GEAR_RATIO))
#define MAX_PERIOD (1.0)

//Death star
#define DEATH_STAR_DISTANCE_PER_ROTATION (360.0)
/*#ifdef PRACTICE_BOT
#define DEATH_STAR_OFFSET       (-14.5)//(14.5)
#else
#define DEATH_STAR_OFFSET       (-14.5)
#endif*/

class TalonXXI;

class SensorState
{
private:
        TalonXXI *mainRobot;

        int loopCount;

        //shooter
        ctre::phoenix::motorcontrol::can::WPI_TalonFX *shooterFalcon;
        double shooterCurrentPos;
        double shooterPosRadians;
        double shooterRadPerSec;
        double shooterOldPosRadians;

        //shrouding
        frc::Encoder *shroud;
        double shroudingAngleRadians;
        double shroudingAngleRadiansz;
        double shroudingRadPerSec;

        //turret
        frc::AnalogInput *turretAnalog;
        frc::AnalogGyro *turretGyro;
        double turretAnalogAngleRaw;
        double turretAnalogAngleMod;
        double turretAnalogAnglez;
        double turretAnalogVel;
        double turretGyroAngle;
        double turretGyroAnglez;
        double turretGyroVel;

        /*
        //climber
        frc::DutyCycleEncoder *climbDeliver;
        frc::Encoder *winch;
        double winchDist;
        double climbDis;
        double oldClimbDis;
        double currentClimbVel;

        //color wheel
        static constexpr auto i2cPort = frc::I2C::Port::kOnboard;
        rev::ColorSensorV3 m_colorSensor{i2cPort};
        rev::ColorMatch m_colorMatcher;
        static constexpr frc::Color kBlueTarget = frc::Color(0.143, 0.427, 0.429);
        static constexpr frc::Color kGreenTarget = frc::Color(0.197, 0.561, 0.240);
        static constexpr frc::Color kRedTarget = frc::Color(0.561, 0.232, 0.114);
        static constexpr frc::Color kYellowTarget = frc::Color(0.361, 0.524, 0.113);
        frc::Color detectedColor;
        frc::Color matchedColor;
        char colorChar;
        std::string colorString;
        double confidence;
        */

        //drivetrain
        ctre::phoenix::motorcontrol::can::WPI_TalonFX *leftDriveFalcon;
        ctre::phoenix::motorcontrol::can::WPI_TalonFX *rightDriveFalcon;

        double leftWheelDisDrive;
        double rightWheelDisDrive;
        double averageWheelDisDrive;
        double oldDriveDis;
        double currentDriveVel;
        double leftEncoderOffset;
        double rightEncoderOffset;
        double leftRawReading;
        double rightRawReading;

        //#ifdef USE_GYRO
        frc::ADXRS450_Gyro *driveGyro;
        //#endif
        double gyroVelDRV;
        double gyroReadingDRV;
        double oldGyroReadingDRV;

        //deathstar
        frc::DigitalInput *cellSensor0;
        frc::DigitalInput *cellSensor1;
        frc::DigitalInput *cellSensor2;
        frc::DigitalInput *cellSensor3;
        frc::DigitalInput *cellSensor4;

        frc::DutyCycleEncoder *deathStar;
        double deathStarAngleDegrees;
        double deathStarAngleDegreesz;
        double deathStarDegreesPerSec;

public:
        SensorState(TalonXXI *pRobot);

        //Robot Functions
        void LocalReset(void);
        void StartingConfig(void);
        void StopAll(void);
        void UpdateDash(void);
        void Analyze(void);
        void ControlSystem(void);
        void ResetShootingEncoder(void);
        void ResetShroudingEncoder(void);
        void ResetTurretEncoder(void);
        void ResetWinchEncoder(void);
        void ResetClimbExtendEncoder(void);
        void ResetDriveEncoders(void);
        void ReadDriveEncoders(void);
        //void ResetDeathStarEncoder(void); // not needed for DutyCycleEncoder

        //shooter
        void InitShooterFalcon(ctre::phoenix::motorcontrol::can::WPI_TalonFX *shooterMotor);
        inline double GetPosRadians_Shooter() { return shooterPosRadians; };
        inline double GetPosRadPerSec_Shooter() { return shooterRadPerSec; };

        //shrouding
        inline double GetAngleRadians_Shrouding() { return shroudingAngleRadians; };
        inline double GetAngleRadPerSec_Shrouding() { return shroudingRadPerSec; };

        //turret
        inline double GetAngleAnalogRaw_Turret() { return turretAnalogAngleRaw; };
        inline double GetAngleAnalog_Turret() { return turretAnalogAngleMod; };
        inline double GetVelAnalog_Turret() { return turretAnalogVel; };
        inline double GetAngleGyro_Turret() { return turretGyroAngle; };
        inline double GetVelGyro_Turret() { return turretGyroVel; };
        inline double GetVelGyro_TurretDir() { return turretGyro->GetRate(); };

        /* inline double GetAngle_Turret() { return turretCurrentAngle; };
        inline double GetAngleRadians_Turret() { return turretAngleRadians; };
        inline double GetAngleRadPerSec_Turret() { return turretRadPerSec; };
        inline double GetDegreesPerSec_Turret() { return turretDegreesPerSec; };*/

        /*
        //climber
        inline double GetClimbExtendDis() { return climbDis; };
        inline double GetClimbWinchDis() { return winchDist; };
        inline double GetClimbeVel() { return currentClimbVel; };

        //color wheel
        inline frc::Color GetColorSensorReading() { return matchedColor; };
        inline char GetColorChar() {return colorChar; };
        */

        //drivetrain
        void InitDriveFalcons(ctre::phoenix::motorcontrol::can::WPI_TalonFX *leftMotor, ctre::phoenix::motorcontrol::can::WPI_TalonFX *rightMotor);
        inline double GetAverageDriveDis() { return averageWheelDisDrive; };
        inline double GetLeftDriveDis() { return leftWheelDisDrive; };
        inline double GetRightDriveDis() { return rightWheelDisDrive; };
        inline double GetAverageDriveVel() { return currentDriveVel; };
        inline double GetOldDriveDis() { return oldDriveDis; };

        inline double GetDriveGyroVelocity() { return gyroVelDRV; };
        inline double GetDriveGyroVelDirect() { return driveGyro->GetRate(); };
        inline double GetDriveGyroAngle() { return gyroReadingDRV; };

        //deathstar
        bool GetCellSensor(int);
        inline double GetAngle_DeathStar() { return deathStarAngleDegrees; };
        inline double GetDegreesPerSec_DeathStar() { return deathStarDegreesPerSec; };
};
#endif /*SensorState_H_*/