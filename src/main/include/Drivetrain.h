#ifndef DRIVETRAIN_H_
#define DRIVETRAIN_H_

#include "ctre/Phoenix.h"
#include "ctre/phoenix/motorcontrol/can/WPI_TalonFX.h"
#include "ctre/phoenix/motorcontrol/TalonFXSensorCollection.h"
#include "SensorState.h"
#include "TalonXXI_main.h"

class TalonXXI;
class SensorState;

//#defines
//example: #define LEVEL_ONE_HEIGHT     (23.0)
#define ROBOT_K_BANDWIDTH               (12.0)
#define ROBOT_K                         (780.0)
#define HIGH_LIMIT                      (ROBOT_K/ROBOT_K_BANDWIDTH)
#define LOW_LIMIT                       (-HIGH_LIMIT)
#define ROBOT_TAU                       (0.16)
#define ROTATE_CONSTANT                 (0.3)//(0.2) //used when gyro off
#define COMMAND_RATE_MAX                (150.0)//Radians/sec 

#define MAX_AUTO_ACCELERATION           (120.0*LOOPTIME)
#define AUTO_ROTATE_NO_GYRO             (0.002)

//#define DRIVE_OMEGA                     (2.0)//(4.0)
#define DRIVE_MAX_ACCEL                 (DRIVE_OMEGA/2)

class Drivetrain
{
    private:
       // Create objects needed by this class
		// example: VictorSP *sampleMotor;
        ctre::phoenix::motorcontrol::can::WPI_TalonFX *frontLeftMotor;
        ctre::phoenix::motorcontrol::can::WPI_TalonFX *frontRightMotor;
        ctre::phoenix::motorcontrol::can::WPI_TalonFX *backLeftMotor;
        ctre::phoenix::motorcontrol::can::WPI_TalonFX *backRightMotor;
        
        TalonXXI *mainRobot;
        SensorState *localSurveillance;
        //add a pointer to sensor class when it is created

        //declare member variables
        //example: float height;
        double postShapingRotate;
        double leftMotorCmd;
        double rightMotorCmd;

        double gyroVel;
	    double gyroAngle;

        double gyroError;
        double curVel; //current velocity
        double gyroCommand;
        double gyroErrorInt; //int is integral here
        double rotate;
        bool gyroOn;

        int loopCount;

        double driveMod;
        double driveModz;
    
    public:
        Drivetrain(TalonXXI* pRobot);
        //Functions
        void LocalReset(void);
        void StartingConfig(void);
        void StopAll(void);
        void DriveControl(double, double, double, double, bool);
        double GyroControl(double);
        void UpdateDash(void);
        void Service(void);
        void GyroOff(void);
        void GyroOn(void);
        void ReceiveGyroInfo(void);
};
#endif /*Drivetrain_H_*/