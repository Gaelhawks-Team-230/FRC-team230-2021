
//#include "ctre/Phoenix.h"
#include "TalonXXI_main.h"
#include "Common.h"
#include "Drivetrain.h"

Drivetrain::Drivetrain(TalonXXI* pRobot)
{
    mainRobot = pRobot;
    localSurveillance = pRobot->surveillance;


    frontLeftMotor = new ctre::phoenix::motorcontrol::can::WPI_TalonFX (CAN_FRONT_LEFT);
    frontRightMotor = new ctre::phoenix::motorcontrol::can::WPI_TalonFX (CAN_FRONT_RIGHT);
    backLeftMotor = new ctre::phoenix::motorcontrol::can::WPI_TalonFX (CAN_BACK_LEFT);
    backRightMotor = new ctre::phoenix::motorcontrol::can::WPI_TalonFX (CAN_BACK_RIGHT);
    
    frontLeftMotor->SetNeutralMode(NeutralMode::Brake);
    frontRightMotor->SetNeutralMode(NeutralMode::Brake);
    backLeftMotor->SetNeutralMode(NeutralMode::Brake);
    backRightMotor->SetNeutralMode(NeutralMode::Brake);
    
    localSurveillance->InitDriveFalcons(frontLeftMotor, frontRightMotor);

   mainRobot->userInput->SetFlightControllerAxisParameter(SPEED_AXIS, 0.07, 0.7);
    mainRobot->userInput->SetFlightControllerAxisParameter(ROTATE_AXIS, 0.07, 0.7);
   // mainRobot->userInput->SetFlightControllerAxisParameter(SPEED_AXIS, 0.1, 0.7);
   // mainRobot->userInput->SetFlightControllerAxisParameter(ROTATE_AXIS, 0.1, 0.7);

    //frontLeftMotor->GetSensorCollection().SetIntegratedSensorPosition(0.0);
    //frontLeftMotor->Set(ControlMode::PercentOutput, frontLeftMotorCmd *0.75);
    LocalReset();
}

//Sets all local variables
void Drivetrain::LocalReset()
{
    rightMotorCmd = 0.0;
    leftMotorCmd = 0.0;

    gyroError = 0.0;
    curVel = 0.0;
    gyroErrorInt = 0.0;
    rotate = 0.0;
    gyroOn = false;
    gyroVel = localSurveillance->GetDriveGyroVelocity();
    localSurveillance->ReadDriveEncoders(); //HAVE TO DO TO INIT
    localSurveillance->ResetDriveEncoders();
    driveMod = 0.0;
    driveModz = 0.0;
    postShapingRotate = 0.0;
    
    driveCurVel = 0.0;
    driveCurErr = 0.0;
    driveErrInt = 0.0;

	gyroAngle = 0;

    loopCount = 0;
}

//Holds how the variables should be set when the robot starts
void Drivetrain::StartingConfig()
{

}


void Drivetrain::StopAll()
{
    LocalReset();
}




void Drivetrain::DriveControl(double driveCmd, double rotateCmd, double forcedDrive, double forcedRotate,bool isAuto, bool isTraj, double velCmd)
{
    ReceiveVelInfo();
    postShapingRotate = rotateCmd;
    if(isAuto)
    {
        if(gyroOn)
        {
            rotate = GyroControl(rotateCmd);
        }
        else
        {
            // use isAuto here to allow faster rotation
            rotate = rotateCmd * AUTO_ROTATE_NO_GYRO;
        }
        driveCmd = driveCmd*-1.0;
    }
    else
    {
        if(gyroOn)
        {
            rotate =  GyroControl(rotateCmd * COMMAND_RATE_MAX );
        }
        else
        {
            // use isAuto here to allow faster rotation
            rotate = ROTATE_CONSTANT *rotateCmd;
        }
    }
    if(isTraj)
    {
        driveMod = VelControl(velCmd);
    }
    else
    {
        driveMod = driveModz + TalonXXI::Limit(-DRIVE_MAX_ACCEL, DRIVE_MAX_ACCEL, (driveCmd - driveModz)*DRIVE_OMEGA)*LOOPTIME;
        driveModz = driveMod;
        driveMod = driveMod + forcedDrive;
        driveCurErr = 0.0;
        driveErrInt = 0.0;
        //driveMod = driveCmd;
    }
    rotate = rotate + forcedRotate;
    //TESTING
   /* if(rotateCmd > 0.2)
    {
        rotate = 0.2;
    }
    else if(rotateCmd < -0.2)
    {
        rotate = -0.2;
        
    }
    else
    {
        rotate = 0.0;
       
    }*/
    //driveCmd = 0.0;
    //left multplied by -1 because it needs to be inverted
    leftMotorCmd = -1.0*(TalonXXI::Limit(-1.0, 1.0, (driveMod - rotate)));
    rightMotorCmd = (TalonXXI::Limit(-1.0, 1.0, (driveMod + rotate)));

    frontLeftMotor ->Set(ControlMode::PercentOutput, leftMotorCmd);
    backLeftMotor->Set(ControlMode::PercentOutput, leftMotorCmd);
    frontRightMotor->Set(ControlMode::PercentOutput, rightMotorCmd);
    backRightMotor->Set(ControlMode::PercentOutput, rightMotorCmd);
    loopCount++;
    //printf("%d %f %f %f %f %f \n", loopCount, rotateCmd, rotateCmd*COMMAND_RATE_MAX, driveCmd, gyroVel);
    //printf("%d %f %f %f %f %f %f \n", loopCount, driveCmd, velCmd, driveMod, driveCurVel, driveCurErr, driveErrInt);
    printf("%d %f %f %f %f %f %f %f\n", loopCount, rotateCmd, velCmd, localSurveillance->GetLeftDriveDis(), localSurveillance->GetRightDriveDis(), localSurveillance->GetDriveGyroAngle(), driveMod, rotate);
}

double Drivetrain::VelControl(double driveVelCmd)
{
    driveCurErr = driveVelCmd - driveCurVel;
    driveErrInt = driveErrInt + (driveCurErr*LOOPTIME);
    driveErrInt = TalonXXI::Limit(DRIVE_LOW_LIMIT, DRIVE_HIGH_LIMIT, driveErrInt);
    double newVelCmd = (DRIVE_ROBOT_K_BANDWIDTH/DRIVE_ROBOT_K)*((driveCurErr*DRIVE_ROBOT_TAU) + driveErrInt);
    return newVelCmd;
}

void Drivetrain::ReceiveVelInfo()
{
    driveCurVel = -localSurveillance->GetAverageDriveVel()/12;
}

void Drivetrain::GyroOff()
{
    gyroOn = false;
#ifdef USE_GYRO
    gyroErrorInt = 0.0;
#endif
}

void Drivetrain::GyroOn()
{
#ifdef USE_GYRO
    gyroOn = true;
#else
    gyroOn = false;
#endif
}

double Drivetrain::GyroControl(double velCmd)
{
#ifdef USE_GYRO
    ReceiveGyroInfo();
    //printf ("%f\n", velCmd);
    gyroError = velCmd - gyroVel;
    gyroErrorInt = gyroErrorInt + (gyroError*LOOPTIME);
    gyroErrorInt = TalonXXI::Limit(LOW_LIMIT, HIGH_LIMIT, gyroErrorInt);
    double newCmd = (ROBOT_K_BANDWIDTH/ROBOT_K)*((gyroError*ROBOT_TAU) + gyroErrorInt);
    return newCmd;
#else
    return motorCmd;
#endif
}

void Drivetrain::ReceiveGyroInfo()
{
#ifdef USE_GYRO
    gyroVel = localSurveillance->GetDriveGyroVelocity();
    gyroAngle = localSurveillance->GetDriveGyroAngle();
#endif
}

//Anything that you want printed on the dashboard for testing or during match. 
//Any information which may be important. 
void Drivetrain::UpdateDash()
{
    // Example: 
    //frc::SmartDashboard::PutNumber("Post shaping rotate", postShapingRotate);
}

//Called every loop (used for timing related stuff)
void Drivetrain::Service()
{
    
}

//ctre::phoenix::motorcontrol::TalonFXSensorCollection& ctre::phoenix::motorcontrol::can::TalonFX::GetSensorCollection()
