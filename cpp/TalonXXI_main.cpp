/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc/Timer.h>
#include <iostream>
#include "ctre/Phoenix.h"
#include "TalonXXI_main.h"
#include <frc/smartdashboard/SmartDashboard.h>

using namespace ctre::phoenix::motorcontrol;

TalonXXI::TalonXXI()
{
  pdp = new frc::PowerDistributionPanel();
  surveillance = new SensorState(this); // NOTE this object must be created first!!!
  userInput = new JoystickState(this);
  limelight = new LimelightCamera(this); //NOTE this object must be created before mechanisms
 
//#ifdef TEST_DRIVE
  drive = new Drivetrain(this);
//#endif
  shooter = new BallShooter(this);
 
  turret = new ShooterTurret(this);
 
  collector = new CellCollector(this);
  climb = new Climber(this);
//#endif
  deathStar = new DeathStar(this);
  colorWheel = new WheelOfFortune(this);

  loopCount = 0;
  
  AutoPositionChooser = new frc::SendableChooser<int>;
  AutoBallNumber = new frc::SendableChooser<int>;
  dashCounter = 0;
  isBlueAlliance = false;
  isAuto = false;
  firstTime = true;
  driveCmd = 0.0;
  rotateCmd = 0.0;
  autoStage = 0;
  autoMode = 0;
  autoStartPosition = 0;
  autoBallNumber = 0;
  isGyroOn = false;
  delayTime = 0.0;
  delayCount = 0;
  isDelay = false;
  isInTeleop = false;

  isJoystickCountInitialized = false;
  printf("constructor end\n");
}

void TalonXXI::RobotInit() 
{
  frc::SmartDashboard::PutNumber("Delay time", 0.0);

  AutoPositionChooser->AddOption("Feeder", FEEDER_POS);
	AutoPositionChooser->SetDefaultOption("Center", CENTER_POS);
	AutoPositionChooser->AddOption("Trench", TRENCH_POS);
  AutoPositionChooser->AddOption("Do Nothing", DO_NOTHING);
  AutoPositionChooser->AddOption("Baseline", BASELINE);
  AutoPositionChooser->AddOption("Barrel Path", BARREL_PATH);
  AutoPositionChooser->AddOption("Slalom Path", SLALOM_PATH);
  AutoPositionChooser->AddOption("Bounce Path", BOUNCE_PATH);
	frc::SmartDashboard::PutData("Position: ", AutoPositionChooser);

  AutoBallNumber->SetDefaultOption("One Ball", ONE_BALL);
  AutoBallNumber->AddOption("Two Ball", TWO_BALL);
  frc::SmartDashboard::PutData("Ball: ", AutoBallNumber);

  printf("robotinit before dash \n");

  ServiceDash();
  printf("robotinit after dash\n");
}

void TalonXXI::DisabledInit()
{
  Omnicide();
  limelight->TurnOffLED();
} 

void TalonXXI::DisabledPeriodic()
{
  //printf("disabledperiodic\n");
  //printf("before mode selection disabled periodic\n");
  ModeSelection();
  limelight->TurnOffLED();
  //printf("after mode selection disabled periodic\n");
  /*if(frc::DriverStation::GetInstance().IsDSAttached() && !isJoystickCountInitialized)
  {
    userInput->JoystickCountInitialize();
    isJoystickCountInitialized = true;
    printf("joystick count initialized\n");
  }*/
}

void TalonXXI::InitializeAlliance()
{
  if(frc::DriverStation::GetInstance().GetAlliance() == frc::DriverStation::kBlue)
		isBlueAlliance = true;
	else
		isBlueAlliance = false;
}

void TalonXXI::RobotPeriodic() 
{
  //printf("Robotperiodic\n");
    if(firstTime)
    {
      InitializeAlliance();
      firstTime = false;
    }
}

void TalonXXI::AutonomousInit() 
{
  isAuto = true;
  ModeSelection();
  loopCount = 0;
}

void TalonXXI::AutonomousPeriodic() 
{
  loopCount++;
  if(loopCount < delayCount)
  {
    isDelay = true;
  }
  else
  {
    isDelay = false;
    switch(autoMode)
    {
      case 0:
        DoNothing();
        break;

      case 1:
        InitiationLine();
        break;

      case 2:
        FeederShootFirst();
        break;

      case 3:
        CenterShootFirst();
        break;

      case 4:
        TrenchShootFirst();
        break;

      case 5:
        FeederShootSecond();
        break;

      case 6:
        CenterShootSecond();
        break;

      case 7:
        TrenchShootSecond();
        break;

      case 8:
        BarrelRacingPath();
        break;

      case 9:
        SlalomPath();
        break;

      case 10:
        BouncePath();
        break;
    }
  }
  CommunityService();
  ServiceDash();
}

void TalonXXI::TeleopInit() 
{
  isAuto = false;
  surveillance->Analyze();
  deathStar->LocalReset();
  turret->LocalReset();
  drive->GyroOn();
  limelight->TurnOffLED();
  //limelight->TurnOnLED();
}

void TalonXXI::TeleopPeriodic() 
{
  
  surveillance->Analyze();
  userInput->Analyze();
//#ifndef TEST_DRIVE
  limelight->Analyze();
  if(userInput->FlightCtrlBtnPushed(1))
  {
    drive->GyroOn();
  }
  else
  {
    drive->GyroOff();
  }
  
  if(userInput->GamepadBtnPushed(6))
  {
    collector->GrabCells();
  }
  else if(userInput->GamepadBtnPushed(5))
  {
    collector->EjectCells();
  }
  else
  {
    collector->StopGrab();
  }

  if(userInput->GamepadBtnPushed(1))
  {
    deathStar->IntakeCells();
  }
  else if(userInput->GamepadBtnPushed(2))
  {
    deathStar->PrepToShoot();
  //  limelight->TurnOnLED();
    shooter->GiveShooterGoalVel(500.0);
  }
  else if(userInput->GamepadBtnPushed(3))
  {
    deathStar->FireAtWill();
    shooter->GiveShooterGoalVel(500.0);
  }
  else
  {
    deathStar->SetIdle();
   // limelight->TurnOffLED();
    shooter->GiveShooterGoalVel(0.0);
  }
  if(userInput->GetGamepadButton(10) == kPressing)
  {
    deathStar->Decreasepcmd();
  }
  if(userInput->GetGamepadButton(9) == kPressing)
  {
    deathStar->Increasepcmd();
  }
  if(userInput->GamepadBtnPushed(8))
  {
    turret->SetTargetingValues();
    shooter->SetTargeting();
  }
  else
  {
    turret->StopTargeting();
  }
 /* if(userInput->GamepadBtnPushed(8))
  {
    deathStar->Testing(1.0);
  }
  else if(userInput->GamepadBtnPushed(7))
  {
    deathStar->Testing(-1.0);
  }
  else
  {
    deathStar->Testing(0.0);
  }
  if(userInput->GamepadBtnPushed(2))
  {
    deathStar->TestExtendKicker();
  }
  else
  {
    deathStar->TestRetractKicker();
  }
  if(userInput->GamepadBtnPushed(3))
  {
    shooter->GiveShooterGoalVel(500.0);
  }
  else
  {
    shooter->GiveShooterGoalVel(0.0);
  }*/
  /*if(userInput->GamepadBtnPushed(4))
  {
    shooter->GiveShroudGoalAngle(SHROUD_BEHIND_COLOR_WHEEL_ANGLE);
  }*/
  
  /*if(userInput->GetGamepadButton(7) == kPressing)
  {
    shooter->GiveShroudGoalAngle(-1.0);
  }
  else if(userInput->GetGamepadButton(8) == kPressing)
  {
    shooter->GiveShroudGoalAngle(1.0);

  }*/
 /* if(userInput->GamepadBtnPushed(5))
  {
    collector->GathererIn();
  }
  else (userInput->GamepadBtnPushed(6))
  {
    collector->GathererOut();
  }

  if(userInput->GamepadBtnPushed(7))
  {
    collector->TestingSpin(1.0);
  }*/
 /* if(userInput->GetGamepadButton(10) == kPressing)
  {
    shooter->TestingShroud(userInput->GetFlightControllerAxis(2));
  }
  else
  {
    shooter->TestingShroud(0.0);
  }*/
  
 /* if(userInput->GetGamepadButton(1) == kPressing)
  {
    deathStar->Increasepcmd();
  }
  else if(userInput->GetGamepadButton(2) == kPressing)
  {
    deathStar->Decreasepcmd();
  }
  else
  {
    deathStar->Testing(userInput->GetGamepadAxis(0));
  }*/
  
 //shooter->TestingShooter(userInput->GetGamepadAxis(0));
 // climb->ExtendTesting(userInput->GetGamepadAxis(0));
  /*climb->WinchTesting(userInput->GetGamepadAxis(2));*/
 // turret->Testing(userInput->GetFlightControllerAxis(0));

  drive->DriveControl(userInput->GetFlightControllerAxis(SPEED_AXIS), userInput->GetFlightControllerAxis(ROTATE_AXIS), false);
  CommunityService();
  ServiceDash();
}

void TalonXXI::ServiceDash()
{
  if(dashCounter == 20)
  {
    dashCounter = 0;
    userInput->UpdateDash();
  //#ifndef TEST_DRIVE
    surveillance->UpdateDash();
    shooter->UpdateDash();
    turret->UpdateDash();
    limelight->UpdateDash();
    deathStar->UpdateDash();
    collector->UpdateDash();
    colorWheel->UpdateDash();
  //#endif
    
   // climb->UpdateDash();
  }
  else
  {
    dashCounter++;
  }
  

}

void TalonXXI::CommunityService()
{
  //surveillance->Service(); - DO NOT CALL THIS HERE
  //userInput->Service(); - DO NOT CALL THIS HERE
  //limelight->Service(); - DO NOT CALL THIS HERE
  shooter->Service();
  turret->Service();
  deathStar->Service();
  collector->Service();
  colorWheel->Service();
  climb->Service();
}

void TalonXXI::Omnicide()
{
  surveillance->StopAll();
  userInput->StopAll();
//#ifndef TEST_DRIVE
  shooter->StopAll();
  turret->StopAll();
  limelight->StopAll();
  deathStar->StopAll();
//#endif
  collector->StopAll();
  colorWheel->StopAll();
  drive->StopAll();
  //climb->StopAll();

}

double TalonXXI::Limit(double min, double max, double curValue)
{
	if (curValue > max)
		return max;
	if (curValue < min)
		return min;
	return curValue;
}

double TalonXXI::Sign(double curValue)
{
  if(curValue < 0.0)
    return (-1.0);
  else
    return (1.0);
}

void TalonXXI::RobotStartingConfig()
{
  surveillance->StartingConfig();
  userInput->StartingConfig();
//#ifndef TEST_DRIVE
  shooter->StartingConfig();
  turret->StartingConfig();
  limelight->StartingConfig();
//#endif
  //collector->StartingConfig();
  //climb->StartingConfig();

}


void TalonXXI::TestPeriodic() 
{

}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<TalonXXI>(); }
#endif
