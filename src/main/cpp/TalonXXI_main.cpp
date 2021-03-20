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

//TalonXXI::TalonXXI()
TalonXXI::TalonXXI():TimedRobot(LOOPTIME)
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
  //climb = new Climber(this);
//#endif
  deathStar = new DeathStar(this);
  //lightStrip = new LightController(this);
  //colorWheel = new WheelOfFortune(this);
  camera = new VisionPi(this);

  planner = new TrajectoryPlan(this);

  loopCount = 0;
  
  AutoPositionChooser = new frc::SendableChooser<int>;
  AutoBallNumber = new frc::SendableChooser<int>;
  dashCounter = 0;
  isBlueAlliance = false;
  isAuto = false;
  isTraj = false;
  firstTime = true;
  driveCmd = 0.0;
  velCmd = 0.0;
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

  addedDrive = 0.0;
  addedRotate = 0.0;

  isJoystickCountInitialized = false;

  xstartPeriodic = 0.0;
  xendPeriodic = 0.0;
  xstartsurv = 0.0;
  xendsurv = 0.0;
  xstartcolor = 0.0;
  xmidcolor = 0.0;
  xendcolor = 0.0;

  isPathRead= false;
}

void TalonXXI::RobotInit() 
{
  frc::SmartDashboard::PutNumber("Delay time", 0.0);

  AutoPositionChooser->AddOption("Feeder", FEEDER_POS);
	AutoPositionChooser->SetDefaultOption("Center", CENTER_POS);
	AutoPositionChooser->AddOption("Trench", TRENCH_POS);
  AutoPositionChooser->AddOption("Do Nothing", DO_NOTHING);
  AutoPositionChooser->AddOption("Baseline", BASELINE);
  AutoPositionChooser->AddOption("Trajectory", TRAJ_PLANNER);
	frc::SmartDashboard::PutData("Position: ", AutoPositionChooser);

  AutoBallNumber->SetDefaultOption("Level One", LEVEL_ONE);
  AutoBallNumber->AddOption("Level Two", LEVEL_TWO);
  AutoBallNumber->AddOption("Test", TEST_TRAJ);
  AutoBallNumber->AddOption("Barrel", BARREL_TRAJ);
  AutoBallNumber->AddOption("Slalom", SLALOM_TRAJ);
  AutoBallNumber->AddOption("Bounce", BOUNCE_TRAJ);
  frc::SmartDashboard::PutData("Level: ", AutoBallNumber);
  ServiceDash();
  }

void TalonXXI::DisabledInit()
{
  Omnicide();
  limelight->TurnOffLED();
} 

void TalonXXI::DisabledPeriodic()
{
  ModeSelection(false);
  limelight->TurnOffLED();
  if(modeChange)
  {
    isPathRead = false;
  }
  if(!isPathRead)
  {
    isPathRead = planner->ReadPath(autoBallNumber);
  }
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
  //printf("Auto Init \n");
  isAuto = true;
  ModeSelection(true);
  surveillance->Analyze();
  deathStar->LocalReset();
  turret->LocalReset();
  drive->GyroOn();
  limelight->TurnOffLED();
  loopCount = 0;
  autoStage = 0;
}

void TalonXXI::AutonomousPeriodic() 
{
  loopCount++;
  surveillance->Analyze();
//#ifndef TEST_DRIVE
  limelight->Analyze();

  if(loopCount < delayCount)
  {
    isDelay = true;
  }
  else
  {
    isDelay = false;
    limelight->AutoHorizontalOffset(autoMode);
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
        TestSkillsChal();
        break;
    }
  }
  drive->DriveControl(driveCmd, rotateCmd, 0.0, 0.0, true, isTraj, velCmd);
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
  //climb->LocalReset();
  loopCount = 0;
  xstartPeriodic = 0.0;
  xendPeriodic = 0.0;
  xstartsurv = 0.0;
  xendsurv = 0.0;
  xstartcolor = 0.0;
  xendcolor = 0.0;
  /*if(firstTime)
  {
    turret->ResetTurretDialAdjust(userInput->GetFlightControllerAxis(6));
    firstTime = false;
  }*/
  //limelight->TurnOnLED();
}

void TalonXXI::TeleopPeriodic() 
{
  loopCount++;
  xstartPeriodic = frc::GetTime();
  surveillance->Analyze();
  camera->Analyze();
  xstartOtherAnalyze = frc::GetTime();
  limelight->AutoHorizontalOffset(-1);userInput->Analyze();
  if(turret->IsLimelightTracking())
  {
    limelight->Analyze();
  }
  xEndOtherAnalyze = frc::GetTime();
 // turret->TurretDialAdjust(userInput->GetFlightControllerAxis(6));
  xstartButtons = frc::GetTime();
  if(userInput->FlightCtrlBtnPushed(ENABLE_GYRO_BUTTON))
  {
    drive->GyroOn();
  }
  else
  {
    drive->GyroOff();
  }
  
  if(userInput->GamepadBtnPushed(CELL_GRAB_BUTTON))
  {
    collector->GrabCells();
    deathStar->IntakeCells();
  }
  else if(userInput->GamepadBtnPushed(CELL_EJECT_BUTTON))
  {
    collector->EjectCells();
    deathStar->IntakeCells();
  }
  else
  {
    collector->StopGrab();
  }

  if(userInput->GetGamepadButton(DEATH_STAR_CLOCKWISE_SPIN) == kPressing)
  {
    deathStar->Decreasepcmd();
  }
  if(userInput->GetGamepadButton(DEATH_STAR_COUNTER_SPIN) == kPressing)
  {
    deathStar->Increasepcmd();
  }

  if(userInput->GamepadBtnPushed(TRACKING_BUTTON))
  {
    turret->SetTargetingValues();
    shooter->SetTargeting();
    deathStar->PrepToShoot();
  }
  else
  {
    turret->StopTargeting();
    shooter->StopTargeting();
  }
  if(userInput->GamepadBtnPushed(SHOOTING_BUTTON))
  {
    deathStar->FireAtWill();
  }

  if(!userInput->GamepadBtnPushed(CELL_EJECT_BUTTON) && !userInput->GamepadBtnPushed(CELL_GRAB_BUTTON) && !userInput->GamepadBtnPushed(TRACKING_BUTTON) && !userInput->GamepadBtnPushed(SHOOTING_BUTTON))
  {
    deathStar->SetIdle();
  }
 
  if(userInput->GamepadBtnPushed(TURRET_FACE_SIDE_BUTTON))
  {
    turret->GiveGoalAngle(TURRET_SIDE_POS);
    shooter->GiveShroudGoalAngle(SHROUD_BEHIND_COLOR_WHEEL_ANGLE);
  }
  if(userInput->GamepadBtnPushed(TURRET_FACE_FRONT_BUTTON))
  {
    turret->GiveGoalAngle(TURRET_FACE_FRONT_POS);
    shooter->GiveShroudGoalAngle(SHROUD_BEHIND_COLOR_WHEEL_ANGLE);
  }
  if(userInput->GetDpadRightPushed())
  {
    shooter->GiveShroudGoalAngle(SHROUD_BAD_BALL_FAR_ANGLE);
  }
  if(userInput->GamepadBtnPushed(TURRET_FACE_BACK_BUTTON))
  {
    turret->GiveGoalAngle(TURRET_FACE_BACK_POS);
    shooter->GiveShroudGoalAngle(SHROUD_BEHIND_COLOR_WHEEL_ANGLE);
  }
  if(userInput->GetFlightControllerButton(COLOR_SPIN_BUTTON) == kPressing)
  {
    planner->PrintPath();
    //drive->DriveControl(0.25, 0.0, 0.0, 0.0, false, false, 0.0);
    //colorWheel->StartWheelRotate();
  }
  /*else
  {
    drive->DriveControl((userInput->GetFlightControllerAxis(SPEED_AXIS)), userInput->GetFlightControllerAxis(ROTATE_AXIS), addedDrive, addedRotate, false, false, 0.0);
  }
  
  if(userInput->GetFlightControllerButton(COLOR_SPIN_BUTTON) == kReleasing)
  {
    colorWheel->SetIdleMode();
  }*/
  if(userInput->GamepadBtnPushed(SHOOTER_SPIN_UP_BUTTON))
  {
    shooter->GiveShooterGoalVel(SHOOTER_COLOR_WHEEL_VEL);
    //shooter->GiveShooterGoalVel(200.0);
  }
  /*if(userInput->GetDpadUpPushed())
  {
    planner->BounceVelArray();
  }
  else if(userInput->GetDpadDownButton())
  {
    climb->MoveExtender(-1.0);
  }
  else
  {
    climb->MoveExtender(0.0);
  }
  if(userInput->GetDpadRightButton())
  {
    climb->GiveGoalPos(CLIMB_MID_DISTANCE);
  }
  if(userInput->GetDpadLeftButton())
  {
    climb->MoveWinch(0.7);
  }
  else
  {
    climb->MoveWinch(0.0);
  }
*/
 // limelight->TakeSnapshot(userInput->FlightCtrlBtnPushed(LIMELIGHT_SNAPSHOT_BUTTON));
  
  shooter->ManualShroudAdjust(-1.0*userInput->GetGamepadAxis(SHROUD_AXIS));
  //colorWheel->ManualWheelMove(userInput->GetGamepadAxis(COLOR_WHEEL_AXIS));
  xendButtons = frc::GetTime();

// shooter->TestingShooter(userInput->GetGamepadAxis(0));
 // shooter->TestingShroud(userInput->GetGamepadAxis(0));
 // turret->Testing(userInput->GetGamepadAxis(0));
  //deathStar->TestingDeathStar(userInput->GetGamepadAxis(2));
 // deathStar->TestingKickMotor(userInput->GetGamepadAxis(2));
 /*if(userInput->GamepadBtnPushed(1))
  {
    deathStar->TestExtendKicker();
  }
  else
  {
    deathStar->TestRetractKicker();
  }*/
/*  collector->TestingSpin(userInput->GetGamepadAxis(0));
  if(userInput->GamepadBtnPushed(1))
  {
    collector->GathererOut();
  }
  else
  {
    collector->GathererIn();
  } */
  //colorWheel->Testing(userInput->GetGamepadAxis(0));
  //climb->ExtendTesting(userInput->GetGamepadAxis(0));
  //climb->WinchTesting(userInput->GetGamepadAxis(2));

  xstartDrive = frc::GetTime();
  drive->DriveControl((userInput->GetFlightControllerAxis(SPEED_AXIS)), userInput->GetFlightControllerAxis(ROTATE_AXIS), addedDrive, addedRotate, false, false, 0.0);
  xendDrive = frc::GetTime();
  xstartService = frc::GetTime();
  CommunityService();
  xendService = frc::GetTime();
  xstartDrive = frc::GetTime();
  ServiceDash();
  xendDrive = frc::GetTime();
  xendPeriodicz = xendPeriodic;
  xendPeriodic = frc::GetTime();
  //printf("%d %5.1f %5.1f %5.1f %5.1f %5.1f %5.1f %5.1f %5.1f %5.1f \n", loopCount, (xendPeriodic-xendPeriodicz)*1000, (xendPeriodic-xstartPeriodic)*1000, (xendsurv-xstartsurv)*1000, (xEndOtherAnalyze-xstartOtherAnalyze)*1000, (xendButtons-xstartButtons)*1000, (xendDrive-xstartDrive)*1000, (xendService-xstartService)*1000, (xendDash-xstartDash)*1000);
  /*if((xendPeriodic - xendPeriodicz) > 1.1*LOOPTIME)
  {
    printf("FAIL END TO END: %d %5.1f %5.1f %5.1f %5.1f %5.1f %5.1f \n", loopCount, (xendPeriodic - xendPeriodicz)*1000, (xendPeriodic - xstartPeriodic)*1000, (xendsurv - xstartsurv)*1000, (xendcolor - xstartcolor)*1000, (xendcolor - xmidcolor)*1000, (xmidcolor - xstartcolor)*1000);
  }
  else if((xendPeriodic - xstartPeriodic) > 1.1*LOOPTIME)
  {
    printf("FAIL LOOP: %d %5.1f %5.1f %5.1f %5.1f %5.1f %5.1f \n", loopCount, (xendPeriodic - xendPeriodicz)*1000, (xendPeriodic - xstartPeriodic)*1000, (xendsurv - xstartsurv)*1000, (xendcolor - xstartcolor)*1000, (xendcolor - xmidcolor)*1000, (xmidcolor - xstartcolor)*1000);
  }
  else if((xendsurv - xstartsurv) > 1.1*LOOPTIME)
  {
    printf("FAIL SURV: %d %5.1f %5.1f %5.1f %5.1f %5.1f %5.1f \n", loopCount, (xendPeriodic - xendPeriodicz)*1000, (xendPeriodic - xstartPeriodic)*1000, (xendsurv - xstartsurv)*1000, (xendcolor - xstartcolor)*1000, (xendcolor - xmidcolor)*1000, (xmidcolor - xstartcolor)*1000);
  }
  else if((xendcolor - xstartcolor) > 1.1*LOOPTIME)
  {
    printf("FAIL COLOR: %d %5.1f %5.1f %5.1f %5.1f %5.1f %5.1f \n", loopCount, (xendPeriodic - xendPeriodicz)*1000, (xendPeriodic - xstartPeriodic)*1000, (xendsurv - xstartsurv)*1000, (xendcolor - xstartcolor)*1000, (xendcolor - xmidcolor)*1000, (xmidcolor - xstartcolor)*1000);
  }
  else if(loopCount%N1SEC == 0)
  {
    printf("ONE SEC: %d %5.1f %5.1f %5.1f %5.1f %5.1f %5.1f \n", loopCount, (xendPeriodic - xendPeriodicz)*1000, (xendPeriodic - xstartPeriodic)*1000, (xendsurv - xstartsurv)*1000, (xendcolor - xstartcolor)*1000, (xendcolor - xmidcolor)*1000, (xmidcolor - xstartcolor)*1000);
  }*/
}

void TalonXXI::ServiceDash()
{
  if(dashCounter == 30)
  {
    dashCounter = 0;
    userInput->UpdateDash();
  //#ifndef TEST_DRIVE
    surveillance->UpdateDash();
    drive->UpdateDash();
    shooter->UpdateDash();
    turret->UpdateDash();
    limelight->UpdateDash();
    deathStar->UpdateDash();
    collector->UpdateDash();
    planner->UpdateDash();
    //colorWheel->UpdateDash();
  //#endif
    //climb->UpdateDash();
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
  //lightStrip->Service();
  collector->Service();
  //colorWheel->Service();
  //climb->Service();
  planner->Service();
}

void TalonXXI::Omnicide()
{
  surveillance->StopAll();
  userInput->StopAll();
  shooter->StopAll();
  turret->StopAll();
  limelight->StopAll();
  deathStar->StopAll();
  collector->StopAll();
  //colorWheel->StopAll();
  drive->StopAll();
  //climb->StopAll();
  planner->StopAll();
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
  shooter->StartingConfig();
  turret->StartingConfig();
  limelight->StartingConfig();
  deathStar->StartingConfig();
  collector->StartingConfig();
  //colorWheel->StartingConfig();
  drive->StartingConfig();
  //climb->StartingConfig();
  planner->StartingConfig();
}


void TalonXXI::TestPeriodic() 
{
  limelight->AutoHorizontalOffset(-1);
  surveillance->Analyze();
  userInput->Analyze();
  limelight->Analyze();
  
 // shooter->TestingShooter(userInput->GetGamepadAxis(0));
 // shooter->TestingShroud(userInput->GetGamepadAxis(0));
 // turret->Testing(userInput->GetGamepadAxis(0));
 // deathStar->TestingDeathStar(userInput->GetGamepadAxis(0));
 // deathStar->TestingKickMotor(userInput->GetGamepadAxis(0));
 /* if(userInput->GamepadBtnPushed(1))
  {
    deathStar->TestExtendKicker();
  }
  else
  {
    deathStar->TestRetractKicker();
  }*/
 // collector->TestingSpin(userInput->GetGamepadAxis(0));
  /*if(userInput->GamepadBtnPushed(1))
  {
    collector->GathererOut();
  }
  else
  {
    collector->GathererIn();
  } */
 // colorWheel->Testing(userInput->GetGamepadAxis(0));
 // climb->ExtendTesting(userInput->GetGamepadAxis(0));
 // climb->WinchTesting(userInput->GetGamepadAxis(2));

  drive->DriveControl((userInput->GetFlightControllerAxis(SPEED_AXIS)), userInput->GetFlightControllerAxis(ROTATE_AXIS), addedDrive, addedRotate, false, false, 0.0);
  CommunityService();
  ServiceDash();
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<TalonXXI>(); }
#endif
