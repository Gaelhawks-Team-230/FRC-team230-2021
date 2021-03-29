/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/Joystick.h>
#include <frc/PowerDistributionPanel.h>
#include <frc/DriverStation.h>
#include "ctre/Phoenix.h"
#include <frc/PWMVictorSPX.h>
#include <frc/VictorSP.h>
#include "frc/smartdashboard/Smartdashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "Autonomous.h"
#include "BallShooter.h"
#include "ShooterTurret.h"
#include "CellCollector.h"
//#include "Climber.h"
#include "LimeLight.h"
#include "SensorState.h"
#include "JoystickState.h"
#include "ShooterTurret.h"
#include "Drivetrain.h"
#include "DeathStar.h"
#include "LimeLight.h"
//#include "TrajectoryPlanner.h"
//#include "TrajectoryPlannerYaw.h"
// #include "WheelOfFortune.h"
#include "VisionPi.h"
#inculde "SkillsChallenge.h"

#include "ctre/phoenix/motorcontrol/TalonFXSensorCollection.h"
#include "ctre/phoenix/motorcontrol/StatorCurrentLimitConfiguration.h"

class TalonXXI : public frc::TimedRobot
{
public:
  TalonXXI();
  frc::PowerDistributionPanel *pdp;
  SensorState *surveillance;
  JoystickState *userInput;
  //#ifdef TEST_DRIVE
  Drivetrain *drive;
  //#endif
  //#else
  BallShooter *shooter;
  ShooterTurret *turret;
  LimelightCamera *limelight;
  CellCollector *collector;
  //WheelOfFortune *colorWheel;
  //Climber *climb;
  //#endif
  DeathStar *deathStar;
  VisionPi *camera;
  //LightController *lightStrip;
  TrajectoryPlanner *pathPlanner;
  TrajectoryPlannerYaw *anglePlanner;

  double addedDrive;
  double addedRotate;
  bool isAuto;

  double xstartPeriodic;
  double xendPeriodic;
  double xendPeriodicz;
  double xstartsurv;
  double xendsurv;
  double xstartcolor;
  double xmidcolor;
  double xendcolor;

  double xstartOtherAnalyze;
  double xEndOtherAnalyze;
  double xstartButtons;
  double xendButtons;
  double xstartDrive;
  double xendDrive;
  double xstartService;
  double xendService;
  double xstartDash;
  double xendDash;

  void RobotInit() override;
  void InitializeAlliance();
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void ServiceDash(void);
  void CommunityService(void);
  void Omnicide(void);
  static double Limit(double min, double max, double curValue);
  static double Sign(double);
  void RobotStartingConfig(void);
  void TestPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;

  void DoNothing(void);
  void InitiationLine(void);
  void PushPartner(void);
  void CenterShootFirst(void);
  void TrenchShootFirst(void);
  void FeederShootFirst(void);
  void CenterShootSecond(void);
  void TrenchShootSecond(void);
  void FeederShootSecond(void);
  void ModeSelection(bool);

private:
  frc::SendableChooser<int> *AutoPositionChooser;
  frc::SendableChooser<int> *AutoBallNumber;
  int autoStage;
  int autoMode;
  int autoModeSecond;
  int autoStartPosition;
  int autoBallNumber;
  double delayTime;
  int delayCount;
  bool isDelay;

  int DO_NOTHING = 0;
  int BASELINE = 1;
  int FEEDER_POS = 2;
  int CENTER_POS = 3;
  int TRENCH_POS = 4;
  int PUSH_PARTNER = 7;

  int LEVEL_ONE = 5;
  int LEVEL_TWO = 6;

  int dashCounter;
  bool isBlueAlliance;
  bool firstTime;
  double driveCmd;
  double rotateCmd;
  double isGyroOn;
  bool isInTeleop;
  int loopCount;

  bool isJoystickCountInitialized;
};
