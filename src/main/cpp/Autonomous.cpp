#include "Common.h"
#include "Autonomous.h"
#include "TalonXXI_main.h"

void TalonXXI::DoNothing()
{
    driveCmd = 0.0; rotateCmd = 0.0;
    loopCount = 0;
    autoStage = 0;
}

void TalonXXI::InitiationLine()
{
    loopCount++;
    switch(autoStage)
    {
        case 0:
            driveCmd = 0.0; rotateCmd = 0.0;
            //drive->LocalReset();
            surveillance->ResetDriveEncoders();
            loopCount = 0;
            autoStage++;
            break;

        case 1:
            if(fabs(surveillance->GetAverageDriveDis()) < DISTANCE_INITIATION_LINE)
            {
                driveCmd = driveCmd + Limit(-MAX_AUTO_ACCELERATION, MAX_AUTO_ACCELERATION, -0.3 - driveCmd);
                rotateCmd = 0.0;
                
            }
            else
            {
                driveCmd = 0.0; rotateCmd = 0.0;
                loopCount = 0;
                autoStage++;
            }
            break;

        case 2:
            driveCmd = 0.0; rotateCmd = 0.0;
            surveillance->ResetDriveEncoders();
            loopCount = 0;
            autoStage = 0;
            break;

    }
}

void TalonXXI::PushPartner()
{
    loopCount++;
    switch(autoStage)
    {

    }
}

void TalonXXI::CenterShootFirst()
{
    loopCount++;
    switch(autoStage)
    {
        case 0:
            driveCmd = 0.0; rotateCmd = 0.0;
            surveillance->ResetDriveEncoders();
            if(!shooter->IsShroudCalibrating())
            {
                loopCount = 0;
                autoStage++;
            }
            break;

        case 1:
            driveCmd = 0.0; rotateCmd = 0.0;
            shooter->GiveShooterGoalVel(SHOOTER_INITIATION_LINE_VEL);
            shooter->GiveShroudGoalAngle(SHROUD_INITIATION_LINE_ANGLE);
            turret->GiveGoalAngle(TURRET_CENTER_AUTO_POS);
            deathStar->PrepToShoot();
            autoStage++;
            break;
            
        case 2:
            if(turret->IsInTrackingRange())
            {
                turret->SetTargetingValues();
                autoStage++;
                loopCount = 0;
            }
            break;

        case 3:
            driveCmd = 0.0; rotateCmd = 0.0;
            if(turret->IsReadyToShoot())
            {
                deathStar->FireAtWill();
                autoStage++;
                loopCount = 0;
            }
            break;

        case 4:
            if(deathStar->IsEmpty())
            {
                autoStage++;
                loopCount = 0;
                surveillance->ResetDriveEncoders();
                shooter->GiveShooterGoalVel(0.0);
                turret->StopTargeting();
            }
            break;
            
        case 5:
            if(fabs(surveillance->GetAverageDriveDis()) < DISTANCE_INITIATION_LINE)
            {
                driveCmd = driveCmd + Limit(-MAX_AUTO_ACCELERATION, MAX_AUTO_ACCELERATION, -0.3 - driveCmd);
                rotateCmd = 0.0;
            }
            else
            {
                driveCmd = 0.0; rotateCmd = 0.0;
                loopCount = 0;
                autoStage++;
            }
            break;

        case 6:
            driveCmd = 0.0; rotateCmd = 0.0;
            autoMode = autoModeSecond;
            autoStage = 0;
            loopCount = 0;
            break;
    }
}

void TalonXXI::TrenchShootFirst()
{
    loopCount++;
    switch(autoStage)
    {
        case 0:
            driveCmd = 0.0; rotateCmd = 0.0;
            surveillance->ResetDriveEncoders();
            if(!shooter->IsShroudCalibrating())
            {
                loopCount = 0;
                shooter->GiveShooterGoalVel(SHOOTER_COLOR_WHEEL_VEL);
                shooter->GiveShroudGoalAngle(SHROUD_BEHIND_COLOR_WHEEL_ANGLE);
                turret->GiveGoalAngle(TURRET_TRENCH_FRONT_POS);
                deathStar->IntakeCells();
                collector->GrabCells();
                autoStage++;
            }
            break;

        case 1:
            if(fabs(surveillance->GetAverageDriveDis()) < DISTANCE_TO_FRONT_TRENCH)
            {
                if(deathStar->GetCellCount() == 4)
                {
                    driveCmd = driveCmd + Limit(-MAX_AUTO_ACCELERATION, MAX_AUTO_ACCELERATION, -0.2 - driveCmd);
                }
                else
                {
                    driveCmd = driveCmd + Limit(-MAX_AUTO_ACCELERATION, MAX_AUTO_ACCELERATION, -0.17 - driveCmd);
                }
                rotateCmd = 0.0;
            }
            else
            {
                driveCmd = 0.0; rotateCmd = 0.0;
                loopCount = 0;
                autoStage++;
                deathStar->PrepToShoot();
            }
            break;
            
        case 2:
            if(turret->IsInTrackingRange())
            {
                turret->SetTargetingValues();
                autoStage++;
                loopCount = 0;
            }
            break;

        case 3:
            driveCmd = 0.0; rotateCmd = 0.0;
            if(turret->IsReadyToShoot())
            {
                deathStar->FireAtWill();
                autoStage++;
                loopCount = 0;
            }
            break;

        case 4:
            if(deathStar->IsEmpty())
            {
                autoStage++;
                loopCount = 0;
                surveillance->ResetDriveEncoders();
                collector->GrabCells();
                deathStar->IntakeCells();
            }
            break;

        case 5:
            if(fabs(surveillance->GetAverageDriveDis()) < DISTANCE_FOR_LAST_BALL)
            {
                driveCmd = driveCmd + Limit(-MAX_AUTO_ACCELERATION, MAX_AUTO_ACCELERATION, -0.2 - driveCmd);
                rotateCmd = 0.0;
            }
            else
            {
                driveCmd = 0.0; rotateCmd = 0.0;
                loopCount = 0;
                autoStage++;
                collector->StopGrab();
                deathStar->PrepToShoot();
            }
            break;

        case 6:
            driveCmd = 0.0; rotateCmd = 0.0;
            autoMode = autoModeSecond;
            autoStage = 0;
            loopCount = 0;
            break;
    }
}

void TalonXXI::FeederShootFirst()
{
   loopCount++;
    switch(autoStage)
    {
        case 0:
            driveCmd = 0.0; rotateCmd = 0.0;
            surveillance->ResetDriveEncoders();
            if(!shooter->IsShroudCalibrating())
            {
                loopCount = 0;
                autoStage++;
            }
            break;

        case 1:
            driveCmd = 0.0; rotateCmd = 0.0;
            shooter->GiveShooterGoalVel(SHOOTER_INITIATION_LINE_VEL);
            shooter->GiveShroudGoalAngle(SHROUD_INITIATION_LINE_ANGLE);
            turret->GiveGoalAngle(TURRET_FEEDER_AUTO_POS);
            deathStar->PrepToShoot();
            autoStage++;
            break;
            
        case 2:
            if(turret->IsInTrackingRange())
            {
                turret->SetTargetingValues();
                autoStage++;
                loopCount = 0;
            }
            break;

        case 3:
            driveCmd = 0.0; rotateCmd = 0.0;
            if(turret->IsReadyToShoot())
            {
                deathStar->FireAtWill();
                autoStage++;
                loopCount = 0;
            }
            break;

        case 4:
            if(deathStar->IsEmpty())
            {
                autoStage++;
                loopCount = 0;
                surveillance->ResetDriveEncoders();
                shooter->GiveShooterGoalVel(0.0);
                turret->StopTargeting();
            }
            break;
            
        case 5:
            if(fabs(surveillance->GetAverageDriveDis()) < DISTANCE_INITIATION_LINE)
            {
                driveCmd = driveCmd + Limit(-MAX_AUTO_ACCELERATION, MAX_AUTO_ACCELERATION, -0.3 - driveCmd);
                rotateCmd = 0.0;
            }
            else
            {
                driveCmd = 0.0; rotateCmd = 0.0;
                loopCount = 0;
                autoStage++;
            }
            break;

        case 6:
            driveCmd = 0.0; rotateCmd = 0.0;
            autoMode = autoModeSecond;
            autoStage = 0;
            loopCount = 0;
            break;
    }
}

void TalonXXI::CenterShootSecond()
{
    loopCount++;
    switch(autoStage)
    {
        case 0:
            driveCmd = 0.0; rotateCmd = 0.0;
            surveillance->ResetDriveEncoders();
            loopCount = 0;
            autoStage++;
            break;

        case 1:
            if(loopCount < TIME_TURNING_TO_WALL)
            {
                driveCmd = -0.1; rotateCmd = -100.0;
            }
            else
            {
                driveCmd = 0.0; rotateCmd = 0.0;
                surveillance->ResetDriveEncoders();
                turret->GiveGoalAngle(TURRET_TRENCH_FRONT_POS);
                shooter->GiveShooterGoalVel(SHOOTER_COLOR_WHEEL_VEL);
                shooter->GiveShroudGoalAngle(SHROUD_BEHIND_COLOR_WHEEL_ANGLE);
                loopCount = 0;
                autoStage++;
            }
            break;

        case 2:
            if(fabs(surveillance->GetAverageDriveDis()) < DISTANCE_CENTER_TO_TRENCH)
            {
                driveCmd = driveCmd + Limit(-MAX_AUTO_ACCELERATION, MAX_AUTO_ACCELERATION, -0.3 - driveCmd);
                rotateCmd = 0.0;
            }
            else
            {
                driveCmd = 0.0; rotateCmd = 0.0;
                collector->GrabCells();
                deathStar->IntakeCells();
                loopCount = 0;
                autoStage++;
            }
            break;

        case 3:
            if(loopCount < TIME_TURNING_TO_TRENCH)
            {
                driveCmd = -0.1; rotateCmd = 100.0;
            }
            else
            {
                driveCmd = 0.0; rotateCmd = 0.0;
                surveillance->ResetDriveEncoders();
                turret->SetTargetingValues();
                shooter->SetTargeting();
                loopCount = 0;
                autoStage++;
            }
            break;

        case 4:
            if(fabs(surveillance->GetAverageDriveDis()) < DISTANCE_GRAB_BALLS_IN_TRENCH)
            {
                driveCmd = driveCmd + Limit(-MAX_AUTO_ACCELERATION, MAX_AUTO_ACCELERATION, -0.2 - driveCmd);
                rotateCmd = 0.0;
            }
            else
            {
                driveCmd = 0.0; rotateCmd = 0.0;
                loopCount = 0;
                autoStage++;
                collector->StopGrab();
                deathStar->PrepToShoot();
            }
            break;
            
        case 5:
            driveCmd = 0.0; rotateCmd = 0.0;
            if(turret->IsReadyToShoot())
            {
                deathStar->FireAtWill();
                autoStage++;
                loopCount = 0;
            }
            break;

        case 6:
            if(deathStar->IsEmpty())
            {
                autoStage++;
                loopCount = 0;
                surveillance->ResetDriveEncoders();
                shooter->GiveShooterGoalVel(0.0);
                turret->StopTargeting();
                shooter->StopTargeting();
            }
            break;

        case 7:
            driveCmd = 0.0; rotateCmd = 0.0;
            autoMode = 0;
            autoStage = 0;
            loopCount = 0;
            break;
        
    }
}

void TalonXXI::TrenchShootSecond()
{
    switch(autoStage)
    {
        
    }
}

void TalonXXI::FeederShootSecond()
{
    switch(autoStage)
    {
        
    }
}

void TalonXXI::TestSkillsChal()
{
    switch(autoStage)
    {
        loopCount++;
        case 0:
            driveCmd = 0.0; rotateCmd = 0.0;
            //drive->LocalReset();
            surveillance->ResetDriveEncoders();
            loopCount = 0;
            trajIndex = 0;
            autoStage++;
            isTraj = false;
            break;
        case 1:
            isTraj = true;
            if(!planner->IsPathComplete(trajIndex))
            {
                std::vector<double> cmds = planner->GetCurrentCmd(trajIndex);
                velCmd = cmds[0];
                rotateCmd = cmds[1]*180/PI;
                trajIndex++;
                printf("VelCmd: %f\n", velCmd);
            }
            else
            {
                trajIndex = 0;
                loopCount = 0;
                autoStage++;
            }
            
            /*if (loopCount<TIME_TEST_AUTO)
            {
                velCmd = 5.0;
            }
            else
            {
                velCmd = 0.0;
                loopCount = 0.0;
                autoStage++;
            }*/
            break;
        case 2:
            isTraj = false;
            driveCmd = 0.0; rotateCmd = 0.0;
            autoMode = 0;
            autoStage = 0;
            loopCount = 0;
            trajIndex = 0;
            break;
    }
}

void TalonXXI::ModeSelection(bool forcePrint)
{
    modeChange = false;
    const char* startPos;
    const char* autoLevel;
    
    int tempStartPos = autoStartPosition;
    int tempBallNumber = autoBallNumber;
    double tempDelayTime = delayTime;
    
    delayTime = frc::SmartDashboard::GetNumber("Delay time", 0.0);
    delayCount = (int)(delayTime * N1SEC);
    autoStartPosition = (AutoPositionChooser->GetSelected());
    autoBallNumber = (AutoBallNumber->GetSelected());
   // printf("%d, %d, %f \n", autoStartPosition, autoBallNumber, delayTime);

    if(autoStartPosition == DO_NOTHING)
    {
        autoMode = 0;
        startPos = "DO NOTHING";
    }
    else if(autoStartPosition == BASELINE)
    {
        autoMode = 1;
        startPos = "BASELINE";
    }
    else if(autoStartPosition == FEEDER_POS)
    {
        autoMode = 2;
        startPos = "FEEDER";
    }
    else if(autoStartPosition == CENTER_POS)
    {
        autoMode = 3;
        startPos = "CENTER";
    }
    else if(autoStartPosition == TRAJ_PLANNER)
    {
        autoMode = 8;
        startPos = "SKILL";
        /*if (autoBallNumber == BARREL_TRAJ)
        {

        }
        else if (autoBallNumber == SLALOM_TRAJ)
        {

        }
        else if (autoBallNumber == BOUNCE_TRAJ)
        {
            
        }
        else
        {
            
        }*/
        

    }
    else if(autoStartPosition == GALACTIC_SEARCH)
    {
        autoMode = 9;
        startPos = "GALACTIC SEARCH";
    }
    else
    {
        autoMode = 4;
        startPos = "TRENCH";
    }
    
    if(autoBallNumber == LEVEL_TWO)
    {
        autoLevel = "LEVEL TWO";
        if(autoStartPosition == FEEDER_POS)
        {
            autoModeSecond = 5;
        }
        else if(autoStartPosition == CENTER_POS)
        {
            autoModeSecond = 6;
        }
        else
        {
            autoModeSecond = 7;
        }
    }
    else
    {
        autoLevel = "LEVEL ONE";
        autoModeSecond = 0;
    }

    if((autoBallNumber != tempBallNumber) || (autoStartPosition != tempStartPos) || (delayTime - tempDelayTime)>0.01)
    {
        modeChange = true;
    }
    if(forcePrint)
    {
        modeChange = true;
    }
    if(modeChange)
    {
        //printf("Mode Selection \n");
        //printf("Delay: %f\n", delayTime);
        //printf("Start: %s \n", startPos);
        //printf("Balls: %s \n", autoLevel);
        //printf("\n");
    }
}