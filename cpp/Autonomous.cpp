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
            if(surveillance->GetAverageDriveDis() < DISTANCE_INITIATION_LINE)
            {
                driveCmd = driveCmd + Limit(MIN_AUTO_ACCELERATION, MAX_AUTO_ACCELERATION, -0.5 - driveCmd);
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

void TalonXXI::CenterShootFirst()
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
            driveCmd = 0.0; rotateCmd = 0.0;
            shooter->GiveShooterGoalVel(SHOOTER_AUTO_VEL);
            shooter->GiveShroudGoalAngle(SHROUD_AUTO_ANGLE);
            turret->GiveGoalAngle(TURRET_CENTER_AUTO_POS);
            deathStar->PrepToShoot();
            if(turret->IsInTrackingRange())
            {
                turret->SetTargetingValues();
            }
            loopCount = 0;
            autoStage++;
            break;

        case 2:
            driveCmd = 0.0; rotateCmd = 0.0;
            if(shooter->IsShooterAtGoal() && shooter->IsShroudAtGoal() && turret->IsReadyToShoot())
            {
                autoStage++;
            }
            loopCount = 0;
            break;

        case 3:
            driveCmd = 0.0; rotateCmd = 0.0;
            deathStar->FireAtWill();
            if(deathStar->IsEmpty())
            {
                autoStage++;
                loopCount = 0;
                surveillance->ResetDriveEncoders();
                shooter->GiveShooterGoalVel(0.0);
                turret->StopTargeting();
            }
            break;
            
        case 4:
            if(surveillance->GetAverageDriveDis() < DISTANCE_INITIATION_LINE)
            {
                driveCmd = driveCmd + Limit(MIN_AUTO_ACCELERATION, MAX_AUTO_ACCELERATION, -0.5 - driveCmd);
                rotateCmd = 0.0;
                
            }
            else
            {
                driveCmd = 0.0; rotateCmd = 0.0;
                loopCount = 0;
                autoStage++;
            }
            break;

        case 5:
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
            loopCount = 0;
            autoStage++;
            break;

        case 1:
            driveCmd = 0.0; rotateCmd = 0.0;
            shooter->GiveShooterGoalVel(SHOOTER_AUTO_VEL);
            shooter->GiveShroudGoalAngle(SHROUD_AUTO_ANGLE);
            turret->GiveGoalAngle(TURRET_TRENCH_AUTO_POS);
            deathStar->PrepToShoot();
            if(turret->IsInTrackingRange())
            {
                turret->SetTargetingValues();
            }
            loopCount = 0;
            autoStage++;
            break;

        case 2:
            driveCmd = 0.0; rotateCmd = 0.0;
            if(shooter->IsShooterAtGoal() && shooter->IsShroudAtGoal() && turret->IsReadyToShoot())
            {
                autoStage++;
            }
            loopCount = 0;
            break;

        case 3:
            driveCmd = 0.0; rotateCmd = 0.0;
            deathStar->FireAtWill();
            if(deathStar->IsEmpty())
            {
                autoStage++;
                loopCount = 0;
                surveillance->ResetDriveEncoders();
                shooter->GiveShooterGoalVel(0.0);
                turret->StopTargeting();
            }
            break;
            
        case 4:
            if(surveillance->GetAverageDriveDis() < DISTANCE_INITIATION_LINE)
            {
                driveCmd = driveCmd + Limit(MIN_AUTO_ACCELERATION, MAX_AUTO_ACCELERATION, -0.5 - driveCmd);
                rotateCmd = 0.0;
                
            }
            else
            {
                driveCmd = 0.0; rotateCmd = 0.0;
                loopCount = 0;
                autoStage++;
            }
            break;

        case 5:
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
            loopCount = 0;
            autoStage++;
            break;

        case 1:
            driveCmd = 0.0; rotateCmd = 0.0;
            shooter->GiveShooterGoalVel(SHOOTER_AUTO_VEL);
            shooter->GiveShroudGoalAngle(SHROUD_AUTO_ANGLE);
            turret->GiveGoalAngle(TURRET_FEEDER_AUTO_POS);
            deathStar->PrepToShoot();
            if(turret->IsInTrackingRange())
            {
                turret->SetTargetingValues();
            }
            loopCount = 0;
            autoStage++;
            break;

        case 2:
            driveCmd = 0.0; rotateCmd = 0.0;
            if(shooter->IsShooterAtGoal() && shooter->IsShroudAtGoal() && turret->IsReadyToShoot())
            {
                autoStage++;
            }
            loopCount = 0;
            break;

        case 3:
            driveCmd = 0.0; rotateCmd = 0.0;
            deathStar->FireAtWill();
            if(deathStar->IsEmpty())
            {
                autoStage++;
                loopCount = 0;
                surveillance->ResetDriveEncoders();
                shooter->GiveShooterGoalVel(0.0);
                turret->StopTargeting();
            }
            break;
            
        case 4:
            if(surveillance->GetAverageDriveDis() < DISTANCE_INITIATION_LINE)
            {
                driveCmd = driveCmd + Limit(MIN_AUTO_ACCELERATION, MAX_AUTO_ACCELERATION, -0.5 - driveCmd);
                rotateCmd = 0.0;
                
            }
            else
            {
                driveCmd = 0.0; rotateCmd = 0.0;
                loopCount = 0;
                autoStage++;
            }
            break;

        case 5:
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
            if(loopCount < HALF_SEC)
            {
                driveCmd = 0.0; rotateCmd = -180.0;
            }
            else
            {
                driveCmd = 0.0; rotateCmd = 0.0;
                surveillance->ResetDriveEncoders();
                loopCount = 0;
                autoStage++;
            }
            break;

        case 2:
            if(surveillance->GetAverageDriveDis() < DISTANCE_CENTER_TO_TRENCH)
            {
                driveCmd = driveCmd + Limit(MIN_AUTO_ACCELERATION, MAX_AUTO_ACCELERATION, -0.5 - driveCmd);
                rotateCmd = 0.0;
                
            }
            else
            {
                driveCmd = 0.0; rotateCmd = 0.0;
                loopCount = 0;
                autoStage++;
            }
            break;

        case 3:
            if(loopCount < HALF_SEC)
            {
                driveCmd = 0.0; rotateCmd = 180.0;
            }
            else
            {
                driveCmd = 0.0; rotateCmd = 0.0;
                surveillance->ResetDriveEncoders();
                //collector->GrabCells();
                deathStar->IntakeCells();
                loopCount = 0;
                autoStage++;
            }
            break;

        case 4:
            turret->GiveGoalAngle(TURRET_TRENCH_FRONT_POS);
        
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

void TalonXXI::BarrelRacingPath()
{
    loopCount++;
    switch(autoStage)
    {

    }
}

void TalonXXI::SlalomPath()
{
    loopCount++;
    switch(autoStage)
    {

    }
}

void TalonXXI::BouncePath()
{
    loopCount++;
    switch(autoStage)
    {

    }
}




void TalonXXI::ModeSelection()
{
    bool modeChange = false;
    const char* startPos;
    const char* secondBall;
    
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
    else if(autoStartPosition == BARREL_PATH)
    {
        autoMode = 8;
        startPos = "BARREL PATH";
    }
    else if(autoStartPosition == SLALOM_PATH)
    {
        autoMode = 9;
        startPos = "Slalom Path";
    }
    else if(autoStartPosition == BOUNCE_PATH)
    {
        autoMode = 10;
        startpos = "Bounce Path";
    }
    else
    {
        autoMode = 4;
        startPos = "TRENCH";
    }
    
    if(autoBallNumber == TWO_BALL)
    {
        secondBall = "TWO BALLS";
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
        secondBall = "ONE BALL";
        autoModeSecond = 0;
    }

    if((autoBallNumber != tempBallNumber) || (autoStartPosition != tempStartPos) || (delayTime - tempDelayTime)>0.01)
    {
        modeChange = true;
    }
    if(modeChange)
    {
        printf("Mode Selection \n");
        printf("Delay: %f\n", delayTime);
        printf("Start: %s \n", startPos);
        printf("Balls: %s \n", secondBall);
        printf("\n");
    }
}