#include "Common.h"
#include "SkillsChallenge.h"
#include "TalonXXI_main.h"
#include "Autonomous.h"


void TalonXXI::GalacticSearchRedA()
{
    loopCount++;
    //printf("stage: %d\n", autoStage);
    switch (autoStage)
    {
        case 0:
            driveCmd = 0.0; rotateCmd = 0.0;
            surveillance->ResetDriveEncoders();
            loopCount = 0;
            printf("GalacticSearchRedA\n");
            camera->SetCurrentTarget(0);
            autoStage++;
            break;

        case 1:
            driveCmd = 0.0; rotateCmd = 0.0;
            loopCount = 0;
            dist = camera->GetCurrDistanceMarker();
            //printf("Distance to powercell: %f", dist);
            collector->GrabCells();
            deathStar->IntakeCells();
            autoStage++;
            break;

        case 2:
            //printf("drive distance: %f\n", surveillance->GetAverageDriveDis());
            if(fabs(surveillance->GetAverageDriveDis()) < dist)
            {
                driveCmd = driveCmd + Limit(-MAX_AUTO_ACCELERATION, MAX_AUTO_ACCELERATION, -0.25 - driveCmd); 
                rotateCmd = 0.0;
                
            }
            else
            {
                driveCmd = 0.0; rotateCmd = 0.0;
                autoStage++;
            }
            break;

        case 3:
            if(deathStar->GetCellCount() == 1)
            {
                dist = camera->GetCurrDistanceMarker();
                surveillance->ResetDriveEncoders();
                autoStage+=2;
            }
            else
            {
                driveCmd = driveCmd + Limit(-MAX_AUTO_ACCELERATION, MAX_AUTO_ACCELERATION, -0.25 - driveCmd); 
                rotateCmd = 0.0;
            }
            break;

        // case 4:
        //     if(fabs(surveillance->GetAverageDriveDis()) < dist)
        //     {
        //         driveCmd = driveCmd + Limit(-MAX_AUTO_ACCELERATION, MAX_AUTO_ACCELERATION, -0.5 - driveCmd); 
        //         rotateCmd = 0.0;
        //     }
        //     else
        //     {
        //         autoStage++;
        //     }
        //     break;

        case 5:
            if(deathStar->GetCellCount() == 2)
            {
                dist = camera->GetCurrDistanceMarker();
                surveillance->ResetDriveEncoders();
                driveCmd = 0.0; rotateCmd = 0.0;
                autoStage++;
                loopCount = 0;
            }
            else
            {
                driveCmd = driveCmd + Limit(-MAX_AUTO_ACCELERATION, MAX_AUTO_ACCELERATION, -0.25 - driveCmd); 
                rotateCmd = 0.0;
            }
            break;

        case 6:
            if(loopCount < TIME_TURNING_TO_A6_REDA)
            {
                driveCmd = -0.1; rotateCmd = -100.0;
            }
            else
            {
                driveCmd = 0.0; rotateCmd = 0.0;
                dist = camera->GetCurrDistanceMarker();
                surveillance->ResetDriveEncoders();
                loopCount = 0;
                autoStage++;
            }
            break;

        case 7:
            if (loopCount < PI_LOAD_PAUSE)
            {
                loopCount++;
            }
            else
            {
                driveCmd = 0.0; rotateCmd = 0.0;
                camera->SetCurrentTarget(0);
                dist = camera->GetCurrDistanceMarker();
                surveillance->ResetDriveEncoders();
                loopCount = 0;
                autoStage++;
            }
            break;
            

        case 8:
            if(fabs(surveillance->GetAverageDriveDis()) < dist)
            {
                driveCmd = driveCmd + Limit(-MAX_AUTO_ACCELERATION, MAX_AUTO_ACCELERATION, -0.25 - driveCmd); 
                rotateCmd = 0.0;
            }
            else
            {
                autoStage++;
            }
            break;

        case 9:
            if(deathStar->GetCellCount() == 3)
            {
                autoStage++;
                loopCount = 0;                
            }
            else
            {
                driveCmd = driveCmd + Limit(-MAX_AUTO_ACCELERATION, MAX_AUTO_ACCELERATION, -0.25 - driveCmd); 
                rotateCmd = 0.0;
                collector->StopGrab();
            }
            break;

        case 10:
            if(loopCount < TIME_TURNING_TO_ENDZONE_REDA)
            {
                driveCmd = -0.1; rotateCmd = 100.0;
            }
            else
            {
                driveCmd = 0.0; rotateCmd = 0.0;
                dist = camera->GetCurrDistanceMarker();
                surveillance->ResetDriveEncoders();
                loopCount = 0;
                autoStage++;
            }
            break;

        case 11:
            if(surveillance->GetAverageDriveDis() < DISTANCE_TO_ENDZONE_REDA)
            {
                driveCmd = driveCmd + Limit(-MAX_AUTO_ACCELERATION, MAX_AUTO_ACCELERATION, -0.5 - driveCmd); 
                rotateCmd = 0.0;
            }
            else
            {
                driveCmd = 0.0; rotateCmd = 0.0;
                loopCount = 0;
                autoStage++;
            }
            break;

        case 12:
            driveCmd = 0.0; rotateCmd = 0.0;
            autoMode = autoModeSecond;
            autoStage = 0;
            loopCount = 0;
            break;

        default:
            break;

    }
}




void TalonXXI::GalacticSearchRedB()
{
    loopCount++;
    switch (autoStage)
    {
        case 0:
            driveCmd = 0.0; rotateCmd = 0.0;
            surveillance->ResetDriveEncoders();
            printf("GalacticSearchRedB\n");
            loopCount = 0;
            autoStage++;
            break;

        case 1:
            driveCmd = 0.0; rotateCmd = 0.0;
            loopCount = 0;
            autoStage++;
            camera->SetCurrentTarget(0);
            dist = camera->GetCurrDistanceMarker();
            collector->GrabCells();
            deathStar->IntakeCells();
            break;

        case 2:
            driveCmd = 0.0; rotateCmd = 0.0;
            loopCount = 0;
            autoStage++;            
            surveillance->ResetDriveEncoders();
            break;

        case 3:
            if(fabs(surveillance->GetAverageDriveDis()) < dist)
            {
                driveCmd = driveCmd + Limit(-MAX_AUTO_ACCELERATION, MAX_AUTO_ACCELERATION, -0.5 - driveCmd); 
                rotateCmd = 0.0;
                
            }
            else
            {
                autoStage++;
            }
            break;

        case 4:
            if(deathStar->GetCellCount() == 1)
            {
                dist = camera->GetCurrDistanceMarker();                
                autoStage++;
            }
            else
            {
                driveCmd = driveCmd + Limit(-MAX_AUTO_ACCELERATION, MAX_AUTO_ACCELERATION, -0.5 - driveCmd); 
                rotateCmd = 0.0;
            }
            break;

        case 5:
            if(fabs(surveillance->GetAverageDriveDis()) < dist)
            {
                driveCmd = driveCmd + Limit(-MAX_AUTO_ACCELERATION, MAX_AUTO_ACCELERATION, -0.5 - driveCmd); 
                rotateCmd = 0.0;
            }
            else
            {
                autoStage++;
            }
            break;

        case 6:
            if(deathStar->GetCellCount() == 2)
            {
                dist = camera->GetCurrDistanceMarker();
                surveillance->ResetDriveEncoders();
                autoStage++;
            }
            else
            {
                driveCmd = driveCmd + Limit(-MAX_AUTO_ACCELERATION, MAX_AUTO_ACCELERATION, -0.5 - driveCmd); 
                rotateCmd = 0.0;
            }
            break;

        case 7:
            if(loopCount < TIME_TURNING_TO_B7_REDB)
            {
                driveCmd = -0.1; rotateCmd = 100.0;
            }
            else
            {
                driveCmd = 0.0; rotateCmd = 0.0;
                dist = camera->GetCurrDistanceMarker();
                surveillance->ResetDriveEncoders();
                loopCount = 0;
                autoStage++;
            }
            break;

        case 8:
            if(fabs(surveillance->GetAverageDriveDis()) < dist)
            {
                driveCmd = driveCmd + Limit(-MAX_AUTO_ACCELERATION, MAX_AUTO_ACCELERATION, -0.5 - driveCmd); 
                rotateCmd = 0.0;
            }
            else
            {
                autoStage++;
            }
            break;

        case 9:
            if(deathStar->GetCellCount() == 3)
            {
                autoStage++;                
            }
            else
            {
                driveCmd = driveCmd + Limit(-MAX_AUTO_ACCELERATION, MAX_AUTO_ACCELERATION, -0.5 - driveCmd); 
                rotateCmd = 0.0;
                collector->StopGrab();
            }
            break;

        case 10:
            if(loopCount < TIME_TURNING_TO_ENDZONE_REDB)
            {
                driveCmd = -0.1; rotateCmd = -100.0;
            }
            else
            {
                driveCmd = 0.0; rotateCmd = 0.0;
                dist = camera->GetCurrDistanceMarker();
                surveillance->ResetDriveEncoders();
                loopCount = 0;
                autoStage++;
            }
            break;

        case 11:
            if(surveillance->GetAverageDriveDis() < DISTANCE_TO_ENDZONE_REDB)
            {
                driveCmd = driveCmd + Limit(-MAX_AUTO_ACCELERATION, MAX_AUTO_ACCELERATION, -0.5 - driveCmd); 
                rotateCmd = 0.0;
            }
            else
            {
                driveCmd = 0.0; rotateCmd = 0.0;
                loopCount = 0;
                autoStage++;
            }
            break;

        case 12:
            driveCmd = 0.0; rotateCmd = 0.0;
            autoMode = autoModeSecond;
            autoStage = 0;
            loopCount = 0;
            break;

    }
}




void TalonXXI::GalacticSearchBlueA()
{
    loopCount++;
    switch (autoStage)
    {
        case 0:
            driveCmd = 0.0; rotateCmd = 0.0;
            surveillance->ResetDriveEncoders();
            printf("GalacticSearchBlueA\n");
            loopCount = 0;
            autoStage++;
            break;

        
        case 1:
            if(loopCount < TIME_TURNING_TO_E6_BLUEA)
            {
                driveCmd = -0.1; rotateCmd = 100.0;
            }
            else
            {
                driveCmd = 0.0; rotateCmd = 0.0;
                dist = camera->GetCurrDistanceMarker();
                surveillance->ResetDriveEncoders();
                loopCount = 0;
                autoStage++;
            }
            break;

        case 2:
            if(fabs(surveillance->GetAverageDriveDis()) < dist)
            {
                driveCmd = driveCmd + Limit(-MAX_AUTO_ACCELERATION, MAX_AUTO_ACCELERATION, -0.5 - driveCmd); 
                rotateCmd = 0.0;
            }
            else
            {
                autoStage++;
            }
            break;

        case 3:
            if(deathStar->GetCellCount() == 1)
            {
                autoStage++;                
            }
            else
            {
                driveCmd = driveCmd + Limit(-MAX_AUTO_ACCELERATION, MAX_AUTO_ACCELERATION, -0.5 - driveCmd); 
                rotateCmd = 0.0;
            }
            break;

        case 4:
            if(loopCount < TIME_TURNING_TO_B7_BLUEA)
            {
                driveCmd = -0.1; rotateCmd = -100.0;
            }
            else
            {
                driveCmd = 0.0; rotateCmd = 0.0;
                dist = camera->GetCurrDistanceMarker();
                surveillance->ResetDriveEncoders();
                loopCount = 0;
                autoStage++;
            }
            break;

        case 5:
            if(fabs(surveillance->GetAverageDriveDis()) < dist)
            {
                driveCmd = driveCmd + Limit(-MAX_AUTO_ACCELERATION, MAX_AUTO_ACCELERATION, -0.5 - driveCmd); 
                rotateCmd = 0.0;
            }
            else
            {
                autoStage++;
            }
            break;

        case 6:
            if(deathStar->GetCellCount() == 2)
            {
                autoStage++;                
            }
            else
            {
                driveCmd = driveCmd + Limit(-MAX_AUTO_ACCELERATION, MAX_AUTO_ACCELERATION, -0.5 - driveCmd); 
                rotateCmd = 0.0;
            }
            break;

        case 7:
            if(loopCount < TIME_TURNING_TO_C9_BLUEA)
            {
                driveCmd = -0.1; rotateCmd = 100.0;
            }
            else
            {
                driveCmd = 0.0; rotateCmd = 0.0;
                dist = camera->GetCurrDistanceMarker();
                surveillance->ResetDriveEncoders();
                loopCount = 0;
                autoStage++;
            }
            break;

        case 8:
            if(fabs(surveillance->GetAverageDriveDis()) < dist)
            {
                driveCmd = driveCmd + Limit(-MAX_AUTO_ACCELERATION, MAX_AUTO_ACCELERATION, -0.5 - driveCmd); 
                rotateCmd = 0.0;
            }
            else
            {
                autoStage++;
            }
            break;

        case 9:
            if(deathStar->GetCellCount() == 3)
            {
                autoStage++;                
            }
            else
            {
                driveCmd = driveCmd + Limit(-MAX_AUTO_ACCELERATION, MAX_AUTO_ACCELERATION, -0.5 - driveCmd); 
                rotateCmd = 0.0;
            }
            break;

        case 10:
            if(surveillance->GetAverageDriveDis() < DISTANCE_TO_ENDZONE_BLUEA)
            {
                driveCmd = driveCmd + Limit(-MAX_AUTO_ACCELERATION, MAX_AUTO_ACCELERATION, -0.5 - driveCmd); 
                rotateCmd = 0.0;
            }
            else
            {
                driveCmd = 0.0; rotateCmd = 0.0;
                loopCount = 0;
                autoStage++;
            }
            break;

        case 11:
            driveCmd = 0.0; rotateCmd = 0.0;
            autoMode = autoModeSecond;
            autoStage = 0;
            loopCount = 0;
            break;


    }
}

void TalonXXI::GalacticSearchBlueB()
{
    loopCount++;
    switch (autoStage)
    {
        case 0:
            driveCmd = 0.0; rotateCmd = 0.0;
            surveillance->ResetDriveEncoders();
            printf("GalacticSearchBlueB\n");
            loopCount = 0;
            autoStage++;
            break;

        case 1:
            if(loopCount < TIME_TURNING_TO_D6_BLUEB)
            {
                driveCmd = -0.1; rotateCmd = -100.0;
            }
            else
            {
                driveCmd = 0.0; rotateCmd = 0.0;
                dist = camera->GetCurrDistanceMarker();
                surveillance->ResetDriveEncoders();
                loopCount = 0;
                autoStage++;
            }
            break;

        case 2:
            if(fabs(surveillance->GetAverageDriveDis()) < dist)
            {
                driveCmd = driveCmd + Limit(-MAX_AUTO_ACCELERATION, MAX_AUTO_ACCELERATION, -0.5 - driveCmd); 
                rotateCmd = 0.0;
            }
            else
            {
                autoStage++;
            }
            break;

        case 3:
            if(deathStar->GetCellCount() == 1)
            {
                autoStage++;                
            }
            else
            {
                driveCmd = driveCmd + Limit(-MAX_AUTO_ACCELERATION, MAX_AUTO_ACCELERATION, -0.5 - driveCmd); 
                rotateCmd = 0.0;
            }
            break;

        case 4:
            if(loopCount < TIME_TURNING_TO_B8_BLUEB)
            {
                driveCmd = -0.1; rotateCmd = 100.0;
            }
            else
            {
                driveCmd = 0.0; rotateCmd = 0.0;
                dist = camera->GetCurrDistanceMarker();
                surveillance->ResetDriveEncoders();
                loopCount = 0;
                autoStage++;
            }
            break;

        case 5:
            if(fabs(surveillance->GetAverageDriveDis()) < dist)
            {
                driveCmd = driveCmd + Limit(-MAX_AUTO_ACCELERATION, MAX_AUTO_ACCELERATION, -0.5 - driveCmd); 
                rotateCmd = 0.0;
            }
            else
            {
                autoStage++;
            }
            break;

        case 6:
            if(deathStar->GetCellCount() == 2)
            {
                autoStage++;                
            }
            else
            {
                driveCmd = driveCmd + Limit(-MAX_AUTO_ACCELERATION, MAX_AUTO_ACCELERATION, -0.5 - driveCmd); 
                rotateCmd = 0.0;
            }
            break;

        case 7:
            if(loopCount < TIME_TURNING_TO_D10_BLUEB)
            {
                driveCmd = -0.1; rotateCmd = -100.0;
            }
            else
            {
                driveCmd = 0.0; rotateCmd = 0.0;
                dist = camera->GetCurrDistanceMarker();
                surveillance->ResetDriveEncoders();
                loopCount = 0;
                autoStage++;
            }
            break;

        case 8:
            if(fabs(surveillance->GetAverageDriveDis()) < dist)
            {
                driveCmd = driveCmd + Limit(-MAX_AUTO_ACCELERATION, MAX_AUTO_ACCELERATION, -0.5 - driveCmd); 
                rotateCmd = 0.0;
            }
            else
            {
                autoStage++;
            }
            break;

        case 9:
            if(deathStar->GetCellCount() == 3)
            {
                autoStage++;                
            }
            else
            {
                driveCmd = driveCmd + Limit(-MAX_AUTO_ACCELERATION, MAX_AUTO_ACCELERATION, -0.5 - driveCmd); 
                rotateCmd = 0.0;
            }
            break;

        case 10:
            if(surveillance->GetAverageDriveDis() < DISTANCE_TO_ENDZONE_BLUEB)
            {
                driveCmd = driveCmd + Limit(-MAX_AUTO_ACCELERATION, MAX_AUTO_ACCELERATION, -0.5 - driveCmd); 
                rotateCmd = 0.0;
            }
            else
            {
                driveCmd = 0.0; rotateCmd = 0.0;
                loopCount = 0;
                autoStage++;
            }
            break;

        case 11:
            driveCmd = 0.0; rotateCmd = 0.0;
            autoMode = autoModeSecond;
            autoStage = 0;
            loopCount = 0;
            break;


    }
}



void TalonXXI::GalacticSearch()
{
    // camera->SetCurrentTarget(0);
    //printf("Target seen: %d\n", camera->SetCurrentTarget(0));
    switch (galacticStage)
    {
    case 0:
        if(camera->SetCurrentTarget(0))
        {
            if(camera->GetCurrDistanceMarker() < 95)
            {
                galacticStage= RED_A;
            }
            else 
            {
                galacticStage= RED_B;
            }
        }
        else
        {
            galacticStage= BLUE;
            surveillance->ResetDriveEncoders();
            driveCmd = 0.0; rotateCmd = 0.0;
        }
        break;

    case BLUE:
        if(fabs(surveillance->GetAverageDriveDis()) < BLUE_START_DISTANCE)
        {
            driveCmd = driveCmd + Limit(-MAX_AUTO_ACCELERATION, MAX_AUTO_ACCELERATION, -0.5 - driveCmd); 
            rotateCmd = 0.0;                
        }
        else
        {
            driveCmd = 0.0; rotateCmd = 0.0;
            if(camera->GetCurrHeadingMarker() > 40)
            {
                galacticStage= BLUE_A;
            }
            else 
            {
                galacticStage= BLUE_B;
            }
        }
        break;

    case RED_A:
        GalacticSearchRedA();
        break;

    case RED_B:
        GalacticSearchRedB();
        break;

    case BLUE_A:
        GalacticSearchBlueA();
        break;

    case BLUE_B:
        GalacticSearchBlueB();
        break;

    default:
        break;

    }
            
}
    