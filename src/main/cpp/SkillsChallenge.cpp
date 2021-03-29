#include "Common.h"
#include "SkillsChallenge.h"
#include "TalonXXI_main.h"


/*
void TalonXXII::SlalomPath()
{
    loopCount++;
    switch (autoStage)
    {
        case 0:
            driveCmd = 0.0; rotateCmd = 0.0;
            surveillance->ResetDriveEncoders();
            loopCount = 0;
            autoStage++;
            break;

        case 1:
            //track D4 with vision system
            driveCmd = 0.0; rotateCmd = 0.0;
            loopCount = 0;
            autoStage++;
            break;
        
        case 2:
            //calculate trajectory to get around d4? or we can just go straight and turn?
            //driveCmd = 0.0; rotateCmd = 0.0;
            loopCount = 0;
            autoStage++;
            break;

        case 3:
            //drive calculated trajectory (somewhere near d4)
            //driveCmd = 0.0; rotateCmd = 0.0;
            loopCount = 0;
            autoStage++;
            break;

        case 4:
            //go straight until somewhere near D10
            if(surveillance->GetAverageDriveDis() < //somewhere near D10)
            {
                driveCmd = driveCmd + Limit(MIN_AUTO_ACCELERATION, MAX_AUTO_ACCELERATION, -0.5 - driveCmd); //same speed as last year?
                rotateCmd = 0.0;
                
            }
            else
            {
                //driveCmd = 0.0; rotateCmd = 0.0; ? would we turn the motors off?
                loopCount = 0;
                autoStage++;
            }
            break;

        case 5:
            //track D10 with vision system
            break;

        case 5:
            //calculate trajectory to make the loop around D10
            break;

        case 6:
            //drive calculated trajectory around d10
            //driveCmd = 0.0; rotateCmd = 0.0;
            loopCount = 0;
            autoStage++;
            break;

        case 7:
            //go straight until somewhere near D4- maybe track D4?
            if(surveillance->GetAverageDriveDis() < //somewhere near D4)
            {
                driveCmd = driveCmd + Limit(MIN_AUTO_ACCELERATION, MAX_AUTO_ACCELERATION, -0.5 - driveCmd); //same speed as last year?
                rotateCmd = 0.0;
                
            }
            else
            {
                //driveCmd = 0.0; rotateCmd = 0.0; ?
                loopCount = 0;
                autoStage++;
            }
            break;

        case 8:
            //same question as before
            //turn to face end zone
            driveCmd = 0.0; 
            rotateCmd = //?
            loopCount = 0;
            autoStage++;
            break;

        case 9:
            //drive to end zone
            driveCmd = driveCmd + Limit(MIN_AUTO_ACCELERATION, MAX_AUTO_ACCELERATION, -0.5 - driveCmd); //?
            rotateCmd = 0.0;
            loopCount = 0;
            autoStage++;
            break;

        case 10:
            driveCmd = 0.0; rotateCmd = 0.0;
            surveillance->ResetDriveEncoders();
            loopCount = 0;
            autoStage = 0;
            break;


    }
}

void TalonXXII::BarrelPath()
{
    loopCount++;
    switch (autoStage)
    {
        case 0:
            driveCmd = 0.0; rotateCmd = 0.0;
            surveillance->ResetDriveEncoders();
            loopCount = 0;
            autoStage++;
            break;

        case 1:
            //track D5
            driveCmd = 0.0; rotateCmd = 0.0;
            loopCount = 0;
            autoStage++;
            break;
        
        case 2:
            //calculate trajectory to get to first cross section on map... or just go straight
            //driveCmd = 0.0; rotateCmd = 0.0;
            loopCount = 0;
            autoStage++;
            break;
        
        case 3:
            //drive calculated trajectory (to cross section)
            //driveCmd = 0.0; rotateCmd = 0.0;
            loopCount = 0;
            autoStage++;
            break;

        case 4:
            //calculate trajectory to get around d5
            //driveCmd = 0.0; rotateCmd = 0.0;
            loopCount = 0;
            autoStage++;
            break;
        
        case 5:
            //drive calculated trajectory (around d5)
            //driveCmd = 0.0; rotateCmd = 0.0;
            loopCount = 0;
            autoStage++;
            break;

        case 6:
            //go straight until big cross section
            if(surveillance->GetAverageDriveDis() < //big cross section)
            {
                driveCmd = driveCmd + Limit(MIN_AUTO_ACCELERATION, MAX_AUTO_ACCELERATION, -0.5 - driveCmd); //same speed as last year?
                rotateCmd = 0.0;
                
            }
            else
            {
                //driveCmd = 0.0; rotateCmd = 0.0; ?
                loopCount = 0;
                autoStage++;
            }
            break;

        case 7:
            //track D8
            //driveCmd = 0.0; rotateCmd = 0.0;
            loopCount = 0;
            autoStage++;
            break;

        case 8:
            //calculate trajectory to get around d8
            //driveCmd = 0.0; rotateCmd = 0.0;
            loopCount = 0;
            autoStage++;
            break;
        
        case 9:
            //drive calculated trajectory (around d8)
            //driveCmd = 0.0; rotateCmd = 0.0;
            loopCount = 0;
            autoStage++;
            break;

        case 10:
            //track d10
            //driveCmd = 0.0; rotateCmd = 0.0;
            loopCount = 0;
            autoStage++;
            break;

        case 11:
            //calculate trajectory to get around d10
            //driveCmd = 0.0; rotateCmd = 0.0;
            loopCount = 0;
            autoStage++;
            break;
        
        case 12:
            //drive calculated trajectory (around d10)
            //driveCmd = 0.0; rotateCmd = 0.0;
            loopCount = 0;
            autoStage++;
            break;

        case 13:
            //go straight until end zone
            if(surveillance->GetAverageDriveDis() < //endzone)
            {
                driveCmd = driveCmd + Limit(MIN_AUTO_ACCELERATION, MAX_AUTO_ACCELERATION, -0.5 - driveCmd); //same speed as last year?
                rotateCmd = 0.0;
                
            }
            else
            {
                driveCmd = 0.0; rotateCmd = 0.0; ?
                loopCount = 0;
                autoStage++;
            }
            break;

        case 14:
            driveCmd = 0.0; rotateCmd = 0.0;
            autoMode = autoModeSecond;
            autoStage = 0;
            loopCount = 0;
            break;



    }
}

void TalonXXII::BouncePath()
{
    loopCount++;
    switch (autoStage)
    {
        case 0:
            driveCmd = 0.0; rotateCmd = 0.0;
            surveillance->ResetDriveEncoders();
            loopCount = 0;
            autoStage++;
            break;
        case 1:
            //track a3
            driveCmd = 0.0; rotateCmd = 0.0;
            loopCount = 0;
            autoStage++;
            break;
        
        case 2:
            //calculate trajectory to get to A3
            driveCmd = 0.0; rotateCmd = 0.0;
            loopCount = 0;
            autoStage++;
            break;
        
        case 3:
            //drive calculated trajectory (to hit a3)
            //driveCmd = 0.0; rotateCmd = 0.0;
            loopCount = 0;
            autoStage++;
            break;

        case 4:
            //calculate trajectory  (backwards?) to get all the way around d5 until between d5 and d7?
            driveCmd = 0.0; rotateCmd = 0.0;
            loopCount = 0;
            autoStage++;
            break;

        case 5:
            //drive calculated trajectory
            //driveCmd = 0.0; rotateCmd = 0.0;
            loopCount = 0;
            autoStage++;
            break;

        case 6:
            //drive straight (backwards?) to hit a6 (might need to add a case for vision system)
            if(surveillance->GetAverageDriveDis() < //a6)
            {
                driveCmd = driveCmd + Limit(MIN_AUTO_ACCELERATION, MAX_AUTO_ACCELERATION, -0.5 - driveCmd); 
                rotateCmd = 0.0;
                
            }
            else
            {
                driveCmd = 0.0; rotateCmd = 0.0; ?
                loopCount = 0;
                autoStage++;
            }
            break;

        case 7:
            //drive straight until near d7
            if(surveillance->GetAverageDriveDis() < //d7ish)
            {
                driveCmd = driveCmd + Limit(MIN_AUTO_ACCELERATION, MAX_AUTO_ACCELERATION, -0.5 - driveCmd); 
                rotateCmd = 0.0;
                
            }
            else
            {
                driveCmd = 0.0; rotateCmd = 0.0; ?
                loopCount = 0;
                autoStage++;
            }
            break;

        case 8:
            //calculate trajectory around d7+d8
            driveCmd = 0.0; rotateCmd = 0.0;
            loopCount = 0;
            autoStage++;
            break;

        case 9:
            //drive straight to hit a9
            if(surveillance->GetAverageDriveDis() < //a9)
            {
                driveCmd = driveCmd + Limit(MIN_AUTO_ACCELERATION, MAX_AUTO_ACCELERATION, -0.5 - driveCmd); 
                rotateCmd = 0.0;
                
            }
            else
            {
                driveCmd = 0.0; rotateCmd = 0.0; ?
                loopCount = 0;
                autoStage++;
            }
            break;

        case 10:
            //calculate trajectory to finish zone
            driveCmd = 0.0; rotateCmd = 0.0;
            loopCount = 0;
            autoStage++;
            break;

        case 11:
            //drive calculated trajectory
            //driveCmd = 0.0; rotateCmd = 0.0;
            loopCount = 0;
            autoStage++;
            break;

        case 12:
            driveCmd = 0.0; rotateCmd = 0.0;
            autoMode = autoModeSecond;
            autoStage = 0;
            loopCount = 0;
            break;

    }
}
*/

void TalonXXII::GalacticSearchRedA()
{
    loopCount++;
    switch (autoStage)
    {
        case 0:
            driveCmd = 0.0; rotateCmd = 0.0;
            surveillance->ResetDriveEncoders();
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
            if(fabs(surveillance->GetAverageDriveDis()) < dist)
            {
                driveCmd = driveCmd + Limit(MIN_AUTO_ACCELERATION, MAX_AUTO_ACCELERATION, -0.5 - driveCmd); 
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
                dist = camera->GetCurrDistanceMarker();                
                autoStage++;
            }
            else
            {
                driveCmd = driveCmd + Limit(MIN_AUTO_ACCELERATION, MAX_AUTO_ACCELERATION, -0.5 - driveCmd); 
                rotateCmd = 0.0;
            }
            break;

        case 4:
            if(fabs(surveillance->GetAverageDriveDis()) < dist)
            {
                driveCmd = driveCmd + Limit(MIN_AUTO_ACCELERATION, MAX_AUTO_ACCELERATION, -0.5 - driveCmd); 
                rotateCmd = 0.0;
            }
            else
            {
                autoStage++;
            }
            break;

        case 5:
            if(deathStar->GetCellCount() == 2)
            {
                dist = camera->GetCurrDistanceMarker();
                surveillance->ResetDriveEncoders();
                autoStage++;
            }
            else
            {
                driveCmd = driveCmd + Limit(MIN_AUTO_ACCELERATION, MAX_AUTO_ACCELERATION, -0.5 - driveCmd); 
                rotateCmd = 0.0;
            }
            break;

        case 6:
            if(loopCount < TIME_TURNING_TO_A6_REDA)
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

        case 7:
            if(fabs(surveillance->GetAverageDriveDis()) < dist)
            {
                driveCmd = driveCmd + Limit(MIN_AUTO_ACCELERATION, MAX_AUTO_ACCELERATION, -0.5 - driveCmd); 
                rotateCmd = 0.0;
            }
            else
            {
                autoStage++;
            }
            break;

        case 8:
            if(deathStar->GetCellCount() == 3)
            {
                autoStage++;                
            }
            else
            {
                driveCmd = driveCmd + Limit(MIN_AUTO_ACCELERATION, MAX_AUTO_ACCELERATION, -0.5 - driveCmd); 
                rotateCmd = 0.0;
                collector->StopGrab();
            }
            break;

        case 9:
            if(loopCount < TIME_TURNING_TO_ENDZONE_REDA)
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

        case 10:
            if(surveillance->GetAverageDriveDis() < DISTANCE_TO_ENDZONE_REDA)
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

        case 11:
            driveCmd = 0.0; rotateCmd = 0.0;
            autoMode = autoModeSecond;
            autoStage = 0;
            loopCount = 0;
            break;

    }
}




void TalonXXII::GalacticSearchRedB()
{
    loopCount++;
    switch (autoStage)
    {
        case 0:
            driveCmd = 0.0; rotateCmd = 0.0;
            surveillance->ResetDriveEncoders();
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
            angle = camera->GetCurrHeadingMarker();
            
            surveillance->ResetDriveEncoders();
            break;

        case 3:
            if(fabs(surveillance->GetAverageDriveDis()) < dist)
            {
                driveCmd = driveCmd + Limit(MIN_AUTO_ACCELERATION, MAX_AUTO_ACCELERATION, -0.5 - driveCmd); 
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
                driveCmd = driveCmd + Limit(MIN_AUTO_ACCELERATION, MAX_AUTO_ACCELERATION, -0.5 - driveCmd); 
                rotateCmd = 0.0;
            }
            break;

        case 5:
            if(fabs(surveillance->GetAverageDriveDis()) < dist)
            {
                driveCmd = driveCmd + Limit(MIN_AUTO_ACCELERATION, MAX_AUTO_ACCELERATION, -0.5 - driveCmd); 
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
                driveCmd = driveCmd + Limit(MIN_AUTO_ACCELERATION, MAX_AUTO_ACCELERATION, -0.5 - driveCmd); 
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
                driveCmd = driveCmd + Limit(MIN_AUTO_ACCELERATION, MAX_AUTO_ACCELERATION, -0.5 - driveCmd); 
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
                driveCmd = driveCmd + Limit(MIN_AUTO_ACCELERATION, MAX_AUTO_ACCELERATION, -0.5 - driveCmd); 
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

        case 12:
            driveCmd = 0.0; rotateCmd = 0.0;
            autoMode = autoModeSecond;
            autoStage = 0;
            loopCount = 0;
            break;

    }
}




void TalonXXII::GalacticSearchBlueA()
{
    loopCount++;
    switch (autoStage)
    {
        case 0:
            driveCmd = 0.0; rotateCmd = 0.0;
            surveillance->ResetDriveEncoders();
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
                driveCmd = driveCmd + Limit(MIN_AUTO_ACCELERATION, MAX_AUTO_ACCELERATION, -0.5 - driveCmd); 
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
                driveCmd = driveCmd + Limit(MIN_AUTO_ACCELERATION, MAX_AUTO_ACCELERATION, -0.5 - driveCmd); 
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
                driveCmd = driveCmd + Limit(MIN_AUTO_ACCELERATION, MAX_AUTO_ACCELERATION, -0.5 - driveCmd); 
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
                driveCmd = driveCmd + Limit(MIN_AUTO_ACCELERATION, MAX_AUTO_ACCELERATION, -0.5 - driveCmd); 
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
                driveCmd = driveCmd + Limit(MIN_AUTO_ACCELERATION, MAX_AUTO_ACCELERATION, -0.5 - driveCmd); 
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
                driveCmd = driveCmd + Limit(MIN_AUTO_ACCELERATION, MAX_AUTO_ACCELERATION, -0.5 - driveCmd); 
                rotateCmd = 0.0;
            }
            break;

        case 10:
            if(surveillance->GetAverageDriveDis() < DISTANCE_TO_ENDZONE_BLUEA)
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

        case 11:
            driveCmd = 0.0; rotateCmd = 0.0;
            autoMode = autoModeSecond;
            autoStage = 0;
            loopCount = 0;
            break;


    }
}

void TalonXXII::GalacticSearchBlueB()
{
    loopCount++;
    switch (autoStage)
    {
        case 0:
            driveCmd = 0.0; rotateCmd = 0.0;
            surveillance->ResetDriveEncoders();
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
                driveCmd = driveCmd + Limit(MIN_AUTO_ACCELERATION, MAX_AUTO_ACCELERATION, -0.5 - driveCmd); 
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
                driveCmd = driveCmd + Limit(MIN_AUTO_ACCELERATION, MAX_AUTO_ACCELERATION, -0.5 - driveCmd); 
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
                driveCmd = driveCmd + Limit(MIN_AUTO_ACCELERATION, MAX_AUTO_ACCELERATION, -0.5 - driveCmd); 
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
                driveCmd = driveCmd + Limit(MIN_AUTO_ACCELERATION, MAX_AUTO_ACCELERATION, -0.5 - driveCmd); 
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
                driveCmd = driveCmd + Limit(MIN_AUTO_ACCELERATION, MAX_AUTO_ACCELERATION, -0.5 - driveCmd); 
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
                driveCmd = driveCmd + Limit(MIN_AUTO_ACCELERATION, MAX_AUTO_ACCELERATION, -0.5 - driveCmd); 
                rotateCmd = 0.0;
            }
            break;

        case 10:
            //drive straight to c3
            if(surveillance->GetAverageDriveDis() < DISTANCE_TO_ENDZONE_BLUEB)
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

        case 11:
            driveCmd = 0.0; rotateCmd = 0.0;
            autoMode = autoModeSecond;
            autoStage = 0;
            loopCount = 0;
            break;


    }
}



void TalonXXII::GalacticSearch()
{
    camera->SetCurrentTarget(0);
    if(SetCurrentTarget(0))
    {
        if(camera->GetCurrDistanceMarker() < 72)
        {
            GalacticSearchRedA();
        }
        else 
        {
            GalacticSearchRedB()
        }
    }
    else
    {
       if(fabs(surveillance->GetAverageDriveDis()) < BLUE_START_DISTANCE)
        {
            driveCmd = driveCmd + Limit(MIN_AUTO_ACCELERATION, MAX_AUTO_ACCELERATION, -0.5 - driveCmd); 
            rotateCmd = 0.0;                
        }
        else
        {
            driveCmd = 0.0; rotateCmd = 0.0;
        }
         
         if(camera->GetCurrHeadingMarker() > 40)
        {
            GalacticSearchBlueA();
        }
        else 
        {
            GalacticSearchBlueB()
        }

    }
            
    }
    