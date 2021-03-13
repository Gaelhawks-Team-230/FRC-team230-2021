#include "Common.h"
#include "Autonomous.h"
#include "TalonXXII_main.h"

void TalonXXII::DoNothing()
{
    driveCmd = 0.0; rotateCmd = 0.0;
    loopCount = 0;
    autoStage = 0;
}

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
            //track c3
            driveCmd = 0.0; rotateCmd = 0.0;
            loopCount = 0;
            autoStage++;
            break;

        case 2:
            //drive straight to c3
            if(surveillance->GetAverageDriveDis() < //c3)
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

        case 3:
            driveCmd = 0.0; rotateCmd = 0.0;
            collector->GrabCells();
            deathStar->IntakeCells();
            loopCount = 0;
            autoStage++;

        case 4:
            //turn until facing d5 (until vision system sees d5?)
            //driveCmd = 0.0; rotateCmd = 0.0;
            loopCount = 0;
            autoStage++;
            break;

        case 5:
            //drive straight to d5
            if(surveillance->GetAverageDriveDis() < //d5)
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

        case 6:
            driveCmd = 0.0; rotateCmd = 0.0;
            collector->GrabCells();
            deathStar->IntakeCells();
            loopCount = 0;
            autoStage++;

        case 7:
            //turn until facing a6 (until vision system sees a6?)
            //driveCmd = 0.0; rotateCmd = 0.0;
            loopCount = 0;
            autoStage++;
            break;

        case 8:
            //drive straight to a6
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

        case 9:
            driveCmd = 0.0; rotateCmd = 0.0;
            collector->GrabCells();
            deathStar->IntakeCells();
            loopCount = 0;
            autoStage++;

        case 10:
            //turn to face finish zone
            //driveCmd = 0.0; rotateCmd = 0.0;
            loopCount = 0;
            autoStage++;
            break;

        case 11:
            //drive straight to finish zone
            if(surveillance->GetAverageDriveDis() < //finish zone)
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

        case 12:
            driveCmd = 0.0; rotateCmd = 0.0;
            autoMode = autoModeSecond;
            autoStage = 0;
            loopCount = 0;
            break;



    }
}