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
            //calculate trajectory to get around it? or we can just go straight and turn?
            loopCount = 0;
            autoStage++;
            break;

        case 2:
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

        case 3:
            //track D10 with vision system
            //calculate trajectory to make the loop around D10
            if(surveillance->GetAverageDriveDis() < //after the loop?)
            {
                driveCmd = //?
                rotateCmd = //?
                
            }
            else
            {
                //driveCmd = 0.0; rotateCmd = 0.0; ?
                loopCount = 0;
                autoStage++;
            }
            break;

        case 4:
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

        case 5:
            //same question as before
            //turn to face end zone
            driveCmd = 0.0; 
            rotateCmd = //?
            loopCount = 0;
            autoStage++;
            break;

        case 6:
            //drive to end zone
            driveCmd = driveCmd + Limit(MIN_AUTO_ACCELERATION, MAX_AUTO_ACCELERATION, -0.5 - driveCmd); //?
            rotateCmd = 0.0;
            loopCount = 0;
            autoStage++;
            break;

        case 7:
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

    }
}