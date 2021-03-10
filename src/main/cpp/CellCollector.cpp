//#include <frc/WPILib.h>
#include "TalonXXI_main.h"
#include "Common.h"
#include "CellCollector.h"

CellCollector::CellCollector(TalonXXI* pRobot)
{
    mainRobot = pRobot;
    grabber = new frc::VictorSP(PWM_GRABBER_WHEELS);    
#ifdef USING_SOLENOID
    cellPiston = new frc::DoubleSolenoid(PCM_GRAB_CELL_1,PCM_GRAB_CELL_2);
#endif
    localSurveillance = mainRobot->surveillance;
    cellPistonCMD = GRABBER_IN;
    LocalReset();
}

//Sets all local variables
void CellCollector::LocalReset()
{
    grabberCommand = 0.0;
    grabberIsIn = true;
    grabOffDelay = 0;
    isGrabbingCells = false;
    operationalMode = IDLE_MODE;
}

//Gatherer is up and in frame perimeter.
void CellCollector::StartingConfig()
{
    GathererIn();
}

void CellCollector::StopAll()
{
    StopGrab();
}

//The grabber is collecting the cells
void CellCollector::GrabCells()
{
    if(operationalMode != UNJAM_MODE)
    {
        operationalMode = GATHERING_MODE;
        grabberCommand = GRAB_CELLS_CMD;
    }
    GathererOut(); 
}

//The grabber is stopping collection
void CellCollector::StopGrab()
{
    if(operationalMode != DELAY_MODE && operationalMode != IDLE_MODE)
    {
        grabOffDelay = 0;
        //operationalMode = IDLE_MODE;
        operationalMode = DELAY_MODE;
        GathererIn();
    }
}

//The power cells will be ejected out in a reverse.
void CellCollector::EjectCells()
{
    grabberCommand = RELEASE_CELLS_CMD;
    operationalMode = EJECTING_MODE;
    GathererOut(); 
}

//Pull the gatherer in.
void CellCollector::GathererIn()
{
    cellPistonCMD = GRABBER_IN;
    grabberIsIn = true;
}

//Pull the gatherer out.
void CellCollector::GathererOut()
{
    cellPistonCMD = GRABBER_OUT;
    grabberIsIn = false;
}

void CellCollector::UpdateDash()
{
    //frc::SmartDashboard::PutNumber("grabberMotorCMD",grabberCommand);
    //frc::SmartDashboard::PutBoolean("Grabber in", grabberIsIn);
    //frc::SmartDashboard::PutBoolean("is grabbing cells", isGrabbingCells);
    //frc::SmartDashboard::PutNumber("grabber current", grabberMotorCurrent);
}

void CellCollector::Service()
{
#ifndef TEST_MODE 
    
    switch (operationalMode)
    {
        case IDLE_MODE:
            grabberCommand = 0.0;
            isGrabbingCells = false;
            break;

        case GATHERING_MODE: //Gathering Cells
            grabOffDelay = 0;
            isGrabbingCells = true;
            if (mainRobot->deathStar->HasCellAtIntakePos())
            {
                grabberCommand = 0.0;
            }
            else
            {
                grabberCommand = GRAB_CELLS_CMD;
            }
            // check for jammed powercell (a.k.a. stalled motor)
            GetCurrent();
            if (grabberMotorCurrent > GRABBER_STALL_CURRENT)
            {
                //If yes, increase stall count
                stallCurrentCount++;
                //If the STALL COUNT is greater than the TARGET STALL COUNT, then stop motor
                if (stallCurrentCount > TARGET_STALL_COUNT)
                {
                    operationalMode = UNJAM_MODE;
                    grabberCommand = 0.0;
                    ejectCellCount = 0.0;
                }
            }
            else
            {
                stallCurrentCount = 0;
            }
            if(mainRobot->deathStar->IsFull())
            {
                GathererIn();
            }
            break;

        case UNJAM_MODE:
            grabberCommand = RELEASE_CELLS_CMD;
            if (ejectCellCount < EJECT_SPIN_TIMER)
            {
                ejectCellCount++;
            }
            else
            {
                ejectCellCount = 0;
                operationalMode = GATHERING_MODE;
            }
            break;
        case EJECTING_MODE:
            isGrabbingCells = false;
            grabberCommand = RELEASE_CELLS_CMD;
            break;

        case DELAY_MODE:
            if(grabOffDelay < DELAY_OFF_TIME)
            {
                grabOffDelay++;
            }
            else
            {
                grabOffDelay = 0;
                operationalMode = IDLE_MODE;
            }
            break;

        default:
            printf("CellCollector::Service - code cannot come here\n");
            break;
    }
#endif
    //printf("%d %d \n", operationalMode, grabOffDelay, grabberCommand);
    grabber->Set(grabberCommand);
#ifdef USING_SOLENOID
    cellPiston->Set(cellPistonCMD);
#endif
}

//Get current and assign it to a variable
void CellCollector::GetCurrent()
{
    grabberMotorCurrent = mainRobot->pdp->GetCurrent(GRABBER_CURRENT);
}

#ifdef TEST_MODE
void CellCollector::TestingSpin(double input)
{
    if(fabs(input) > 0.1)
    {
        grabberCommand = input;
    }
    else
    {
        grabberCommand = 0.0;
    }
    printf("%f \n", grabberCommand);
}
#endif
