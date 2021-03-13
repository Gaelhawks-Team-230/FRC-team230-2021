/*
#include "Common.h"
#include "LightController.h"

LightController::LightController(TalonXXI* pRobot)
{
    light1 = new frc::DigitalOutput(LED_LIGHT_OUTPUT_3);
	light2 = new frc::DigitalOutput(LED_LIGHT_OUTPUT_2);
	light3 = new frc::DigitalOutput(LED_LIGHT_OUTPUT_1);
    mainRobot = pRobot;
    LocalReset();
}

void LightController::LocalReset()
{
    light1->Set(true);
    light2->Set(true);        
    light3->Set(true);
    isTargetting = false;
    loopCount = 0;
}

void LightController::StartingConfig()
{
    LocalReset();
}

void LightController::StopAll()
{
    LocalReset();
}


//Anything that you want printed on the dashboard for testing or during match. 
//Any information which may be important. 
void LightController::UpdateDash()
{

}
//Called every loop
void LightController::Service()
{
    //int cellCount = mainRobot->deathStar->GetCellCount();
    light1->Set(true);
    light2->Set(true);
    light3->Set(true);

    loopCount++;
    cellCount = (int) (loopCount/100);
    printf("cellCount %d\n", cellCount);
    if (cellCount > 5)
    {
        cellCount = 0;
        loopCount = 0;
    }

    if(cellCount == 0)
    {
        //Search mode :)
        //good
        light1->Set(true);
        light2->Set(true);
        light3->Set(true);
    }
    else if(cellCount == 1)
    {
        //good
        light1->Set(true);
        light2->Set(true);
        light3->Set(false);
    }
    else if(cellCount == 2)
    {
        
        //good
        light1->Set(true);
        light2->Set(false);
        light3->Set(true);
    }
    else if(cellCount == 3)
    {
        light1->Set(true);
        light2->Set(false);
        light3->Set(false);
    }
    else if(cellCount == 4)
    {
        light1->Set(false);
        light2->Set(true);
        light3->Set(true);
    }
    else if(cellCount == 5)
    {
        light1->Set(false);
        light2->Set(true);
        light3->Set(false);

    }
    else
    {
        light1->Set(false);
        light2->Set(false);
        light3->Set(false);
    }
    */