/*
#ifndef LIGHTCONTROLLER_H_
#define LIGHTCONTROLLER_H_

#include "Common.h"
#include <frc/DigitalOutput.h>

class TalonXXI;


class LightController
{
    private:
  
        frc::DigitalOutput *light1;
        frc::DigitalOutput *light2;
        frc::DigitalOutput *light3;

        TalonXXI *mainRobot;
        
        bool isTargetting;
        int loopCount;
   
    public:
        LightController(TalonXXI* pRobot);
        void LocalReset(void);
        void StartingConfig(void);
        void StopAll(void);
        void UpdateDash(void);
        void Service(void);

};
#endif /*LIGHTCONTROLLER_H_*/
*/
