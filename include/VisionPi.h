#ifndef VisionPi_H_
#define VisionPi_H_

#include "TalonXXI_main.h"
#include "frc/smartdashboard/Smartdashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"

//VisionPi
#define 
#define

class TalonXXI;

class VisionPi
{
    private:
        TalonXXI *mainRobot;

        //VisionPi
        std::shared_ptr<NetworkTable> table;
        std::vector<double>targetIdentify_array;
        std::vector<double>targetXMarker_array;
        std::vector<double>targetXPowerCell_array;
        std::vector<double>targetYMarker_array;
        std::vector<double>targetYPowerCell_array;
        std::vector<double>targetDistanceMarker_array;
        std::vector<double>targetDistancePowerCell_array;
        std::vector<double>targetHeadingMarker_array;
        std::vector<double>targetHeadingPowerCell_array;

    public:
        //Constructer
        VisionPi(TalonXXI* pRobot);

        //RobotFunctions
        void LocalReset(void)
        void StartingConfig(void)
        void StopAll(void)
        void UpdateDash(void)
        void Analyze (void)
}