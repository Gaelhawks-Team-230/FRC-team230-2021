#ifndef VisionPi_H_
#define VisionPi_H_

#include "TalonXXI_main.h"
#include "frc/smartdashboard/Smartdashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include <vector>

//VisionPi
#define POWER_CELL_ID
#define BLUE_TARGET_ID
#define GREEN_TARGET_ID
#define BLUE_TARGET_ID
#define PURPLE_TARGET_ID

class TalonXXI;

class VisionPi
{
    private:
        TalonXXI *mainRobot;

        //VisionPi
        std::shared_ptr<NetworkTable> table;
        std::vector<double> targetIdentify_array;
        std::vector<double> targetXMarker_array;
        std::vector<double> targetXPowerCell_array;
        std::vector<double> targetYMarker_array;
        std::vector<double> targetYPowerCell_array;
        std::vector<double> targetDistanceMarker_array;
        std::vector<double> targetDistancePowerCell_array;
        std::vector<double> targetHeadingMarker_array;
        std::vector<double> targetHeadingPowerCell_array;
        std::vector<double> currTargetX;
        std::vector<double> currTargetY;
        std::vector<double> currDistanceMarker;
        std::vector<double> currHeadingMarker;

    public:
        //Constructer
        VisionPi(TalonXXI* pRobot);

        //RobotFunctions
        void LocalReset(void)
        void StartingConfig(void)
        void StopAll(void)
        void UpdateDash(void)
        void Analyze (void)
        bool SetCurrentTarget(int TargetID)
        double GetCurrTargetX(void)
        double GetCurrTargetY(void)
        double GetCurrDistanceMarker(void)
        double GetCurrHeadingMarker(void)
}