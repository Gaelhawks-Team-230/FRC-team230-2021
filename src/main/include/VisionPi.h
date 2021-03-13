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
        std::shared_ptr<NetworkTable> cameraTable;
        double is_new_data;    
        std::vector<double> object_id;
        std::vector<double> distance;
        std::vector<double> header;
        std::vector<double> xcenter;
        std::vector<double> ycenter;
        double currTargetX;
        double currTargetY;
        double currDistanceMarker;
        double currHeadingMarker;

    public:
        //Constructer
        VisionPi(TalonXXI* pRobot);

        //RobotFunctions
        void LocalReset(void);
        void StartingConfig(void);
        void StopAll(void);
        void UpdateDash(void);
        void Analyze (void);
        bool SetCurrentTarget(int TargetID);
        double GetCurrTargetX(void);
        double GetCurrTargetY(void);
        double GetCurrDistanceMarker(void);
        double GetCurrHeadingMarker(void);
};

#endif