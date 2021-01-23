#ifndef LIMELIGHTCAMERA_H_
#define LIMELIGHTCAMERA_H_

#include "TalonXXI_main.h"
#include "frc/smartdashboard/Smartdashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"

//limelight
#define COMMAND_ARRAY_SIZE         (6)
#define LIMELIGHT_X_OFFSET                    (-3.25)

class TalonXXI;

class LimelightCamera
{
    private:
        TalonXXI *mainRobot;

        //limelight
        std::shared_ptr<NetworkTable> table;
        double seesTarget;
        double targetX;
        double targetY;
        double targetArea;
        double targetRotation;
        double targetLag;
        double targetShortSide;
        double targetLongSide;
        double targetHSide;
        double targetVSide;
        double targetPipelineIndex;
        double targetTransArray[COMMAND_ARRAY_SIZE];

    public:
        // Constructor
        LimelightCamera(TalonXXI* pRobot);
        
        //Robot Functions
        void LocalReset(void);
        void StartingConfig(void);
        void StopAll(void);
        void UpdateDash(void);
        void Analyze(void);
        void TurnOnLED(void);
        void TurnOffLED(void);
        
        //limelight
        inline double GetTargetVisibility() { return seesTarget; };
        bool SeesShooterTarget(void);
        inline double GetTargetHOffset() { return targetX; };
        inline double GetTargetVOffset() { return targetY; };
        inline double GetTargetArea() { return targetArea; };
        inline double GetTargetSkewAngle() { return targetRotation; };
        inline double GetTargetCaptureLag() { return targetLag; };
        inline double GetTargetShortLength() { return targetShortSide; };
        inline double GetTargetLongLength() { return targetLongSide; };
        inline double GetTargetHLength() { return targetHSide; };        
        inline double GetTargetVLength() { return targetVSide; }; 
        inline double GetTargetPipelineIndex() { return targetPipelineIndex; }; 
        void GetTargetTranslation(double *inArray);
        
    private:
        void TargetTranslationReset(void);

};
#endif /*LimeLight_H_*/