#include "Common.h"
#include "VisionPi.h"


VisionPi::Visionpi(TalonXXI* pRobot)
{
    mainRobot = pRobot;
    table = nt::NetworkTableInstance::GetDefault().GetTable("VisionPi");
    LocalReset();
}

void Sample::LocalReset()
{
    targetIdentify = []
    targetXMarker = []
    targetXPowerCell = []
    targetYMarker = []
    targetYPowerCell = []
    targetDistanceMarker = []
    targetDistancePowerCell = []
    targetHeadingMarker = []
    targetHeadingPowerCell = []
    currTargetX = 0
    currTargetY = 0
    currDistanceMarker = 0
    currHeadingMarker = 0;
    TargetTranslationReset();
}

void VisionPi::StartingConfig()
{

}

void VisionPi::StopAll()
{

}

void VisionPi::UpdateDash()
{
frc::SmartDashboard::PutNumber("Target Identified", targetIdentify);
frc::SmartDashboard::PutNumber("Target X Marker", targetXMarker);
frc::SmartDashboard::PutNumber("Target X Power Cell", targetXPowerCell);
frc::SmartDashboard::PutNumber("Target Y Marker", targetYMarker);
frc::SmartDashboard::PutNumber("Target Y Power Cell", targetYPowerCell);
frc::SmartDashboard::PutNumber("Target Distance Marker", targetDistanceMarker);
frc::SmartDashboard::PutNumber("Target Distance Power Cell", targetDistancePowerCell);
frc::SmartDashboard::PutNumber("Target Heading Marker", targetHeadingMarker);
frc::SmartDashboard::PutNumber("Target Heading Power Cell", targetHeadingPowerCell);
/*
frc::SmartDashboard::PutNumber("Target Identified Translation", targetTransArray[0]);
frc::SmartDashboard::PutNumber("Target X Marker Translation", targetTransArray[1]);
frc::SmartDashboard::PutNumber("Target X  Power Cell Translation", targetTransArray [2]);
frc::SmartDashboard::PutNumber("Target Y Marker Translation", targetTransArray[3]);
frc::SmartDashboard::PutNumber("Target Y Power Cell Translation", targetTransArray [4]);
frc::SmartDashboard::PutNumber("Target Distance Marker Translaton", targetTransArray[5]);
frc::SmartDashboard::PutNumber("Target Distance Power Cell Translation", targetTransArray[6]);
frc::SmartDashboard::PutNumber("Target Heading Marker Translation", targetTransArray[7]);
frc::SmartDashboard::PutNumber("Target Heading Power Cell Translation", targetTransArray [8]);
*/
}

void VisionPi::Analyze()
{
    is_new_data = table->GetNumber("state", 1);
    if (is_new_data > 1)
    {  
        return;
    }
    table->PutNumber("state", 1);
    object_id = table->GetNumberArray("id", []);
    distance = table->GetNumberArray("distance", []);
    header = table->GetNumberArray("head"[]);
    xcenter = table->GetNumberArray("xcenter", []);
    ycenter = table->GetNumberArray("ycenter", []);


}
bool VisionPi::SetCurrentTarget(int targetID)
{
    for (int i = 0; i < object_id.size(); i++)
    {
        if (object_id[i]==targetID)
        {
            currTargetX = distance[i];
            currTargetY = x_center[i];

            return true;
        }
    } 
    return false;
}

double VisionPi::GetCurrTargetX(void)
{
    return currTargetX;
}

double VisionPi::GetCurrTargetY(void)
{
    return currTargetY;
}

double VisionPi::GetCurrDistanceMarker(void)
{
    return currDistanceMarker;
}

double VisionPi::GetCurrHeadingMarker(void)
{
    return currHeadingMarker;
}