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
    targetIdentify_array = table->GetNumberArray("ti", []);
    targetXMarker_array = table->GetNumberArray("txm", []);
    targetXPowerCell_array = table->GetNumberArray("txp"[]);
    targetYMarker_array = table->GetNumberArray("tym", []);
    targetYPowerCell_array = table->GetNumberArray("typ", []);
    targetDistanceMarker_array = table->GetNumberArray("tdm",[]);
    targetDistancePowerCell_array = table->GetNumberArray("tdp", []);
    targetHeadingMarker_array = table->GetNumberArray("thm",[]);
    targetHeadingPowerCell_array = table->GetNumberArray("thp",[]);

}
bool VisionPi::SetCurrentTarget(int targetID)
{
    for (int i = 0; i < targetIdentify_array.size(); i++)
    {
        if (targetIdentify_array[i]==targetID)
        {
            currTargetX = targetXMarker_array[i];
            currTargetY = targetYMarker_array[i];
            currDistanceMarker = targetDistanceMarker_array[i];
            currHeadingMarker = targetHeadingMarker_array[i];

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