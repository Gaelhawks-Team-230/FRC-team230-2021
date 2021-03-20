#include "Common.h"
#include "VisionPi.h"


VisionPi::VisionPi(TalonXXI* pRobot)
{
    mainRobot = pRobot;
    cameraTable = nt::NetworkTableInstance::GetDefault().GetTable("VisionPi");
    //unsigned short port{1735};
    //const char* Server ="PiIP";
    //cameraTable = nt::NetworkTableInstance::Create()::StartClient(Server,port).GetTable("VisionPi");
    LocalReset();
}

void VisionPi::LocalReset()
{
    object_id.clear();
    distance.clear();
    heading.clear();
    xcenter.clear();
    ycenter.clear();
    currTargetX = 0.0;
    currTargetY = 0.0;
    currDistanceMarker = 0.0;
    currHeadingMarker = 0.0;
}

void VisionPi::StartingConfig()
{

}

void VisionPi::StopAll()
{

}

void VisionPi::UpdateDash()
{
// frc::SmartDashboard::PutNumber("Target Identified", targetIdentify);
// frc::SmartDashboard::PutNumber("Target X Marker", targetXMarker);
// frc::SmartDashboard::PutNumber("Target X Power Cell", targetXPowerCell);
// frc::SmartDashboard::PutNumber("Target Y Marker", targetYMarker);
// frc::SmartDashboard::PutNumber("Target Y Power Cell", targetYPowerCell);
// frc::SmartDashboard::PutNumber("Target Distance Marker", targetDistanceMarker);
// frc::SmartDashboard::PutNumber("Target Distance Power Cell", targetDistancePowerCell);
// frc::SmartDashboard::PutNumber("Target Heading Marker", targetHeadingMarker);
// frc::SmartDashboard::PutNumber("Target Heading Power Cell", targetHeadingPowerCell);
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
    is_new_data = cameraTable->GetNumber("state", 1);
    if (is_new_data > 0) //Old Data is 1, New Data is 0
    {  
        return;
    }
    //printf("visionPi state: %f\n", is_new_data); 
    cameraTable->PutNumber("state", 1);
    object_id = cameraTable->GetNumberArray("id", object_id);
    distance = cameraTable->GetNumberArray("distance", distance);
    heading = cameraTable->GetNumberArray("head", heading);
    xcenter = cameraTable->GetNumberArray("xcenter", xcenter);
    ycenter = cameraTable->GetNumberArray("ycenter", ycenter);
    //printf("%f id %f distance %f head %f xcenter %f ycenter", object_id[0], distance[0], heading[0], xcenter[0], ycenter[0]);

}

bool VisionPi::SetCurrentTarget(int targetID)
{
    for (unsigned int i = 0; i < object_id.size(); i++)
    {
        if (object_id[i]==targetID)
        {
            currTargetX = xcenter[i];
            currTargetY = ycenter[i];

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