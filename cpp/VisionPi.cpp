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
    targetIdentify = 0.0
    targetX = 0.0
    targetY = 0.0
    targetDistance = 0.0;
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
frc::SmartDashboard::PutNumber("Target X", targetX);
frc::SmartDashboard::PutNumber("Target Y", targetY);
frc::SmartDashboard::PutNumber("Target Distance", targetDistance);
/*
frc::SmartDashboard::PutNumber("Target Identified Translation", targetTransArray[0]);
frc::SmartDashboard::PutNumber("Target X Translation", targetTransArray[1]);
frc::SmartDashboard::PutNumber("Target Y Translation", targetTransArray[2]);
frc::SmartDashboard::PutNumber("Target Distance Translaton", targetTransArray[4]);
*/
}

void VisionPi::Analyze()
{
    targetIdentify = table->GetNumber("ti", 0.0);
    targetX = table->GetNumber("tx", 0.0);
    targetY = table->GetNumber("ty", 0.0);
    targetDistance = table->GetNumber("td", 0.0);
}

//Called every loop (used for timing related stuff)
void Sample::Service()
{
    
}
