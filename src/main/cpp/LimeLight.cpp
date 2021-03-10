#include "Common.h"
#include "Limelight.h"


LimelightCamera::LimelightCamera(TalonXXI* pRobot)
{
    mainRobot = pRobot;
    table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
    LocalReset();
}

void LimelightCamera::LocalReset()
{
    seesTarget = 0.0;
    targetX = 0.0;
    targetY = 0.0;
    /*targetArea = 0.0;
    targetRotation = 0.0;
    targetLag = 0.0;
    targetShortSide = 0.0;
    targetLongSide = 0.0;
    targetHSide = 0.0;
    targetVSide = 0.0;
    targetPipelineIndex = 0.0;*/
    horizontalOffset = 0.0;
    addedOffset = 0.0;
    //TargetTranslationReset();
}

void LimelightCamera::StartingConfig()
{
    
}

void LimelightCamera::StopAll()
{

}

void LimelightCamera::UpdateDash()
{
    //table->PutNumber("stream", 0);
    frc::SmartDashboard::PutBoolean("Target Seen:", SeesShooterTarget());
    frc::SmartDashboard::PutNumber("Target X:", targetX);
    frc::SmartDashboard::PutNumber("Target Y:", targetY);
  /*  frc::SmartDashboard::PutNumber("Target Area:", targetArea);
    frc::SmartDashboard::PutNumber("Target Rotation:", targetRotation);
    frc::SmartDashboard::PutNumber("Target Lag:", targetLag);
    frc::SmartDashboard::PutNumber("Target Short Side:", targetShortSide);
    frc::SmartDashboard::PutNumber("Target Long Side:", targetLongSide);
    frc::SmartDashboard::PutNumber("Target Horizontal Side:", targetHSide);
    frc::SmartDashboard::PutNumber("Target Vertical Side:", targetVSide);
   */ /*
    frc::SmartDashboard::PutNumber("Target X Translation:", targetTransArray[0]);
    frc::SmartDashboard::PutNumber("Target Y Translation:", targetTransArray[1]);
    frc::SmartDashboard::PutNumber("Target Z Translation:", targetTransArray[2]);
    frc::SmartDashboard::PutNumber("Target Pitch Translation:", targetTransArray[3]);
    frc::SmartDashboard::PutNumber("Target Yaw Translation:", targetTransArray[4]);
    frc::SmartDashboard::PutNumber("Target Roll Translation:", targetTransArray[5]);
    */
}

void LimelightCamera::Analyze()
{
    seesTarget = table->GetNumber("tv",0.0);
    targetX = table->GetNumber("tx",0.0);
    targetY = table->GetNumber("ty",0.0);
    /*targetArea = table->GetNumber("ta",0.0);
    targetRotation = table->GetNumber("ts",0.0);
    targetLag = table->GetNumber("tl",0.0);
    targetShortSide = table->GetNumber("tshort",0.0);
    targetLongSide = table->GetNumber("tlong",0.0);
    targetHSide = table->GetNumber("thor",0.0);
    targetVSide = table->GetNumber("tvert",0.0);
    targetPipelineIndex = table->GetNumber("getpipe",0.0);*/
    if(targetY > LIMELIGHT_Y_THRESHOLD)
    {
        horizontalOffset = LIMELIGHT_X_OFFSET_NEAR;
    }
    else
    {
        horizontalOffset = LIMELIGHT_X_OFFSET_FAR;
    }
    targetX = targetX - (horizontalOffset + addedOffset);
    //targetTransArray[COMMAND_ARRAY_SIZE] = table->GetNumber("camtran",0.0);   needs more research
}

void LimelightCamera::AutoHorizontalOffset(int autoMode)
{
    if(autoMode == 4)
    {
        addedOffset = LIMELIGHT_X_AUTO_TRENCH_OFFSET;
    }
    else
    {
        addedOffset = 0.0;
    }
}

bool LimelightCamera::SeesShooterTarget()
{
    if(seesTarget == 1.0)
    {
        return true;
    }
    return false;
}

/*void LimelightCamera::TargetTranslationReset()
{
    for (int i = 0; i < 6; i++)
    {
        targetTransArray[i] = 0;
    }
}

void LimelightCamera::GetTargetTranslation(double *inArray)
{
    for (int i = 0; i < 6; i++)
    {
        inArray[i] = targetTransArray[i];
    }
}*/

void LimelightCamera::TurnOnLED()
{
    table->PutNumber("camMode", 0);
    table->PutNumber("ledMode", 3);
}

void LimelightCamera::TurnOffLED()
{
    table->PutNumber("camMode", 1);
    table->PutNumber("ledMode", 1);
}

void LimelightCamera::TakeSnapshot(bool button)
{
    if(button)
    {
        table->PutNumber("snapshot", 1);
    }
    else
    {
        table->PutNumber("snapshot", 0);
    }
}