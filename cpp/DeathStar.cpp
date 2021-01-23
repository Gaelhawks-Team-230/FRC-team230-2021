
#include "TalonXXI_main.h"
#include "Common.h"
#include "DeathStar.h"

DeathStar::DeathStar(TalonXXI* pRobot)
{
    mainRobot = pRobot;
    localSurveillance = pRobot->surveillance;

    starMotor = new frc::VictorSP(PWM_STAR_SPIN_MOTOR);
    kickMotor = new frc::VictorSP(PWM_CELL_KICK_MOTOR);
    kickSolenoid = new frc::DoubleSolenoid(PCM_KICK_CELL_1, PCM_KICK_CELL_2);
    LocalReset();
}

//Sets all local variables
void DeathStar::LocalReset()
{
    loopCount = 0;
    spinSpeed = 0.0;
    cellCount = 0;
   // isSpinning = false;
    ballSettleCount = 0;
    shootWaitCount = 0;
    tolerance = LOADING_TOLERANCE;
    alignOffset = SHOOTING_OFFSET;
    kickSpeed = 0.0;
    kickPos = RETRACTED_KICK;
    deathStarCurrent = 0.0;
    
    currentPos = localSurveillance->GetAngle_DeathStar();
    printf("local reset curpos %f \n", currentPos);
    pcmd = ((int) ((currentPos-ONE_SLOT_ANGLE)/ONE_SLOT_ANGLE))*ONE_SLOT_ANGLE;
    goalPos = ((int) ((currentPos-ONE_SLOT_ANGLE)/ONE_SLOT_ANGLE))*ONE_SLOT_ANGLE;
   // pcmd = currentPos;
    poserr = 0.0;
    mode = IDLE_MODE;

    for(int i=0; i<5; i++)
    {
        hasCell[i] = false;
    }
}

//Holds how the variables should be set when the robot starts
void DeathStar::StartingConfig()
{
    
}

void DeathStar::StopAll()
{
   // isSpinning = false;
    spinSpeed = 0.0;
    kickSpeed = 0.0;
    mode = IDLE_MODE;
    LocalReset();
}

void DeathStar::SpinOneSlot()
{
    goalPos = goalPos - ONE_SLOT_ANGLE;
    kickPos = RETRACTED_KICK;

  //  isSpinning = true;
   // SpinStarService();
}

void DeathStar::KickCell()
{
   /* if(isSpinning)
    {
        kickPos = RETRACTED_KICK;
    }
    else
    {
        //spin wheel before you kick in!
        if(kickSpeed != KICK_SPEED)
        {
            kickSpeed = KICK_SPEED;
        }
        kickPos = EXTENDED_KICK;
    }*/
}

void DeathStar::SetIdle()
{
    mode = IDLE_MODE;
}

void DeathStar::IntakeCells()
{
    mode = INTAKE_MODE;
}

void DeathStar::PrepToShoot()
{
    mode = PREP_MODE;
}

void DeathStar::FireAtWill()
{
    if(IsEmpty())
    {
        mode = IDLE_MODE;
    }
    else
    {
        mode = SHOOT_MODE;
    }
}

void DeathStar::CheckCells()
{
    //Add something to make sure the slots are aligned to use the sensors!
    cellCount = 0;
    for(int slot = 1; slot <=5; slot++)
    {
        if(localSurveillance->GetCellSensor(slot))
        {
            hasCell[slot-1] = true;
            cellCount++;
        }
        else
        {
            hasCell[slot-1] = false;
        }
    }
}


bool DeathStar::IsEmpty()
{
    if(cellCount == 0)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool DeathStar::IsFull()
{
    if(cellCount == 5)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool DeathStar::HasCellAtShootPos()
{
    if(hasCell[SHOOTING_POS])
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool DeathStar::HasCellAtIntakePos()
{
    if(hasCell[INTAKE_POS])
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool DeathStar::IsInPosition()
{
    if(fabs(goalPos - currentPos) <= tolerance)
    {
      //  isSpinning = false;
        return true;
    }
    else
    {
      //  isSpinning = true;
        return false;
    }
}

int DeathStar::GetCellCount()
{
    return cellCount;
}

#ifdef TEST_MODE
void DeathStar::Testing(float axis)
{
    if(fabs(axis) > 0.1)
    {
        //spinSpeed = axis;
        kickSpeed = axis;
    }
    else
    {
        //spinSpeed = 0.0;
        kickSpeed = 0.0;
    }
    spinSpeed = TalonXXI::Limit(-0.3, 0.3, spinSpeed);
}
#endif


//Anything that you want printed on the dashboard for testing or during match. 
//Any information which may be important. 
void DeathStar::UpdateDash()
{
    // Example: 
    //frc::SmartDashboard::PutNumber("Text to label data here:", ShownVariable);
    frc::SmartDashboard::PutNumber("Death Star Curpos: ", currentPos);
    frc::SmartDashboard::PutBoolean("Slot 0 Sensor: ", hasCell[0]);
    frc::SmartDashboard::PutBoolean("Slot 1 Sensor: ", hasCell[1]);
    frc::SmartDashboard::PutBoolean("Slot 2 Sensor: ", hasCell[2]);
    frc::SmartDashboard::PutBoolean("Slot 3 Sensor: ", hasCell[3]);
    frc::SmartDashboard::PutBoolean("Slot 4 Sensor: ", hasCell[4]);
    frc::SmartDashboard::PutNumber("CellCount: ", cellCount);
    GetCurrent();
    frc::SmartDashboard::PutNumber("Death Star Current", deathStarCurrent);
}

//Called every loop (used for timing related stuff)
void DeathStar::Service()
{
    loopCount++;
   // CheckCells();
    currentPos = localSurveillance->GetAngle_DeathStar();
#ifndef TEST_MODE
    if(IsInPosition())
    {
        CheckCells();
        ModeService();
    }
    else
    {
        kickPos = RETRACTED_KICK;
        //SpinStarService();
    }
    
#endif    
   
    poserr = goalPos - currentPos + alignOffset;
    spinSpeed = poserr * DEATH_STAR_POS_ERR_K;
    spinSpeed = TalonXXI::Limit(DEATH_STAR_MIN_CMD, DEATH_STAR_MAX_CMD, spinSpeed);
   //printf("%d %f %f %f \n", loopCount, pcmd, currentPos, spinSpeed);
  // printf("%d %f \n", loopCount, kickSpeed);
  //  printf("%d %f %f \n", loopCount, spinSpeed, currentPos);
    //printf("%d %f %f %f\n", loopCount, spinSpeed, currentPos, localSurveillance->GetDegreesPerSec_DeathStar());    
    starMotor->Set(spinSpeed);
    kickMotor->Set(kickSpeed);
    kickSolenoid->Set(kickPos);
}

void DeathStar::SpinStarService()
{
  /*  kickPos = RETRACTED_KICK;
    //currentPos = localSurveillance->GetAngle_DeathStar();
    if(IsInPosition(tolerance))
    {
        //spinSpeed = 0.0;
        isSpinning = false;
    }
    else
    {
        //spinSpeed = 0.5;
        isSpinning = true;
    }*/
}

void DeathStar::GetCurrent()
{
    deathStarCurrent = mainRobot->pdp->GetCurrent(DEATH_STAR_CURRENT);
}

void DeathStar::ModeService()
{
    int testCell;
    switch(mode)
    {
        case IDLE_MODE:
            alignOffset = SHOOTING_OFFSET;
            spinSpeed = 0.0;
            kickSpeed = 0.0;
            kickPos = RETRACTED_KICK;
            break;

        case INTAKE_MODE:
            alignOffset = INTAKE_OFFSET;
            kickPos = RETRACTED_KICK;
            tolerance = LOADING_TOLERANCE;
            if(IsFull())
            {
                mode = IDLE_MODE;
            }
            else
            {
                if (hasCell[INTAKE_POS])
                {
                    if(ballSettleCount < BALL_SETTLE_DELAY)
                    {
                        ballSettleCount++;
                    }
                    else
                    {
                        ballSettleCount = 0;
                        SpinOneSlot();
                    }
                }
            }
            break;
        case PREP_MODE:
            alignOffset = SHOOTING_OFFSET;
            kickSpeed = KICK_SPEED;
            
            if(!hasCell[SHOOTING_POS]) //Only moves deathstar when no cell at shooter
            {
                if(hasCell[1])
                {
                    SpinOneSlot();
                }
                else if(hasCell[0])
                {
                    SpinOneSlot();
                    SpinOneSlot();
                }
                else if(hasCell[4])
                {
                    SpinOneSlot();
                    SpinOneSlot();
                    SpinOneSlot();
                }
                else if(hasCell[3])
                {
                    SpinOneSlot();
                    SpinOneSlot();
                    SpinOneSlot();
                    SpinOneSlot();
                }
            }
            break;

        case SHOOT_MODE:
            alignOffset = SHOOTING_OFFSET;
            tolerance = SHOOTING_TOLERANCE;
            kickSpeed = KICK_SPEED;
            if(IsEmpty())
            {
                mode = IDLE_MODE;
                break;
            }
            if (hasCell[SHOOTING_POS])
            {
                //Check if shooter is up to speed...
                //THEN shoot it/launch to shooter
                //if(mainRobot->shooter->IsShooterAtGoal() && mainRobot->shooter->IsShroudAtGoal() && mainRobot->turret->IsReadyToShoot())
                //if(true)
                if(mainRobot->shooter->IsShooterAtGoal())
                {
                    kickPos = EXTENDED_KICK;
                    //KickCell();
                }
            }
            else
            {
                kickPos = RETRACTED_KICK;
                SpinOneSlot();
                /*if(shootWaitCount < SHOOT_WAIT_DELAY)
                {
                    shootWaitCount++;
                }
                else
                {
                    shootWaitCount = 0;
                    SpinOneSlot();
                }*/
            }
            break;

        default:
            printf("DeathStar service should not be in here!");
            break;
    }
}

void DeathStar::TestRetractKicker()
{
    kickPos = RETRACTED_KICK;
}

void DeathStar::TestExtendKicker()
{
    kickPos = EXTENDED_KICK;
}

void DeathStar::Increasepcmd()
{
    goalPos = goalPos+72.0;
}
void DeathStar::Decreasepcmd()
{
    goalPos = goalPos-72.0;
}