
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
    shootingStage = 0;
    shootDelayCount = 0;
    kickDurationCount = 0;
   // isSpinning = false;
    ballSettleCount = 0;
    tolerance = LOADING_TOLERANCE;
    alignOffset = SHOOTING_OFFSET;
    kickSpeed = 0.0;
    kickPos = RETRACTED_KICK;
    deathStarCurrent = 0.0;

    isJammed = false;
    unjamStage = 0;
    stallCount = 0;
    unjamCount = 0;
    
    currentPos = localSurveillance->GetAngle_DeathStar();
    goalPos = ((int) ((currentPos-ONE_SLOT_ANGLE)/ONE_SLOT_ANGLE))*ONE_SLOT_ANGLE;
    posCmd = goalPos;
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
    kickPos = RETRACTED_KICK;
    Decreasepcmd();
}

void DeathStar::StopAll()
{
    LocalReset();
}

void DeathStar::SpinOneSlot()
{
    goalPos = goalPos - ONE_SLOT_ANGLE;
    kickPos = RETRACTED_KICK;
}

void DeathStar::SetIdle()
{
    if(mode != IDLE_MODE)
    {
        mode = IDLE_MODE;
    }
}

void DeathStar::IntakeCells()
{
    if(mode != INTAKE_MODE)
    {
        mode = INTAKE_MODE;
    }
}

void DeathStar::PrepToShoot()
{
    if(mode != PREP_MODE)
    {
        mode = PREP_MODE;
    }
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

void DeathStar::UpdateDash()
{
    //frc::SmartDashboard::PutNumber("Death Star GoalPos: ", goalPos);
    //frc::SmartDashboard::PutNumber("Death Star Curpos: ", currentPos);
    frc::SmartDashboard::PutBoolean("Slot 0 Sensor: ", hasCell[0]);
    frc::SmartDashboard::PutBoolean("Slot 1 Sensor: ", hasCell[1]);
    frc::SmartDashboard::PutBoolean("Slot 2 Sensor: ", hasCell[2]);
    frc::SmartDashboard::PutBoolean("Slot 3 Sensor: ", hasCell[3]);
    frc::SmartDashboard::PutBoolean("Slot 4 Sensor: ", hasCell[4]);
    //frc::SmartDashboard::PutNumber("CellCount: ", cellCount);
  //  frc::SmartDashboard::PutNumber("Shooting Stage", shootingStage);
  //  frc::SmartDashboard::PutBoolean("Is Kick Extended", (kickPos == EXTENDED_KICK));
  //  StallCheck();
    //frc::SmartDashboard::PutNumber("Death Star Current", deathStarCurrent);
}

void DeathStar::Service()
{
    loopCount++;
#ifdef TEST_MODE
    CheckCells();
#endif
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
    }

    StallCheck();
    if(!isJammed)
    {
        posCmd = posCmd + TalonXXI::Limit(-DEATH_STAR_PLANNER_VEL, DEATH_STAR_PLANNER_VEL, goalPos - posCmd);
        //poserr = goalPos - currentPos + alignOffset;
        poserr = posCmd - currentPos + alignOffset;
        spinSpeed = poserr * DEATH_STAR_POS_ERR_K;
        spinSpeed = TalonXXI::Limit(DEATH_STAR_MIN_CMD, DEATH_STAR_MAX_CMD, spinSpeed);
    }
 #endif    
   // printf("%f ", currentPos);
    starMotor->Set(spinSpeed);
    kickMotor->Set(kickSpeed);
    kickSolenoid->Set(kickPos);
}

void DeathStar::StallCheck()
{
    deathStarCurrent = mainRobot->pdp->GetCurrent(DEATH_STAR_CURRENT);
    switch(unjamStage)
    {
        case 0:
            if(deathStarCurrent > DEATH_STAR_STALL_CURRENT)
            {
                stallCount++;
            }
            else
            {
                stallCount = 0;
            }
            if(stallCount > DEATH_STAR_STALL_COUNT)
            {
                isJammed = true;
                stallCount = 0;
                unjamCount = 0;
                unjamStage++;
            }
            break;

        case 1:
            if(unjamCount < DEATH_STAR_UNJAM_COUNT)
            {
                spinSpeed = DEATH_STAR_UNJAM_CMD;
                unjamCount++;
            }
            else
            {
                unjamCount = 0;
                isJammed = false;
                unjamStage = 0;
                stallCount = 0;
            }
            break;
    }
    

}

void DeathStar::ModeService()
{
    switch(mode)
    {
        case IDLE_MODE:
            alignOffset = SHOOTING_OFFSET;
            spinSpeed = 0.0;
            kickSpeed = 0.0;
            kickPos = RETRACTED_KICK;
            shootDelayCount = 0;
            kickDurationCount = 0;
            shootingStage=0;


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
            if(IsEmpty() && shootingStage == 5)
            {
                mode = IDLE_MODE;
                break;
            }
            ShootingService();
            
            break;

        default:
            printf("DeathStar service should not be in here!");
            break;
    }
}

void DeathStar::ShootingService()
{
    switch(shootingStage)
    {
        case 0:
            kickPos = RETRACTED_KICK;
            shootDelayCount = 0;
            kickDurationCount = 0;
            shootingStage++;
            break;

        case 1:
            if(hasCell[SHOOTING_POS])
            {
                shootingStage++;
                shootDelayCount = 0;
            }
            else
            {
                shootingStage = 5;
            }
            break;

        case 2:
            if(shootDelayCount < SHOOT_WAIT_DELAY)
            {
                shootDelayCount++;
            }
            else
            {
                shootDelayCount = 0;
                kickDurationCount = 0;
                shootingStage++;
            }
            break;

        case 3:
            if(mainRobot->shooter->IsShooterAtGoal())
            {
                kickPos = EXTENDED_KICK;
                shootingStage++;
                kickDurationCount = 0;
            }
            break;

        case 4:
            if(kickDurationCount < KICK_HOLD_DELAY)
            {
                kickDurationCount++;
            }
            else
            {
                kickDurationCount = 0;
                kickPos = RETRACTED_KICK;
                shootDelayCount = 0;
                shootingStage++;
            }
            break;

        case 5:
            if(!hasCell[SHOOTING_POS])
            {
                SpinOneSlot();
                shootDelayCount = 0;
                kickDurationCount = 0;
                shootingStage = 0;
            }
            break;
    }

   // printf("%d %d %d \n", loopCount, shootDelayCount, kickDurationCount);
    
}

void DeathStar::Increasepcmd()
{
    goalPos = goalPos + ONE_SLOT_ANGLE;
}
void DeathStar::Decreasepcmd()
{
    goalPos = goalPos - ONE_SLOT_ANGLE;
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
        return true;
    }
    else
    {
        return false;
    }
}

int DeathStar::GetCellCount()
{
    return cellCount;
}

#ifdef TEST_MODE
void DeathStar::TestRetractKicker()
{
    kickPos = RETRACTED_KICK;
}

void DeathStar::TestExtendKicker()
{
    kickPos = EXTENDED_KICK;
}

void DeathStar::TestingDeathStar(double axis)
{
    if(fabs(axis) > 0.1)
    {
        spinSpeed = axis;
    }
    else
    {
        spinSpeed = 0.0;
    }
    spinSpeed = TalonXXI::Limit(-0.3, 0.3, spinSpeed);
   // printf("%f ", spinSpeed);
}

void DeathStar::TestingKickMotor(double axis)
{
    if(fabs(axis) > 0.1)
    {
        kickSpeed = axis;
    }
    else
    {
        kickSpeed = 0.0;
    }
    //printf("%f \n", kickSpeed);    
}
#endif
