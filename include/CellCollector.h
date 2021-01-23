#ifndef CELL_COLLECTOR_H_
#define CELL_COLLECTOR_H_

#include "Common.h"
#include "TalonXXI_main.h"
#include "SensorState.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "frc/VictorSP.h"
#include "frc/DoubleSolenoid.h"

/*
* CellCollector.h
*
* Created on: Jan 11, 2020
*        Author: Sanjana Jain
*/

class CellCollector;

//#defines
//example: #define LEVEL_ONE_HEIGHT     (23.0)

#define GRAB_CELLS_CMD (1.0)
#define RELEASE_CELLS_CMD (-1.0)

#define GRABBER_IN     (frc::DoubleSolenoid::Value::kReverse)
#define GRABBER_OUT    (frc::DoubleSolenoid::Value::kForward)

#define IDLE_MODE        (0)
#define GATHERING_MODE   (1)
#define EJECTING_MODE    (2)
#define UNJAM_MODE       (3)
#define DELAY_MODE       (4)

//#define CLIMBER_CURRENT_IN (0) //placeholder value

#define TARGET_STALL_COUNT  ((int)(N1SEC * 0.2))//It will take 0.5 seconds to do one rotation
#define GRABBER_STALL_CURRENT (20.0) //placeholder value

#define EJECT_SPIN_TIMER     ((int)(0.5 * (N1SEC))) //It will take 0.75 seconds to do one rotation
#define DELAY_OFF_TIME       ((int)(1.0*N1SEC))

class CellCollector
{
    private:
       // Create objects needed by this class
		// example: VictorSP *sampleMotor;
        TalonXXI *mainRobot;
        SensorState *localSurveillance;
        frc::VictorSP *grabber;
        frc::DoubleSolenoid *cellPiston; //the air pressure to the piston to shoot the grabber out
        
        bool isGrabbingCells;   //the motor is spinning to grab power cells
        double grabberCommand;  //number that represents the angle that the grabber is at when up or down.
        frc::DoubleSolenoid::Value cellPistonCMD;     //true and false of whether the cell piston is out or in.
        bool grabberIsIn;       //true if grabber is in, false if grabber is out.
        int operationalMode;    // IDLE, GATHERING, EJECTING, UNJAM
        double grabberMotorCurrent; //
        int stallCurrentCount;
        int ejectCellCount; 
        int grabOffDelay;

    public:
        CellCollector(TalonXXI* pRobot);

        //Functions
        void LocalReset();
        void StartingConfig();
        void StopAll();
        void GrabCells();
        void StopGrab();
        void EjectCells();
        void GathererIn();
        void GathererOut();
        void UpdateDash();
        void Service();
        void TestingSpin(double);

    private:
        void GetCurrent();
};
#endif /*CELL_COLLECTOR_H_*/