/*#ifndef TRAJECTORYPLANNER_H_
#define TRAJECTORYPLANNER_H_

#include "TalonXXI_main.h"
#include "Common.h"
#include "SensorState.h"

class TalonXXI;
class SensorState;

#define TEST_ZETA 	(1.4)	//test value
#define A_LIM 		(100.0) //(125.0) // (50.0)	//maximum possible acceleration
#define V_LIM 		(100.0) //(125.0) // (50.0)	//maximum possible velocity

class TrajectoryPlanner 
{
private:
    TalonXXI *mainRobot;
    SensorState *localSurveillance;
	double pCmd;	//position command
	double a, v, p;	//current values of acceleration, velocity, position

	//constants that don't change after definition, defined by SetZeta()
	double zeta;
	double w_n;
	double k1;
	double k2;
	double dt;

public:
	TrajectoryPlanner(void);
	void Abort(void);

	double Limit(double cmd, double loLim, double hiLim);

	//determines target position, velocity,  and acceleration
	void Calculate(double *position, double *velocity, double *acceleration);

	double GetPositionCmd(void);
	void SetPositionCmd(double newPositionCmd);	//set position command that is the target distance
	double GetZeta(void);
	void SetZeta(double newZeta);	//set zeta and all other constants
	void ForcedReset(double updatedPos);
    
};

#endif /* TRAJECTORYPLANNER_H_ */
