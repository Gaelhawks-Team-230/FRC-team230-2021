/*#include "TalonXXI_main.h"
#include "Common.h"
#include "TrajectoryPlanner.h"

#ifndef CALIBRATION

TrajectoryPlannerYaw::TrajectoryPlannerYaw(void)
{
    mainRobot = pRobot;
    localSurveillance = pRobot->surveillance;
    
	a = 0.0;
	v = 0.0;
	p = 0.0;
	pCmd = 0.0;
	SetZeta(TEST_ZETA);	//should not be changed later
	dt = LOOPTIME;	//also should not be changed later
}

double TrajectoryPlannerYaw::Limit(double cmd, double loLim, double hiLim)
{
	if (cmd < loLim)
	{
		return loLim;
	}
	else if (cmd > hiLim)
	{
		return hiLim;
	}
	return cmd;
}

double TrajectoryPlannerYaw::GetZeta(void)
{
	return zeta;
}


//sets various constants that are only set once
void TrajectoryPlannerYaw::SetZeta(double newZeta)
{
	zeta = newZeta;
	w_n = 4*zeta*(A_LIM_YAW/V_LIM_YAW);
	k1 = w_n*w_n;
	k2 = 2*zeta*w_n;	//orig: k2 = 2*1.4*w_n
}

double TrajectoryPlannerYaw::GetPositionCmd(void)
{
	return pCmd;
}

void TrajectoryPlannerYaw::SetPositionCmd(double newPositionCmd)
{
	pCmd = newPositionCmd;
	printf("Position Cmd: %f\n", pCmd);
}

//determines target position, velocity,  and acceleration
void TrajectoryPlannerYaw::Calculate(double *position, double *velocity, double *acceleration)
{
	//calculates target acceleration
	a = k1*(pCmd - p) - k2*v;
	a = Limit(a, -A_LIM_YAW, A_LIM_YAW);

	//calculates target velocity
	v = v + a*dt;
	v = Limit(v, -V_LIM_YAW, V_LIM_YAW);
	*velocity = v;

	//calculates target position
	p = p + v*dt;
	*position = p;

	//printf("Target Position: %f\n", position);

	//corrects acceleration for being one loopcount behind
	a = k1*(pCmd - p - v*dt) - k2*v;
	*acceleration = a;
}


void TrajectoryPlannerYaw::Abort(void)
{
	a = 0.0;
	v = 0.0;
	p = 0.0;
	pCmd = 0.0;
}

void TrajectoryPlannerYaw::ForcedReset(double updatedPos)
{
	a = 0.0;
	v = 0.0;
	p = updatedPos;
	pCmd = updatedPos;
}
#endif
*/