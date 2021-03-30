#include "TalonXXI_main.h"
#include "Common.h"
#include "TrajectoryPlanner.h"

#ifndef CALIBRATION

TrajectoryPlanner::TrajectoryPlanner(TalonXXI* pRobot)
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

double TrajectoryPlanner::Limit(double cmd, double loLim, double hiLim)
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

double TrajectoryPlanner::GetZeta(void)
{
	return zeta;
}


//sets various constants that are only set once
void TrajectoryPlanner::SetZeta(double newZeta)
{
	zeta = newZeta;
	w_n = 4*zeta*(A_LIM/V_LIM);
	k1 = w_n*w_n;
	k2 = 2*zeta*w_n;	//orig: k2 = 2*1.4*w_n
}

double TrajectoryPlanner::GetPositionCmd(void)
{
	return pCmd;
}

void TrajectoryPlanner::SetPositionCmd(double newPositionCmd)
{
	pCmd = newPositionCmd;
	printf("Position Cmd: %f\n", pCmd);
}

//determines target position, velocity,  and acceleration
void TrajectoryPlanner::Calculate(double *position, double *velocity, double *acceleration)
{
	//calculates target acceleration
	a = k1*(pCmd - p) - k2*v;
	a = Limit(a, -A_LIM, A_LIM);

	//calculates target velocity
	v = v + a*dt;
	v = Limit(v, -V_LIM, V_LIM);
	*velocity = v;

	//calculates target position
	p = p + v*dt;
	*position = p;

	//printf("Target Position: %f\n", position);

	//corrects acceleration for being one loopcount behind
	a = k1*(pCmd - p - v*dt) - k2*v;
	*acceleration = a;
}


void TrajectoryPlanner::Abort(void)
{
	a = 0.0;
	v = 0.0;
	p = 0.0;
	pCmd = 0.0;
}
void TrajectoryPlanner::ForcedReset(double updatedPos)
{
	a = 0.0;
	v = 0.0;
	p = updatedPos;
	pCmd = updatedPos;
}
#endif
