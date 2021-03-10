#include "Sample.h"

Sample::Sample(TalonXXI* pRobot)
{
    mainRobot = pRobot;
    //add a sensor class object here when that class is created
    
    LocalReset();
}

//Sets all local variables
void Sample::LocalReset()
{
}

//Holds how the variables should be set when the robot starts
void Sample::StartingConfig()
{

}


void Sample::StopAll()
{

}

//Just an example for creating a function that takes input
void Sample::DoAThing(int a)
{

}

//Anything that you want printed on the dashboard for testing or during match. 
//Any information which may be important. 
void Sample::UpdateDash()
{
    // Example: 
    //frc::SmartDashboard::PutNumber("Text to label data here:", ShownVariable);
}

//Called every loop (used for timing related stuff)
void Sample::Service()
{
    
}
