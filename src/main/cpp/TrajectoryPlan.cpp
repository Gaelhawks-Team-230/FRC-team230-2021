#include "TalonXXI_main.h"
#include "Common.h"
#include "TrajectoryPlan.h"
using namespace std;

TrajectoryPlan::TrajectoryPlan(TalonXXI* pRobot)
{
    mainRobot = pRobot;
    localSurveillance = pRobot->surveillance;
    // frc::filesystem::GetDeployDirectory(path);
    LocalReset();
}

//Sets all local variables
void TrajectoryPlan::LocalReset()
{
    plan.clear();
}

//Holds how the variables should be set when the robot starts
void TrajectoryPlan::StartingConfig()
{

}


void TrajectoryPlan::StopAll()
{

}

void TrajectoryPlan::Tokenize(std::string const &str, const char delim, std::vector<double> &out)
{
    size_t start;
    size_t end = 0;
    while ((start = str.find_first_not_of(delim, end)) != string::npos)
    {
        end = str.find(delim, start);
        string temp= str.substr(start, end - start);
        out.push_back(stod(temp));
    }
}

bool TrajectoryPlan::ReadPath(int autoModeNum)
{
    string line;
    string directory;
    const char delim = ',';
    // FILE *pFile;
    // pFile = fopen("/home/lvuser/deploy/example.txt","r");
    plan.clear(); //clear vector here before it is read because vector will add to self otherwise
    printf("Case: %d\n", autoModeNum);
    switch (autoModeNum)
    {
        case 9:
            directory = "/home/lvuser/deploy/Test.txt";
            break;
        case 10:
            directory = "/home/lvuser/deploy/BarrArr.txt";
            break;
        case 11:
            directory = "/home/lvuser/deploy/SlalArr.txt";
            break;
        case 12:
            directory = "/home/lvuser/deploy/BounArr.txt";
            break;
    }
    ifstream myfile (directory);
    printf("file seen\n");
    if (myfile.is_open())
    {
        printf ("File Open\n");
        while(!myfile.eof())
        {
            getline(myfile, line);
            vector<double> out;
            Tokenize(line, delim, out);
            plan.push_back(out);
        }
        myfile.close();
        return true;
    }
    printf("Vector Size: %d\n", plan.size());
    return false;
    
}

void TrajectoryPlan::PrintPath()
{
    for(int i= 0; i< plan.size(); i++)
    {
        printf("%f %f\n", plan[i][0], plan[i][1]);
    }
}

bool TrajectoryPlan::IsPathComplete(int loopCount)
{
    if (loopCount < plan.size())
    {
        return false;
    }
    return true;
}

std::vector<double> TrajectoryPlan::GetCurrentCmd(int loopCount)
{
    //printf("%d %f %f\n", loopCount, plan[loopCount][1], frc::GetTime());
    return plan[loopCount];
}


//Anything that you want printed on the dashboard for testing or during match. 
//Any information which may be important. 
void TrajectoryPlan::UpdateDash()
{
    // Example: 
    //frc::SmartDashboard::PutNumber("Text to label data here:", ShownVariable);
}

//Called every loop (used for timing related stuff)
void TrajectoryPlan::Service()
{
    
}
