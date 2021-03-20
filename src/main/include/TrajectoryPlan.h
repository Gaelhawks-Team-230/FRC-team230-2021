#ifndef TRAJECTORYPLAN_H_
#define TRAJECTORYPLAN_H_
// #include <frc/WPILib.h>
#include "frc/Filesystem.h"
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <stdio.h>
class TalonXXI;

//#defines
//example: #define LEVEL_ONE_HEIGHT     (23.0)

class TrajectoryPlan
{
    private:
        TalonXXI *mainRobot;
        SensorState *localSurveillance;
        std::vector<std::vector<double>> plan;
        // wpi::SmallVectorImpl<char> path;

    public:
        TrajectoryPlan(TalonXXI* pRobot);
        //Functions
        void LocalReset(void);
        void StartingConfig(void);
        void StopAll(void);
        void Tokenize(std::string const &str, const char delim, std::vector<double> &out);
        bool ReadPath(int);
        bool IsPathComplete(int);
        std::vector<double> GetCurrentCmd(int);
        void PrintPath(void);
        void UpdateDash(void);
        void Service(void);
        void ControlSystem(void);
};
#endif /*TrajectoryPlan_H_*/