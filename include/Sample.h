#ifndef SAMPLE_H_
#define SAMPLE_H_

class TalonXXI;

//#defines
//example: #define LEVEL_ONE_HEIGHT     (23.0)

class Sample
{
    private:
       // Create objects needed by this class
		// example: VictorSP *sampleMotor;

        TalonXXI *mainRobot;
        //add a pointer to sensor class when it is created

        //declare member variables
        //example: float height;


    public:
        Sample(TalonXXI* pRobot);
        //Functions
        void LocalReset(void);
        void StartingConfig(void);
        void StopAll(void);
        void DoAThing(int);
        void UpdateDash(void);
        void Service(void);
        void ControlSystem(void);
};
#endif /*Sample_H_*/