#include "generalExperimentalParadigm.h"
#include "customizedExperimentalParadigm.h"
#include <iostream>
generalExperimentalParadigm::generalExperimentalParadigm()
{
    customizedExperiment.initialize(& motors);
    printf("Enabling motors\n");
    motorState = STATE_PRE_INIT;
    motorError = motors.motorEnable();
    if (motorError == 0)
    {
        motorState = STATE_POST_INIT;
        printf("Motors are Enabled; Next stage is Winding Up\n");
    }
    else
    {
        motorState = STATE_PRE_SHUTDOWN;
        printf("There was an error enabling the motors; Next stage is Shutting Down\n");
    }
    
}
generalExperimentalParadigm::~generalExperimentalParadigm(void)
{
}
void generalExperimentalParadigm::nextState()
{
   switch(motorState)
   {
   case STATE_POST_INIT:
        motorState = STATE_PRE_WINDING_UP;
        printf("Motors Winding Up\n");
        motorError = motors.motorWindUp(); 

        if (motorError == 0)
        {
            motorState = STATE_POST_WINDING_UP;
            printf("Motors are winded; Next stage is Closed Loop\n");
        }
        else
        {
            motorState = STATE_PRE_SHUTDOWN;
            printf("There was an error winding the motors; Next stage is Shutting Down\n");
        }
        break;
    case STATE_POST_WINDING_UP:
        //Start data acquision
        motorState = STATE_PRE_DATA_ACQUISITION;
        printf("Starting Data Acquisition\n");
        //motors.setOpenLoop();//Comment out this line for closed-loop control
        motorError = motors.motorControllerStart();
        if (motorError == 0)
        {
            Sleep(1000);
            motorState = STATE_POST_DATA_ACQUISITION;
            printf("Data Acquisition has begun; Next stage is Custimized Experimental Paradigm\n");
        }
        else
        {
            motorState = STATE_PRE_SHUTDOWN;
            printf("There was an error activating Data Acquisition module; Next stage will shut down system\n");
        }
        break;
    case STATE_POST_DATA_ACQUISITION:
        motorState = STATE_PRE_CUSTOMIZED_PARADIGM;
        printf("Starting Customized Experimenal Paradigm\n");
        motorError = customizedExperiment.runExperiment();
        if (motorError == 0)
        {
            Sleep(1000);
            motorState = STATE_POST_CUSTOMIZED_PARADIGM;
            printf("Customized Experiment is finished; Next stage is shutting down\n");
        }
        else
        {
            motorState = STATE_PRE_SHUTDOWN;
            printf("Customized Experiment failed; Next stage will shut down the system\n");
        }
    break;
    case STATE_PRE_SHUTDOWN:
        motorState = STATE_POST_SHUTDOWN;
        printf("Shutting Down\n");
        motors.motorControllerEnd();
        printf("Press Enter to Exit\n");
        exit(0);
    break;
        default: break;
        }
}
void generalExperimentalParadigm::endExperiment()
{
    motorState = STATE_PRE_SHUTDOWN;
    nextState();
}