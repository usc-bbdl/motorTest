#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#include <NIDAQmx.h>
#include <utilities.h>
#include <iostream>
#include <windows.h>
#include <process.h>
#include <ctime>
#include <timeClass.h>

#define LAG 1
#define INTEGRAL 2
#define OPTIMAL_GAIN 3
#define PI 4

class motorControl
{
    int errorMotorControl, newTrial, controlLaw;
    uInt32      dataEnable;
    TaskHandle  motorTaskHandle, motorEnableHandle, loadCelltaskHandle, encodertaskHandle[NUMBER_OF_MUSCLES];
    double I, encoderBias, encoderGain, paradigm[5], experimentControl, optimalGain, b0, b1, b2, a1, a2, ki;
    timeClass timeData;
    static void motorControlLoop(void*);
    void controlLoop(void);
    HANDLE hIOMutex;
    bool live, closedLoop;
    double tick,tock;
    float64 encoderData[NUMBER_OF_MUSCLES], loadCellOffset[NUMBER_OF_MUSCLES], windingUpCmnd[NUMBER_OF_MUSCLES];
    float64 loadCellData[NUMBER_OF_MUSCLES], motorRef[NUMBER_OF_MUSCLES], muscleLength[NUMBER_OF_MUSCLES], motorCommand[NUMBER_OF_MUSCLES];
    char header[200], dataSample[600], fileName[200];
    int controlLoopParadigm;
    int createHeader4DataFile();
    int initializeTaskHandles();
    int createDataEnable();
    int createWindingUpCommand();
    int createPortNumber(int,int,char *,char *);
    int createFileName();
    void customizedControllerLaw(int muscleIndex);
    int scaleMuscleLengthData(float64 *);
    int scaleloadCellData(float64 *);
    void createDataSampleString();
    int dataAcquisitionFlag[4];
public:
    motorControl();
    ~motorControl(void);
    bool isEnable, isWindUp, isControlling;
    int motorEnable();
    int motorWindUp();
    int motorDisable();
    int motorControllerStart();
    int motorControllerEnd();
    double getTime();
    void updateMotorRef(float64 *, int , double *);
    void setDataAcquisitionFlag(bool *);
    void setOpenLoop();
    void setControlLaw(int controlLaw);
};
#endif
