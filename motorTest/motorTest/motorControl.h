#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#include <NIDAQmx.h>
#include <utilities.h>
#include <iostream>
#include <windows.h>
#include <process.h>
#include <ctime>
#include <timeClass.h>

class motorControl
{
    int errorMotorControl;
    uInt32      dataEnable;
    TaskHandle  motorTaskHandle, motorEnableHandle, loadCelltaskHandle, encodertaskHandle[NUMBER_OF_MUSCLES];
    double I, encoderBias, encoderGain;
    timeClass timeData;
    static void motorControlLoop(void*);
    void controlLoop(void);
    HANDLE hIOMutex;
    bool live, closedLoop;
    double tick,tock;
    float64 encoderData[NUMBER_OF_MUSCLES], loadCellOffset[NUMBER_OF_MUSCLES], windingUpCmnd[NUMBER_OF_MUSCLES];
    float64 loadCellData[NUMBER_OF_MUSCLES], motorRef[NUMBER_OF_MUSCLES], muscleLength[NUMBER_OF_MUSCLES], motorCommand[NUMBER_OF_MUSCLES];
    char header[200], dataSample[600], fileName[200];;
    int controlLoopParadigm;
    int createHeader4DataFile();
    int initializeTaskHandles();
    int createDataEnable();
    int createWindingUpCommand();
    int createPortNumber(int,int,char *,char *);
    int createFileName();
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
    void updateMotorRef(float64 *);
    void setDataAcquisitionFlag(bool *);
    void setOpenLoop();
};
#endif
