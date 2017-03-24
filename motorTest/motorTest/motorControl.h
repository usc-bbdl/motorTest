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
    bool live;
    float64 encoderData[NUMBER_OF_MUSCLES], loadCellOffset[NUMBER_OF_MUSCLES], windingUpCmnd[NUMBER_OF_MUSCLES];
    char header[200];
    int controlLoopParadigm;
    int createHeader4DataFile();
    int initializeTaskHandles();
    int createDataEnable();
    int createWindingUpCommand();
    int createPortNumber(int,int,char *,char *);
    int createFileName(char *,tm *);
    int scaleMuscleLengthData(float64 *);
    int scaleloadCellData(float64 *);
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
};
#endif
