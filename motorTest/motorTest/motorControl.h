#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#include <NIDAQmx.h>
#include <utilities.h>
#include <iostream>
#include <windows.h>
#include <process.h>
#include <ctime>

class motorControl
{
    TaskHandle  motorTaskHandle, motorEnableHandle, loadCelltaskHandle, encodertaskHandle;
    TaskHandle  encodertaskHandle[3];
    double loadCellOffset1, loadCellOffset2, loadCellOffset3,I;
    TimeData timeData;
    static void motorControlLoop(void*);
    void controlLoop(void);
    HANDLE hIOMutex;
    bool live;
    float64 encoderData1[1],encoderData2[1],encoderData3[1],muscleLengthPreviousTick[2], muscleLengthOffset[3];
    char header[200];
public:    
    bool resetMuscleLength;
    float64 loadCellData[4],motorRef[3],muscleLength[3],muscleVel[2];
    double cortexVoluntaryAmp, cortexVoluntaryFreq;
    unsigned int muscleSpikeCount[2],raster_MN_1[2],raster_MN_2[2],raster_MN_3[2],raster_MN_4[2],raster_MN_5[2],raster_MN_6[2];
    float muscleEMG[2],spindleIa[2], spindleII[2],encoderBias[3],encoderGain[3];
    motorControl(double,double,double);
    ~motorControl(void);
    bool isEnable, isWindUp, isControlling;
    int motorEnable();
    int motorWindUp();
    int motorDisable();
    int motorControllerStart();
    int motorControllerEnd();
    int gammaDynamic1, gammaStatic1,gammaDynamic2, gammaStatic2,trialTrigger;
    double cortexDrive[2], angle, velocity;
    bool newPdgm_Flag;
    double newPdgm_ref[2];

    double getTime();
    void dummy();
};

#endif
