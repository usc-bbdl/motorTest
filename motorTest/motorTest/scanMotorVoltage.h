#ifndef SCANFORCE_H
#define SCANFORCE_H

#include <windows.h>
#include <process.h>
#include "utilities.h"
#include "timeClass.h"
#include "motorControl.h"
#include <NIDAQmx.h>
#define NOISE 0
#define SINUSOID 1
class scanMotorVoltage
{
     timeClass timeData;
     double tick;
     //double phaseShift[NUMBER_OF_MUSCLES],offset[NUMBER_OF_MUSCLES],frequency[NUMBER_OF_MUSCLES],amplitude[NUMBER_OF_MUSCLES];
     motorControl *motorsScanVoltage;
     HANDLE hIOMutex;
     bool live,isResetTimer;
     static void scanVoltageControlLoop(void*);
     void controlLoop();
     void update();
     float maxMuscleForce;
     float forceIncrement;
     float64 motorRef[NUMBER_OF_MUSCLES];
     float64 frequency, amplitude, offset;
     int flag;
     int numOfSinFreq, numOfOffsetVoltage;
     double sinFreq[100], voltOffset[100], minSinFreq, maxSinFreq, minVoltOffset, maxVoltOffset, sinAmp, voltOffsetResolution, sinFreqResolution, noiseMin, noiseMax;

public:
     scanMotorVoltage(motorControl *);
     ~scanMotorVoltage(void);
     void startScan();
     int startSinVoltageScan();
     int startNoiseVoltageScan();
     bool isOn_Off;
     void setSinValues(float64 arg[]);
     void setSinusoidalScan();
     void setNoiseScan();
     void generateSinusoidFrequencies();
     void generateSinusoidAmplitudes();
};

#endif