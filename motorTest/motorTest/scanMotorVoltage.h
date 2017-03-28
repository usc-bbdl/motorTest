#ifndef SCANFORCE_H
#define SCANFORCE_H

#include <windows.h>
#include <process.h>
#include "utilities.h"
#include "timeClass.h"
#include "motorControl.h"

class scanMotorVoltage
{
     timeClass timeData;
     double tick,amplitude[NUMBER_OF_MUSCLES];
     double phaseShift[NUMBER_OF_MUSCLES],offset[NUMBER_OF_MUSCLES],frequency[NUMBER_OF_MUSCLES];
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
public:
     scanMotorVoltage(motorControl*);
     ~scanMotorVoltage(void);
     void startScan();
     int startSinVoltageScan();
     int startNoiseVoltageScan();
     bool isOn_Off;
     void setSinValues(float64 arg[]);
};

#endif