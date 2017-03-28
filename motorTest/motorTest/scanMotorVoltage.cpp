#include <conio.h>
#include <math.h>
#include "motorControl.h"
#include "scanMotorVoltage.h"
#include "utilities.h"


scanMotorVoltage::scanMotorVoltage(motorControl *temp){
    motorsScanVoltage = temp;
    maxMuscleForce = 4;
    forceIncrement = 1;
    isOn_Off=true;
    tick = 0.0;

    frequency =0.0;
    amplitude=0.0;
    offset = 0.0;
    phaseShift[0] = 0;
    phaseShift[1] = 2.099;//2.099;
    phaseShift[2] = 4.189;//4.189;
    //frequency[0] = 1;
    //frequency[1] = 1;
    //frequency[2] = 1;
    //amplitude[0] = 3.5;
    //amplitude[1] = 2.5;
    //amplitude[2] = 5;
    //offset[0] = .5;
    //offset[1] = .5;
    //offset[2] = .5;
    isResetTimer = FALSE;
}


scanMotorVoltage::~scanMotorVoltage(void){
    live=FALSE;
}

void scanMotorVoltage::setSinValues(float64 sinValues[])
{
    frequency = sinValues[0];
    amplitude = sinValues[1];
    offset =    sinValues[3];
    flag =      sinValues[4];
}


void scanMotorVoltage::startScan()
{
    live = TRUE;
    hIOMutex = CreateMutex(NULL, FALSE, NULL);
	_beginthread(scanMotorVoltage::scanVoltageControlLoop,0,this);
}

void scanMotorVoltage::scanVoltageControlLoop(void* a)
{
	((scanMotorVoltage*)a)->controlLoop();
}

void scanMotorVoltage::controlLoop(void){
    int key = 0;
    while (live)
    {
        if (kbhit()!=0){
            key = getch();
            if(key == 27) live = FALSE;
        }
        if(flag)
            startSinVoltageScan();
        else
            startNoiseVoltageScan();
    }
}

int scanMotorVoltage::startSinVoltageScan(){
    if (~isResetTimer)
    {
        timeData.resetTimer();
        isResetTimer = TRUE;
    }
    for (int i = 1; i <=3000; i++){
        tick = timeData.getCurrentTime();
        for(int j = 0; j<NUMBER_OF_MUSCLES;j++)
            motorRef[i] = offset + (amplitude * sin (2*3.14*frequency*tick+ phaseShift[i]) + amplitude) / 2;
        
        motorsScanVoltage->updateMotorRef(motorRef);
        //motorsScanVoltage->newCommand = 1;
        Sleep(10);
    }
    return 1;
}

int scanMotorVoltage::startNoiseVoltageScan(){

    return 1;
}


