#include <conio.h>
#include <math.h>
#include "motorControl.h"
#include "scanMotorVoltage.h"
#include "utilities.h"
#include <random>


scanMotorVoltage::scanMotorVoltage(motorControl *temp){
    motorsScanVoltage = temp;
    maxMuscleForce = 4;
    forceIncrement = 1;
    isOn_Off=true;
    tick = 0.0;
    frequency =0.0;
    amplitude=0.0;
    offset = 0.0;
    isResetTimer = FALSE;
    sinAmp = 1;
    maxVoltOffset = 10;
    minVoltOffset = 1;
    noiseMin = 0;
    noiseMax = 1;

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
        live = FALSE;
    }
}

int scanMotorVoltage::startSinVoltageScan(){
    voltOffsetResolution = 1;

    if (~isResetTimer)
    {
        timeData.resetTimer();
        isResetTimer = TRUE;
    }
    generateSinusoidAmplitudes();
    generateSinusoidFrequencies();
    int cycle = 0, newTrial = 0, key = 0;
    double sinPeriod=0, paradigm[5] = {0.0};
    for (int i = 0; i < numOfOffsetVoltage; i ++){
        for(int j = 0; j < numOfSinFreq; j++)
        {
            std::cout<<"Offset"<<voltOffset[i]<<" SinFreq:"<<sinFreq[j]<<std::endl<<std::endl<<std::endl;
            newTrial = 1;
            cycle = 0;
            paradigm[0] = 999;
            paradigm[1] = 1;
            paradigm[2] = voltOffset[i];
            paradigm[3] = sinFreq[j];
            paradigm[4] = sinAmp;
            do{
                tick = timeData.getCurrentTime();
                sinPeriod = 1 / sinFreq[j];
                for (int k = 0 ; k < NUMBER_OF_MUSCLES; k++)
                    motorRef[k] = voltOffset[i] + sinAmp * sin (2 * PI * sinFreq[j] * tick);
                motorsScanVoltage->updateMotorRef(motorRef, newTrial, paradigm);
                newTrial = 0;
                Sleep(2);
                if (tick>sinPeriod)
                {
                    cycle ++;
                    timeData.resetTimer();
                }
            }while (cycle<10);
        }
    }
    isResetTimer = FALSE;
    return 1;
}

int scanMotorVoltage::startNoiseVoltageScan(){
    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(noiseMin,noiseMax);
    voltOffsetResolution = 1;

    if (~isResetTimer)
    {
        timeData.resetTimer();
        isResetTimer = TRUE;
    }
    int newTrial = 0, key = 0;
    double paradigm[5] = {0.0}, noise = 0;
    double noiseVoltageOffset[10];
    numOfOffsetVoltage = 10;
    for (int i = 0; i < numOfOffsetVoltage; i ++)
        noiseVoltageOffset[i] = i;
    
    for (int i = 0; i < numOfOffsetVoltage; i ++){
            timeData.resetTimer();
            std::cout<<std::endl<<"Noise Offset"<<noiseVoltageOffset[i]<<std::endl<<std::endl<<std::endl;
            newTrial = 1;
            paradigm[0] = 999;
            paradigm[1] = 2;
            paradigm[2] = noiseVoltageOffset[i];
            paradigm[3] = noiseMin;
            paradigm[4] = noiseMax;
            do{
                tick = timeData.getCurrentTime();
                for (int k = 0 ; k < NUMBER_OF_MUSCLES; k++)
                {
                    noise = distribution(generator);
                    motorRef[k] = noiseVoltageOffset[i] + noise;
                }
                motorsScanVoltage->updateMotorRef(motorRef, newTrial, paradigm);
                newTrial = 0;
                Sleep(5);
            }while (tick<120);
    }
    isResetTimer = FALSE;
    return 1;
}


void scanMotorVoltage::setSinusoidalScan()
{
    flag = SINUSOID;
}

void scanMotorVoltage::setNoiseScan()
{
    flag = NOISE;
}

void scanMotorVoltage::generateSinusoidAmplitudes()
{
    numOfOffsetVoltage = int (maxVoltOffset - minVoltOffset)/voltOffsetResolution;
    for (int i = 0; i <numOfOffsetVoltage; i++)
        voltOffset[i] = minVoltOffset + i * voltOffsetResolution;
}
void scanMotorVoltage::generateSinusoidFrequencies()
{
    //sinFreq[100]
    //minSinFreq, maxSinFreq
    int count = 0;
    for (int i = 1; i<10;i++)
    {
        sinFreq[count] = 0.1 * i;
        count ++;
    }
    for (int i = 1; i<10;i++)
    {
        sinFreq[count] = i;
        count++;
    }
    for (int i = 1; i<10;i++)
    {
        sinFreq[count] = 10 * i;
        count++;
    }
    numOfSinFreq = count;
}

