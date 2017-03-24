#pragma once
#ifndef UTILITIES_H
#define UTILITIES_H
#include <math.h>
#include <NIDAQmx.h>
#include <stdio.h>
#define TRUE  1
#define FALSE 0

#define DAQmxErrChk(functionCall) if( DAQmxFailed(error=(functionCall)) ) goto Error; else

const int samplingFrequency = 1000;
const double PI = 3.14159265358979323846;
const double shaftRadius = 0.003;// motor shaft radius in cm
const double loadCellScale[7] = {(1/sqrt(2.0)) * 50.53}; //From calibration test with weights

const int sampleFreq = 50000;
const double motorMinVoltage = -7;
const double motorMaxVoltage = 7;
const int messageMinVoltage = 0;
const double messageMaxVoltage = 0.03;
const double loadCellMinVoltage = -10;
const double loadCellMaxVoltage = +10;
const int encoderPulsesPerRev = 500;

#define     STATE_INIT 0
#define     STATE_WINDING_UP 1
#define     STATE_OPEN_LOOP 2
#define     STATE_CLOSED_LOOP 3
#define     STATE_PARADIGM_LENGTH_CALIBRATION 4
#define     STATE_RUN_PARADIGM_SERVO_PERTURBATION 5
#define     STATE_RUN_PARADIGM_MANUAL_PERTURBATION 6
#define     STATE_RUN_PARADIGM_VOLUNTARY_MOVEMENT 7
#define     STATE_SHUTTING_DOWN 8
#define     STATE_RUN_PARADIGM_CDMRP_IMPLANT 9

extern int dataAcquisitionFlag[12];
int proceedState(int *);


typedef unsigned char       BYTE;
#endif
