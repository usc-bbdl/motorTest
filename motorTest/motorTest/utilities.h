#pragma once
#ifndef UTILITIES_H
#define UTILITIES_H
#include <math.h>
#include <NIDAQmx.h>
#include <stdio.h>


//Physical channel number
const int loadCellSlotNumber = 5;
const int motorAnalogOutSlotNumber = 2;
const int motorEnableSlotNumber = 2;
const int motorEncoderSlotNumber = 3;
const int motorEnableChannelNumber = 0;
const int loadCellChannelNumber[7] = {0, 8, 1, 9, 2, 10, 11};
const int motorAnalogOutChannelNumber[7] = {8, 9, 10, 11, 12, 13, 14};
const int motorEncoderChannelNumber[7] = {0, 1, 2, 3, 4, 5 , 6};

#define DAQmxErrChk(functionCall) if( DAQmxFailed(error=(functionCall)) ) goto Error; else

//constant numbers
const int samplingFrequency = 1000;
const double PI = 3.14159265358979323846;


//constant numbers for hardware
const double motorMinVoltage = -7;
const double motorMaxVoltage = 7;
const int messageMinVoltage = 0;
const double messageMaxVoltage = 0.03;
const double loadCellMinVoltage = -10;
const double loadCellMaxVoltage = +10;
const int encoderPulsesPerRev = 500;
const double shaftRadius = 0.003;// motor shaft radius in cm
const double loadCellScale[7] = {(1/sqrt(2.0)) * 50.53}; //From calibration test with weights

#define NUMBER_OF_MUSCLES 1
//State machine and paradigm

#define MOTOR 0
#define LOAD_CELL 1
#define ENCODER 2
#define MOTOR_ENABLE 3
#define ENCODER_MOTOR_SYNC 4

#define     STATE_INIT 0
#define     STATE_WINDING_UP 1
#define     STATE_OPEN_LOOP 2
#define     STATE_CLOSED_LOOP 3
#define     STATE_SINUSOIDAL_VOLTAGE 4
#define     STATE_WHITE_NOISE 5

#define     STATE_SHUTTING_DOWN 6


#define     OPEN_LOOP 0
#define     CLOSED_LOOP 0

#define LOADCELL_DAQ 0
#define MOTOR_VOLTAGE_DAQ 1
#define MOTOR_ENCODER_DAQ 2
#define MOTOR_REF_DAQ 3

extern int dataAcquisitionFlag[4];
int proceedState(int *);


typedef unsigned char       BYTE;
#endif
