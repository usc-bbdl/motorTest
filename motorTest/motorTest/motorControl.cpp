#include "motorControl.h"
#include <utilities.h>
#include <stdio.h>
#include <math.h>
#include <algorithm>
#include <ctime>
motorControl::motorControl()
{
    I = 3;
    createHeader4DataFile();
    errorMotorControl = initializeTaskHandles();
    createDataEnable();
    createWindingUpCommand();
    isEnable = FALSE;
    isWindUp = FALSE;
    isControlling = FALSE;
    encoderBias = 0;
    encoderGain = 1;
    for (int i = 0; i < NUMBER_OF_MUSCLES; i ++)
        loadCellOffset[i] = 0;
}
motorControl::~motorControl()
{
    DAQmxClearTask(motorEnableHandle);
    DAQmxClearTask(motorTaskHandle);
    DAQmxClearTask(loadCelltaskHandle);
    live = FALSE;
}

int motorControl::motorEnable()
{
    
    char        errBuff[2048]={'\0'};
    int32       error=0;
    float64 zeroVoltages[NUMBER_OF_MUSCLES]={0.0},zeroVoltage={0.0};
    DAQmxErrChk (DAQmxStartTask(motorEnableHandle));
    DAQmxErrChk (DAQmxStartTask(motorTaskHandle));
    DAQmxErrChk (DAQmxWriteDigitalU32(motorEnableHandle,1,1,10.0,DAQmx_Val_GroupByChannel,&dataEnable,NULL,NULL));
    DAQmxErrChk (DAQmxWriteAnalogF64(motorTaskHandle,1,FALSE,10,DAQmx_Val_GroupByChannel,zeroVoltages,NULL,NULL));
    Sleep(500);
    DAQmxStopTask(motorTaskHandle);
    DAQmxStopTask(motorEnableHandle);
    isEnable = TRUE;
Error:
	if( DAQmxFailed(error) ) {
		DAQmxGetExtendedErrorInfo(errBuff,2048);
        DAQmxStopTask(motorTaskHandle);
		DAQmxClearTask(motorTaskHandle);
        DAQmxStopTask(motorEnableHandle);
		DAQmxClearTask(motorEnableHandle);
		printf("DAQmx Error: %s\n",errBuff);
        printf("Motor enable Error\n");
        return 2;
	}
    return 0;
}

int motorControl::motorWindUp()
{
    char        errBuff[2048]={'\0'};
    int32       error=0;
    float64 windingUpCmnd[5]={0.5,0.5,0.5,0,0};
    if (isEnable){
        DAQmxErrChk (DAQmxStartTask(motorTaskHandle));
        DAQmxErrChk (DAQmxWriteAnalogF64(motorTaskHandle,1,FALSE,10,DAQmx_Val_GroupByChannel,windingUpCmnd,NULL,NULL));
        Sleep(500);
        isWindUp = TRUE;

        DAQmxStopTask(motorTaskHandle);
        
        printf("Windup Pass.\n");
    }else  printf("Motors must be first enabled prior to winding up.\n");
Error:
	if( DAQmxFailed(error) ) {
		DAQmxGetExtendedErrorInfo(errBuff,2048);
        DAQmxStopTask(motorTaskHandle);
        DAQmxClearTask(motorTaskHandle);
		printf("DAQmx Error: %s\n",errBuff);
        printf("winding up Error\n");
        return 3;
	}
     return 0;
}

void motorControl::motorControlLoop(void* a)
{
	((motorControl*)a)->controlLoop();
}

void motorControl::controlLoop(void)
{
    int32       error=0;
    float cotexDrive = 0.0;
    bool firstSample = TRUE;
    bool32 isLate = {0};
    double tick=0.0,tock=0.0;
    float64 motorCommand[NUMBER_OF_MUSCLES]={0.0},errorForce[NUMBER_OF_MUSCLES]= {0.0},integral[NUMBER_OF_MUSCLES]={0};
    float64 loadCellData[NUMBER_OF_MUSCLES]={0.0}, motorRef[NUMBER_OF_MUSCLES]={0.0}, muscleLength[NUMBER_OF_MUSCLES]={0.0};
    FILE *dataFile;
    char fileName[200], dataSample[600]="", dataTemp[100]="", errBuff[2048]={'\0'};

    time_t t = time(NULL);
    tm* timePtr = localtime(&t);
    //createFileName(fileName,timePtr);
        sprintf_s(
            fileName,
            "C:\\data\\realTimeData%4d_%02d_%02d_%02d_%02d_%02d.txt",
            timePtr->tm_year+1900, 
            timePtr->tm_mon+1, 
            timePtr->tm_mday, 
            timePtr->tm_hour, 
            timePtr->tm_min, 
            timePtr->tm_sec
            );

    dataFile = fopen(fileName,"w");
    fprintf(dataFile,header);
    
    DAQmxErrChk (DAQmxStartTask(loadCelltaskHandle));
    DAQmxErrChk (DAQmxStartTask(motorTaskHandle));
    for (int i = 0; i < NUMBER_OF_MUSCLES; i++)
        DAQmxErrChk (DAQmxStartTask(encodertaskHandle[i]));
    DAQmxErrChk (DAQmxStartTask(motorEnableHandle));

    timeData.resetTimer();
    tick = timeData.getCurrentTime();
    float64 goffsetLoadCell[2]={0};
    while(live)
    {
        DAQmxErrChk (DAQmxWriteDigitalU32(motorEnableHandle,1,1,10.0,DAQmx_Val_GroupByChannel,&dataEnable,NULL,NULL));
        WaitForSingleObject(hIOMutex, INFINITE);
        DAQmxErrChk(DAQmxWaitForNextSampleClock(loadCelltaskHandle,10, &isLate));
        DAQmxErrChk (DAQmxReadAnalogF64(loadCelltaskHandle,-1,10.0,DAQmx_Val_GroupByScanNumber,loadCellData,NUMBER_OF_MUSCLES,NULL,NULL));
        DAQmxErrChk (DAQmxWriteAnalogF64(motorTaskHandle,1,FALSE,10,DAQmx_Val_GroupByChannel,motorCommand,NULL,NULL));
        for (int i=0; i < NUMBER_OF_MUSCLES; i++)
            DAQmxErrChk (DAQmxReadCounterF64(encodertaskHandle[i],1,10.0,&encoderData[i],1,NULL,0));
        tock = timeData.getCurrentTime();
        scaleMuscleLengthData(encoderData);
        scaleloadCellData(loadCellData);
        if (firstSample)
        {
            for (int i=0; i < NUMBER_OF_MUSCLES; i++)
                loadCellOffset[i] = loadCellData[i];
            firstSample = FALSE;
        }
        for (int i=0; i < NUMBER_OF_MUSCLES; i++)
        {
            errorForce[i] = motorRef[i] - loadCellData[i];
            integral[i] = integral[i] + errorForce[i] * (tock - tick);
            motorCommand[i] = integral[i] * I;
            if (motorCommand[i] > motorMaxVoltage)
                motorCommand[i] = motorMaxVoltage;
            if (motorCommand[i] < motorMinVoltage)
                motorCommand[i] = motorMinVoltage;
        }
        printf("LC1: %+4.2f; MR1: %+4.2f, \r",loadCellData[0],motorRef[0]);
        ReleaseMutex( hIOMutex);
        sprintf(dataSample,"%.3f",tock);
        if (dataAcquisitionFlag[0])
        {
            for (int i=0; i < NUMBER_OF_MUSCLES; i++)
                sprintf(dataTemp,",%.6f",loadCellData[i]);
            strcat (dataSample, dataTemp);
        }
        if (dataAcquisitionFlag[1])
        {
            for (int i=0; i < NUMBER_OF_MUSCLES; i++)
                sprintf(dataTemp,",%.6f",encoderData[i]);
            strcat (dataSample, dataTemp);
        }
        if (dataAcquisitionFlag[2])
        {
            for (int i=0; i < NUMBER_OF_MUSCLES; i++)
                sprintf(dataTemp,",%.6f",motorRef[i]);
            strcat (dataSample, dataTemp);
        }
        if (dataAcquisitionFlag[3])
        {
            for (int i=0; i < NUMBER_OF_MUSCLES; i++)
                sprintf(dataTemp,",%.6f",motorCommand[i]);
            strcat (dataSample, dataTemp);
        }

        //sprintf(dataTemp,",%d,%d,%d,%d,%.3f,%.3f,%d\n",gammaStatic1, gammaDynamic1, gammaStatic2, gammaDynamic2, cortexDrive[0], cortexDrive[1],newTrial);
        sprintf(dataTemp,"\n");
        strcat (dataSample, dataTemp);
        fprintf(dataFile,dataSample);
        tick = timeData.getCurrentTime();

    }

    DAQmxStopTask(motorTaskHandle);
    DAQmxStopTask(motorEnableHandle);
    isControlling = FALSE;
    fclose(dataFile);
Error:
	if( DAQmxFailed(error) ) {
		DAQmxGetExtendedErrorInfo(errBuff,2048);
		/*********************************************/
		// DAQmx Stop Code
		/*********************************************/
		DAQmxStopTask(loadCelltaskHandle);
		DAQmxClearTask(loadCelltaskHandle);
        DAQmxStopTask(encodertaskHandle[0]);
        DAQmxStopTask(encodertaskHandle[1]);
        DAQmxStopTask(encodertaskHandle[2]);
		DAQmxClearTask(encodertaskHandle[0]);
        DAQmxClearTask(encodertaskHandle[1]);
        DAQmxClearTask(encodertaskHandle[2]);
        DAQmxStopTask(motorTaskHandle);
		DAQmxClearTask(motorTaskHandle);
        DAQmxStopTask(motorEnableHandle);
		DAQmxClearTask(motorEnableHandle);
		printf("DAQmx Error: %s\n",errBuff);
        printf("Motor control Error\n");
	}
}

int motorControl::motorControllerStart()
{
    if ((isEnable) && (isWindUp))
    {
        isControlling = TRUE;
        live = TRUE;
        hIOMutex = CreateMutex(NULL, FALSE, NULL);
		isControlling = TRUE;
		_beginthread(motorControl::motorControlLoop,0,this);
    }else
    {
        isControlling = FALSE;
        printf("Motors must be first enabled or wind up before closed-loop control.\n");
    }
    return 0;
}
int motorControl::motorControllerEnd()
{
    live = FALSE;
    Sleep(500);
    motorControl::motorDisable();
    isControlling = FALSE;
    DAQmxStopTask(motorTaskHandle);
	DAQmxClearTask(motorTaskHandle);
    DAQmxStopTask(loadCelltaskHandle);
	DAQmxClearTask(loadCelltaskHandle);
    DAQmxStopTask(encodertaskHandle[0]);
    DAQmxStopTask(encodertaskHandle[1]);
    DAQmxStopTask(encodertaskHandle[2]);
	DAQmxClearTask(encodertaskHandle[0]);
    DAQmxClearTask(encodertaskHandle[1]);
    DAQmxClearTask(encodertaskHandle[2]);
    return 0;
}

int motorControl::motorDisable()
{
	int32       error=0;
	char        errBuff[2048] = {'\0'};
    uInt32      dataDisable=0x00000000;
    int32		written;
    float64 zeroVoltages[5]={0.0,0.0,0.0,0.0,0.0};
    while(isControlling){
    }
    DAQmxErrChk (DAQmxStartTask(motorEnableHandle));
    DAQmxErrChk (DAQmxStartTask(motorTaskHandle));
    DAQmxErrChk (DAQmxWriteDigitalU32(motorEnableHandle,1,1,10.0,DAQmx_Val_GroupByChannel,&dataDisable,NULL,NULL));
    DAQmxErrChk (DAQmxWriteAnalogF64(motorTaskHandle,1,FALSE,10,DAQmx_Val_GroupByChannel,zeroVoltages,NULL,NULL));
    Sleep(500);
    DAQmxStopTask(motorTaskHandle);
    DAQmxStopTask(motorEnableHandle);
    isControlling = FALSE;
    isWindUp = FALSE;
    live = FALSE;
    isEnable = FALSE;

Error:
	if( DAQmxFailed(error) ){
		printf("DisableMotor Error: %s\n",errBuff);
        DAQmxGetExtendedErrorInfo(errBuff,2048);
        printf("DAQmx Error: %s\n",errBuff);
		DAQmxStopTask(motorEnableHandle);
        DAQmxStopTask(motorTaskHandle);
        DAQmxClearTask(motorEnableHandle);
        DAQmxClearTask(motorTaskHandle);
    }
	return 0;
}

int motorControl::createHeader4DataFile()
{
    char dataTemp[20];
    strcpy(header,"Time");
    if (dataAcquisitionFlag[0]){
        for (int i=0; i < NUMBER_OF_MUSCLES;i++)
        {
            sprintf(dataTemp,"Measured Force %d", i);
            strcat (header, dataTemp);
        }
    }
    if (dataAcquisitionFlag[1]){
        for (int i=0; i < NUMBER_OF_MUSCLES;i++)
        {
            sprintf(dataTemp,"Shaft Encoder %d", i);
            strcat (header, dataTemp);
        }
    }
    if (dataAcquisitionFlag[2]){
        for (int i=0; i < NUMBER_OF_MUSCLES;i++)
        {
            sprintf(dataTemp,"Reference Force %d", i);
            strcat (header, dataTemp);
        }
    }
    if (dataAcquisitionFlag[3]){
        for (int i=0; i < NUMBER_OF_MUSCLES;i++)
        {
            sprintf(dataTemp,"Motor Command %d", i);
            strcat (header, dataTemp);
        }
    }
    return 0;
}
    
int motorControl::initializeTaskHandles()
{
    char        errBuff[2048]={'\0'};
    int32       error=0;
    DAQmxErrChk (DAQmxCreateTask("",&loadCelltaskHandle));
    DAQmxErrChk (DAQmxCreateTask("",&motorTaskHandle));
    DAQmxErrChk (DAQmxCreateTask("",&motorEnableHandle));
    char portNumber[20],channelDescription[20];
    for (int i = 0; i < NUMBER_OF_MUSCLES; i ++)
    {
        createPortNumber(LOAD_CELL,i,portNumber,channelDescription);
        DAQmxErrChk (DAQmxCreateAIVoltageChan(loadCelltaskHandle,portNumber,channelDescription,DAQmx_Val_RSE,loadCellMinVoltage,loadCellMaxVoltage,DAQmx_Val_Volts,NULL));
        //DAQmxErrChk (DAQmxCreateAIVoltageChan(loadCelltaskHandle,"PXI1Slot5/ai0","loadCell0",DAQmx_Val_RSE,loadCellMinVoltage,loadCellMaxVoltage,DAQmx_Val_Volts,NULL));
        DAQmxErrChk (DAQmxCfgSampClkTiming(loadCelltaskHandle,"",samplingFrequency,DAQmx_Val_Rising,DAQmx_Val_HWTimedSinglePoint,NULL));
        DAQmxErrChk (DAQmxSetRealTimeConvLateErrorsToWarnings(loadCelltaskHandle,1));
        createPortNumber(MOTOR,i,portNumber,channelDescription);
        DAQmxErrChk (DAQmxCreateAOVoltageChan(motorTaskHandle,portNumber,channelDescription,motorMinVoltage,motorMaxVoltage,DAQmx_Val_Volts,NULL));
        DAQmxErrChk (DAQmxCfgSampClkTiming(motorTaskHandle,"",samplingFrequency,DAQmx_Val_Rising,DAQmx_Val_HWTimedSinglePoint,1));
        createPortNumber(MOTOR_ENABLE,i,portNumber,channelDescription);
        DAQmxErrChk (DAQmxCreateDOChan(motorEnableHandle,portNumber,channelDescription,DAQmx_Val_ChanForAllLines));
        DAQmxErrChk (DAQmxCreateTask("",&encodertaskHandle[i]));
        createPortNumber(ENCODER,i,portNumber,channelDescription);
        DAQmxErrChk (DAQmxCreateCIAngEncoderChan(encodertaskHandle[0],portNumber,channelDescription,DAQmx_Val_X4,0,0.0,DAQmx_Val_AHighBHigh,DAQmx_Val_Degrees,encoderPulsesPerRev,0.0,""));
        createPortNumber(ENCODER_MOTOR_SYNC,i,portNumber,channelDescription);
        //DAQmxErrChk (DAQmxCfgSampClkTiming(encodertaskHandle[0],"/PXI1Slot5/ai/SampleClock",samplingFrequency,DAQmx_Val_Rising,DAQmx_Val_HWTimedSinglePoint,1));
        DAQmxErrChk (DAQmxCfgSampClkTiming(encodertaskHandle[0],portNumber,samplingFrequency,DAQmx_Val_Rising,DAQmx_Val_HWTimedSinglePoint,1));
    }
Error:
	if( DAQmxFailed(error) ) {
		DAQmxGetExtendedErrorInfo(errBuff,2048);
		DAQmxClearTask(motorTaskHandle);
		DAQmxClearTask(loadCelltaskHandle);
		DAQmxClearTask(motorEnableHandle);
		printf("DAQmx Error: %s\n",errBuff);
        printf("Motor, load cell or encoder initialization error\n");
        return 1;
	}
    return 0;
}
int motorControl::createDataEnable()
{
    dataEnable = 0x01;
    return 0;
}
int motorControl::createPortNumber(int hardware,int index,char * portNumber,char * channelDescription)
{
    //TBD
    return 0;
}
int motorControl::createWindingUpCommand()
{
    for (int i = 0; i < NUMBER_OF_MUSCLES; i ++)
    {
        windingUpCmnd[i] = 0.5;
    }
    return 0;
}
int motorControl::createFileName(char * fileName,tm * timePtr)
{
 /**   sprintf_s(
            fileName,
            "C:\\data\\realTimeData%4d_%02d_%02d_%02d_%02d_%02d.txt",
            timePtr->tm_year+1900, 
            timePtr->tm_mon+1, 
            timePtr->tm_mday, 
            timePtr->tm_hour, 
            timePtr->tm_min, 
            timePtr->tm_sec
            );
            **/
    return 0;
}
int motorControl::scaleMuscleLengthData(float64 *encoderData)
{
    for (int i = 0; i < NUMBER_OF_MUSCLES; i ++)
    {
        encoderData[i] = (2 * PI * shaftRadius * encoderData[i] / 365);
        encoderData[i] = encoderBias + encoderData[0] *encoderGain;
    }
    return 0;
}
int motorControl::scaleloadCellData(float64 *loadCellData)
{
    for (int i = 0; i < NUMBER_OF_MUSCLES; i ++)
        loadCellData[i] = (loadCellData[i] * loadCellScale[i]) - loadCellOffset[i];
    return 0;
}


 