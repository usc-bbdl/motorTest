#include "motorControl.h"
#include <utilities.h>
#include <stdio.h>
#include <math.h>
#include <algorithm>
#include <ctime>
motorControl::motorControl()
{
    I = 3;
    b0 = 0;
    b1 = 0;
    b2 = 0;
    a1 = 0;
    a2 = 0;
    optimalGain = 0;

    ki=0;
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
    {
        loadCellOffset[i] = 0;
        loadCellData[i] = 0.0;
        motorRef[i]= 0.0;
        muscleLength[i] = 0.0;
        motorCommand[i] = 0.0;
        errorNow[i] = 0.0;
        motorCommandPastValue[i] =0.0;
    }
    tick=0.0;
    tock=0.0;
    sprintf(dataSample,"");
    //Flag for data acquision, the sequence is [loadcell, motorVoltage, motorEncoder, motorRef]
    bool flag[5];
    for(int i = 0; i<5; i++)
        flag[i] = true;
    setDataAcquisitionFlag(flag);
    closedLoop = false;
    newTrial = 0;
    experimentControl = 0.0;
}
void motorControl::setDataAcquisitionFlag(bool flag[])
{
    dataAcquisitionFlag[LOADCELL_DAQ] = flag[0];
    dataAcquisitionFlag[MOTOR_VOLTAGE_DAQ] = flag[1];
    dataAcquisitionFlag[MOTOR_ENCODER_DAQ] = flag[2];
    dataAcquisitionFlag[MOTOR_REF_DAQ] = flag[3];
    dataAcquisitionFlag[ERROR_] = flag[4];
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
    float64 errorForce[NUMBER_OF_MUSCLES]= {0.0},integral[NUMBER_OF_MUSCLES]={0};
    
    
    FILE *dataFile;
    char dataTemp[100]="", errBuff[2048]={'\0'};
    
    
    createFileName();
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
            if (closedLoop==true)
            {

                customizedControllerLaw(i);
            }
            else if (closedLoop==false)
            {
                motorCommand[i] = motorRef[i];
            }
           
        }
        printf("LC1: %+4.2f; MR1: %+4.2f; MC1: %+4.2f\r",loadCellData[0],motorRef[0], motorCommand[0]);
        ReleaseMutex( hIOMutex);
        createDataSampleString();
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
void motorControl::customizedControllerLaw(int muscleIndex)
{ 
    static double errorPastValue[NUMBER_OF_MUSCLES], error2PastValue[NUMBER_OF_MUSCLES];
    motorCommand[muscleIndex] = (b0/a0)*(tock - tick)* errorNow[muscleIndex] + (b1/a0) *(tock - tick)* errorPastValue[muscleIndex] + (b2/a0) *error2PastValue[muscleIndex] - (a1/a0) * motorCommandPastValue[muscleIndex] - (a2/a0) * motorCommand2PastValue[muscleIndex];
    //motorCommand[muscleIndex] = 20 * motorCommandPastValue[muscleIndex] + 20*(tock - tick)* errorNow[muscleIndex]; 
    
    if (motorCommand[muscleIndex] > motorMaxVoltage)
            motorCommand[muscleIndex] = motorMaxVoltage;
        if (motorCommand[muscleIndex] < motorMinVoltage)
            motorCommand[muscleIndex] = motorMinVoltage;

    errorNow[muscleIndex] = motorRef[muscleIndex] - loadCellData[muscleIndex] * optimalGain;

    error2PastValue[muscleIndex] = errorPastValue[muscleIndex];
    errorPastValue[muscleIndex] = errorNow[muscleIndex];
    motorCommand2PastValue[muscleIndex] = motorCommandPastValue[muscleIndex];
    motorCommandPastValue[muscleIndex] = motorCommand[muscleIndex];


    //loadCellData
    //motorCommand
}
void motorControl::setControlLaw(int controlLaw)
{
    this->controlLaw = controlLaw;
    switch (controlLaw) 
    {
        case LAG:
            b0 = 1;
            b1 = 0;
            b2 = 0;
            a1 = 0;
            a2 = 0;
            optimalGain = 1;
            //NEEDS AN OBSERVER. NOT COMPLETE. DO NOT USE NOW.
        break;
        case INTEGRAL:
            b0 = 2.0;
            b1 = 2.0;
            b2 = 0;
            a0 = 2;
            a1 = -2;
            a2 = 0;
            optimalGain = 1;
            
        break;
        case OPTIMAL_GAIN:
            b0 = 1;
            b1 = 0;
            b2 = 0;
            a1 = 0;
            a2 = 0;
            optimalGain = 1;
        break;
        case PI:
            b0 = 1;
            b1 = 0;
            b2 = 0;
            a1 = 0;
            a2 = 0;
            optimalGain = 1;
        break;
    }
}
void motorControl::createDataSampleString()
{
    char dataTemp[100]="";
    static int k;
    if ((newTrial) | (k>0))
    {
        experimentControl = paradigm [k];
        k = k + 1;
        if (k == 4)
            k = 0;
    }
    else
    {
        k = 0;
        experimentControl = 0;
    }
    sprintf(dataSample,"%.3f,%08.3f",tock,experimentControl);
        if (dataAcquisitionFlag[0])
        {
            for (int i=0; i < NUMBER_OF_MUSCLES; i++)
            {
                sprintf(dataTemp,",%.6f",loadCellData[i]);
                strcat (dataSample, dataTemp);
            }
        }
        if (dataAcquisitionFlag[1])
        {
            for (int i=0; i < NUMBER_OF_MUSCLES; i++)
            {
                sprintf(dataTemp,",%.6f",encoderData[i]);
                strcat (dataSample, dataTemp);
            }
        }
        if (dataAcquisitionFlag[2])
        {
            for (int i=0; i < NUMBER_OF_MUSCLES; i++)
            {
                sprintf(dataTemp,",%.6f",motorRef[i]);
                strcat (dataSample, dataTemp);
            }
        }
        if (dataAcquisitionFlag[3])
        {
            for (int i=0; i < NUMBER_OF_MUSCLES; i++)
            {
                sprintf(dataTemp,",%.6f",motorCommand[i]);
                strcat (dataSample, dataTemp);
            }
        }
        if (dataAcquisitionFlag[4])
        {
             for (int i=0; i < NUMBER_OF_MUSCLES; i++)
            {
                sprintf(dataTemp,",%.6f",errorNow[i]);
                strcat (dataSample, dataTemp);
            }
        }

        sprintf(dataTemp,"\n");
        strcat (dataSample, dataTemp);
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
    strcpy(header,"Time, Experimental Paradigm,");
    if (dataAcquisitionFlag[0]){
        for (int i=0; i < NUMBER_OF_MUSCLES;i++)
        {
            sprintf(dataTemp,"Measured Force %d,", i);
            strcat (header, dataTemp);
        }
    }
    if (dataAcquisitionFlag[1]){
        for (int i=0; i < NUMBER_OF_MUSCLES;i++)
        {
            sprintf(dataTemp,"Shaft Encoder %d,", i);
            strcat (header, dataTemp);
        }
    }
    if (dataAcquisitionFlag[2]){
        for (int i=0; i < NUMBER_OF_MUSCLES;i++)
        {
            sprintf(dataTemp,"Reference Force %d,", i);
            strcat (header, dataTemp);
        }
    }
    if (dataAcquisitionFlag[3]){
        for (int i=0; i < NUMBER_OF_MUSCLES;i++)
        {
            sprintf(dataTemp,"Motor Command %d,", i);
            strcat (header, dataTemp);
        }
    if (dataAcquisitionFlag[4]){
        for (int i=0; i < NUMBER_OF_MUSCLES;i++)
        {
            sprintf(dataTemp,"Error %d,", i);
            strcat (header, dataTemp);
        }

    }
    }
    sprintf(dataTemp,"\n");
    strcat (header, dataTemp);
    return 0;
}
    
int motorControl::initializeTaskHandles()
{
    char        errBuff[2048]={'\0'};
    int32       error=0;
    DAQmxErrChk (DAQmxCreateTask("",&loadCelltaskHandle));
    DAQmxErrChk (DAQmxCreateTask("",&motorTaskHandle));
    DAQmxErrChk (DAQmxCreateTask("",&motorEnableHandle));
    char portNumber[50],channelDescription[20];
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
    dataEnable = 0xFF;
    return 0;
}
int motorControl::createPortNumber(int hardware,int index,char * portNumber,char * channelDescription)
{
    switch(hardware) 
    {
    case MOTOR:
        sprintf(portNumber,"PXI1Slot%d/ao%d", motorAnalogOutSlotNumber,motorAnalogOutChannelNumber[index]);
        sprintf(channelDescription,"motor%d",index);
        break;
    case LOAD_CELL:
      sprintf(portNumber,"PXI1Slot%d/ai%d", loadCellSlotNumber,loadCellChannelNumber[index]);
      sprintf(channelDescription,"loadcell%d",index);
      break;
    case ENCODER:
      sprintf(portNumber,"PXI1Slot%d/ctr%d", motorEncoderSlotNumber,motorEncoderChannelNumber[index]);
      sprintf(channelDescription,"encoder%d",index);
      break;
    case MOTOR_ENABLE:
        sprintf(portNumber,"PXI1Slot%d/port%d", motorEnableSlotNumber,motorEnableChannelNumber);
        break;
    case ENCODER_MOTOR_SYNC:
         sprintf(portNumber,"/PXI1Slot%d/ai/SampleClock",loadCellSlotNumber); 
        break;
    }
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
int motorControl::createFileName()
{
    time_t t = time(NULL);
    tm* timePtr = localtime(&t);
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
void motorControl::updateMotorRef(float64 *a,int newTrial, double * paradigm){
    for (int i =0;i<NUMBER_OF_MUSCLES;i++)
    {
        motorRef[i] = a[i];
    }
    this->newTrial = newTrial;
    for (int i =0;i<5;i++)
    {
        this->paradigm[i] = paradigm[i];
    }
}

void  motorControl::setClosedLoop()
{
    closedLoop = true;
}