#include "motorControl.h"
#include <utilities.h>
#include <stdio.h>
#include <math.h>
#include <algorithm>
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
    float64 zeroVoltages[5]={0.0,0.0,0.0,0.0,0.0},zeroVoltage={0.0};
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
    bool keepReading=TRUE;
    bool32 isLate = {0};
    double tick=0.0,tock=0.0;
    float64 motorCommand[5]={0.0,0.0,0.0,0.0,0.0},errorForce[3]= {0.0,0.0,0.0},integral[3]={0.0,0.0,0.0},EMG={0.0};
    char        errBuff[2048]={'\0'};
    FILE *dataFile;
    time_t t = time(NULL);
    tm* timePtr = localtime(&t);
    char fileName[200];
    char dataSample[600]="";
    char dataTemp[100]="";
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
    //fprintf(dataFile,"Time, Load Cell1, Load Cell2, Motor Command1, Motor Command2, Length 1, Length2, Velocity1, Velocity2, EMG1, EMG2, is sample missed\n");
    //fprintf(dataFile,"Time, Load Cell1, Load Cell2, Length 1, Length2, Velocity1, Velocity2, EMG1, EMG2, GammaStat, GammaDyn, is sample missed\n");
    //fprintf(dataFile,"Time, Load Cell1, Load Cell2, Length 1, Length2, motorRef1, motorRef2, spindleIa1, spindleIa2, spindleII1, spindleII2, EMG1, EMG2, GammaStat, GammaDyn, is sample missed\n");
    //fprintf(dataFile,"Time, Length 1, Length2, EMG1, EMG2, GammaStat, GammaDyn, is sample missed\n");
    fprintf(dataFile,header);
    DAQmxErrChk (DAQmxStartTask(loadCelltaskHandle));
    DAQmxErrChk (DAQmxStartTask(motorTaskHandle));
    DAQmxErrChk (DAQmxStartTask(encodertaskHandle[0]));
    DAQmxErrChk (DAQmxStartTask(encodertaskHandle[1]));
    DAQmxErrChk (DAQmxStartTask(encodertaskHandle[2]));
    DAQmxErrChk (DAQmxStartTask(motorEnableHandle));
    timeData.resetTimer();
    tick = timeData.getCurrentTime();
    float64 goffsetLoadCell[2]={0};
    int expProtocol = 0;
    int expProtocoAdvance = 0;
    uInt32 clockBit = 0x00000080;
    uInt32 clockBitXOR;
    clockBitXOR = ((clockBit>>7) | (dataEnable>>31));
    bool flg = true;
    while(live)
    {
        if(flg)
        {
        clockBitXOR = 0xff;
        flg = false;
        }
        else
        {
            clockBitXOR = 0x01111111;
            flg = true;
        }
        if (dataEnable == 0x07)
            dataEnable = 0x87;
        else
            dataEnable = 0x07;
        //else if (dataEnable == 71)
        //    dataEnable = 7;

        //DAQmxErrChk (DAQmxWriteDigitalU32(motorEnableHandle,1,1,10.0,DAQmx_Val_GroupByChannel,&clockBitXOR,NULL,NULL));
        DAQmxErrChk (DAQmxWriteDigitalU32(motorEnableHandle,1,1,10.0,DAQmx_Val_GroupByChannel,&dataEnable,NULL,NULL));
        WaitForSingleObject(hIOMutex, INFINITE);
        //desire Forced, muscle Length, muscle Velocity PIPES should be read here
        
        DAQmxErrChk(DAQmxWaitForNextSampleClock(loadCelltaskHandle,10, &isLate));
        DAQmxErrChk (DAQmxReadAnalogF64(loadCelltaskHandle,-1,10.0,DAQmx_Val_GroupByScanNumber,loadCellData,4,NULL,NULL));
        DAQmxErrChk (DAQmxWriteAnalogF64(motorTaskHandle,1,FALSE,10,DAQmx_Val_GroupByChannel,motorCommand,NULL,NULL));
        DAQmxErrChk (DAQmxReadCounterF64(encodertaskHandle[0],1,10.0,encoderData1,1,NULL,0));
        DAQmxErrChk (DAQmxReadCounterF64(encodertaskHandle[1],1,10.0,encoderData2,1,NULL,0));
        DAQmxErrChk (DAQmxReadCounterF64(encodertaskHandle[2],1,10.0,encoderData3,1,NULL,0));
        if (dataAcquisitionFlag[1]){
            EMG = muscleEMG[0];
            if (EMG > 6)
                EMG = 6;
            if (EMG < -6)
                EMG = -6;
        }
        else
            EMG = 0;
        tock = timeData.getCurrentTime();
        if (resetMuscleLength)
        {
            muscleLengthOffset[0] = 2 * PI * shaftRadius * encoderData1[0] / 365;
            muscleLengthOffset[1] = 2 * PI * shaftRadius * encoderData2[0] / 365;
            muscleLengthOffset[2] = 2 * PI * shaftRadius * encoderData3[0] / 365;
            resetMuscleLength = FALSE;
        }
        muscleLength[0] = ((2 * PI * shaftRadius * encoderData1[0] / 365) - muscleLengthOffset[0]);
        muscleLength[0] = encoderBias[0] + muscleLength[0] *encoderGain[0];
        muscleLength[1] = ((2 * PI * shaftRadius * encoderData2[0] / 365) - muscleLengthOffset[1]);
        muscleLength[1] = encoderBias[1] + muscleLength[1] *encoderGain[1];

        muscleLength[2] = ((2 * PI * shaftRadius * encoderData3[0] / 365) - muscleLengthOffset[2]);
        muscleLength[2] = encoderBias[2] + muscleLength[2] *encoderGain[2];

        muscleVel[0] = (muscleLength[0] -  muscleLengthPreviousTick[0]) / (tock - tick);
        muscleVel[1] = (muscleLength[1] -  muscleLengthPreviousTick[1]) / (tock - tick);

        muscleLengthPreviousTick[0] = muscleLength[0];
        muscleLengthPreviousTick[1] = muscleLength[1];
        
        loadCellData[0] = (loadCellData[0] * loadCellScale1) - loadCellOffset1;
        loadCellData[1] = (loadCellData[1] * loadCellScale2) - loadCellOffset2;
        loadCellData[2] = (loadCellData[2] * loadCellScale3) - loadCellOffset3;

        
        if(newPdgm_Flag)
        {
            //errorForce[0] = newPdgm_ref[1] - loadCellData[0];
            //errorForce[1] = newPdgm_ref[0] - loadCellData[1];
            //errorForce[2] = newPdgm_ref[0] - loadCellData[2];
            motorRef[0] = newPdgm_ref[1];
            motorRef[1] = newPdgm_ref[0];
            motorRef[2] = newPdgm_ref[0];
            
        }
        errorForce[0] = motorRef[0] - loadCellData[0];
        errorForce[1] = motorRef[1] - loadCellData[1];
        errorForce[2] = motorRef[2] - loadCellData[2];

        integral[0] = integral[0] + errorForce[0] * (tock - tick);
        integral[1] = integral[1] + errorForce[1] * (tock - tick);
        integral[2] = integral[2] + errorForce[2] * (tock - tick);

        motorCommand[0] = integral[0] * I;
        motorCommand[1] = integral[1] * I;
        motorCommand[2] = integral[2] * I;

        motorCommand[3] = EMG;
        if (motorCommand[0] > motorMaxVoltage)
            motorCommand[0] = motorMaxVoltage;
        if (motorCommand[0] < motorMinVoltage)
            motorCommand[0] = motorMinVoltage;
        if (motorCommand[1] > motorMaxVoltage)
            motorCommand[1] = motorMaxVoltage;
        if (motorCommand[1] < motorMinVoltage)
            motorCommand[1] = motorMinVoltage;
        //printf("F1: %+6.2f; F2: %+6.2f;L1: %+6.2f; L2: %+6.2f;, Dyn: %d, Sta: %d, \r",loadCellData[0],loadCellData[1],muscleLength[0],muscleLength[1],gammaDynamic1, gammaStatic1);
        printf("LC1: %+4.2f; LC2: %+4.2f; LC31: %+4.2f; MR1: %+4.2f; MR2: %+4.2f, MR3: %+4.2f, \r",loadCellData[0],loadCellData[1],loadCellData[2],motorRef[0], motorRef[1], motorRef[2]);
        //printf("F1: %+6.2f; F2: %+6.2f;MtrCmd1: %010d, MtrCmd2: %010d\r",loadCellData[0],loadCellData[1], motorCommand[0], motorCommand[1]);
        //printf("Mtr1: %020d, Mtr2: %020d\r\r",motorCommand[0], motorCommand[1]);
        ReleaseMutex( hIOMutex);
        //fprintf(dataFile,"%.3f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d\n",tock,loadCellData[0],loadCellData[1],motorRef[0],motorRef[1], muscleLength[0], muscleLength[1], muscleVel[0],muscleVel[1], muscleEMG[0], muscleEMG[1], isLate);
        //fprintf(dataFile,"%.3f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d,%d,%d\n",tock,loadCellData[0],loadCellData[1], muscleLength[0], muscleLength[1], muscleVel[0],muscleVel[1], muscleEMG[0], muscleEMG[1], gammaStatic, gammaDynamic, isLate);
        //fprintf(dataFile,"%.3f,%.6f,%.6f,%.6f,%.6f,%d,%d,%d\n",tock, muscleLength[0], muscleLength[1], muscleEMG[0], muscleEMG[1], gammaStatic, gammaDynamic, isLate);
        sprintf(dataSample,"%.3f,%.1f,%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f",tock,loadCellData[3],expProtocol,muscleLength[0], muscleLength[1], muscleLength[2], loadCellData[0],loadCellData[1],loadCellData[2]);
        if (dataAcquisitionFlag[0]){
            sprintf(dataTemp,",%.6f,%.6f,%.6f,%.6f,%.6f,%.6f",motorRef[0],motorRef[1],motorRef[2],motorCommand[0],motorCommand[1],motorCommand[2]);
            strcat (dataSample, dataTemp);
        }
        if (dataAcquisitionFlag[1]){
            sprintf(dataTemp,",%.6f,%.6f",muscleEMG[0], muscleEMG[1]);
            strcat (dataSample, dataTemp);
        }
         if (dataAcquisitionFlag[2]){
            sprintf(dataTemp,",%.6f,%.6f",spindleIa[0], spindleIa[1]);
            strcat (dataSample, dataTemp);
        }
        if (dataAcquisitionFlag[3]){
            sprintf(dataTemp,",%.6f,%.6f",spindleII[0], spindleII[1]);
            strcat (dataSample, dataTemp);
        }
        if (dataAcquisitionFlag[4]){
            sprintf(dataTemp,",%d,%d",muscleSpikeCount[0], muscleSpikeCount[1]);
            strcat (dataSample, dataTemp);
        }
        if (dataAcquisitionFlag[5]){
            sprintf(dataTemp,",%u,%u",raster_MN_1[0], raster_MN_1[1]);
            strcat (dataSample, dataTemp);
        }
        if (dataAcquisitionFlag[6]){
            sprintf(dataTemp,",%u,%u",raster_MN_2[0], raster_MN_2[1]);
            strcat (dataSample, dataTemp);
        }
        if (dataAcquisitionFlag[7]){
            sprintf(dataTemp,",%u,%u",raster_MN_3[0], raster_MN_3[1]);
            strcat (dataSample, dataTemp);
        }
        if (dataAcquisitionFlag[8]){
            sprintf(dataTemp,",%u,%u",raster_MN_4[0], raster_MN_4[1]);
            strcat (dataSample, dataTemp);
        }
        if (dataAcquisitionFlag[9]){
            sprintf(dataTemp,",%u,%u",raster_MN_5[0], raster_MN_5[1]);
            strcat (dataSample, dataTemp);
        }
        if (dataAcquisitionFlag[10]){
            sprintf(dataTemp,",%u,%u",raster_MN_6[0], raster_MN_6[1]);
            strcat (dataSample, dataTemp);
        }
        if (dataAcquisitionFlag[11]){
            cortexDrive[0] = max((cortexVoluntaryAmp -0) * sin (2 * 3.1416 * cortexVoluntaryFreq * tick), 0);
            cortexDrive[1] = max((cortexVoluntaryAmp -0) * sin (2 * 3.1416 * cortexVoluntaryFreq * tick + 3.1416), 0);
        }
        //sprintf(dataTemp,",%d,%d,%d,%d,%.3f,%.3f,%d\n",gammaStatic1, gammaDynamic1, gammaStatic2, gammaDynamic2, cortexDrive[0], cortexDrive[1],newTrial);
        sprintf(dataTemp,"\n");
        if (trialTrigger == 1){
            expProtocoAdvance = 1;
            trialTrigger = 0;
        }
        if (trialTrigger == 2){
            expProtocoAdvance = 10;
            trialTrigger = 0;
        }
        if (trialTrigger == 3){
            expProtocoAdvance = 11;
            trialTrigger = 0;
        }
        expProtocol = 0;
        switch(expProtocoAdvance){
            case 1:
                expProtocol = -1000;
                expProtocoAdvance = 2;
                break;
            case 2:
                expProtocol = gammaDynamic1;
                expProtocoAdvance = 3;
                break;
            case 3:
                expProtocol = gammaStatic1;
                expProtocoAdvance = 4;
                break;
            case 4:
                expProtocol =  cortexDrive[0];
                expProtocoAdvance = 5;
                break;
            case 5:
                expProtocol = gammaDynamic2;
                expProtocoAdvance = 6;
                break;
            case 6:
                expProtocol = gammaStatic2;
                expProtocoAdvance = 7;
                break;
            case 7:
                expProtocol =  cortexDrive[1];
                expProtocoAdvance = 8;
                break;
            case 8: 
                expProtocol = angle;
                expProtocoAdvance = 9;
                break;
            case 9:
                expProtocol = velocity;
                expProtocoAdvance = 0;
                break;
            case 10:
                expProtocol = -1;
                expProtocoAdvance = 0;
                break;
            case 11:
                expProtocol = -2;
                expProtocoAdvance = 0;
                break;
        }
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
}
int createPortNumber(int hardware,int index,char * portNumber,char * channelDescription)
{
    //TBD
}