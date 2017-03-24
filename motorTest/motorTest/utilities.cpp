#include <utilities.h>
#include <stdio.h>
#include <conio.h>
#include <motorControl.h>

//#include <expParadigmMuscleLengthCalibration.h>
//#include <expParadigmServoPerturbation.h>
//#include <expParadigmManualPerturbation.h>
//#include <expParadigmVoluntaryMovement.h>
//#include <FPGAControl.h>
//#include <expParadigmCDMRPimplant.h>
int dataAcquisitionFlag[4] = {1,1,1,1}; //force[0], motorEncoder[1], reference force[2], motor command [3]
int proceedState(int *state)
{
    int retVal = 0;
    int menu = 0;
    //static servoControl servo;
    static motorControl motors();
    //static expParadigmMuscleLengthCalibration paradigmMuscleLengthCalibration(&servo);
    //static expParadigmServoPerturbation paradigmServoPerturbation(loadCellOffsets.loadCell1,loadCellOffsets.loadCell2,&servo);
    static expParadigmManualPerturbation paradigmManualPerturbation;
    static expParadigmVoluntaryMovement paradigmVoluntaryMovement(&motors);
    //static FPGAControl bicepFPGA(BICEP,&motors);
    //static FPGAControl tricepFPGA(TRICEP,&motors);
    static expParadigmCDMRPimplant paradigmCDMRPimplant;

    switch(*state)
    {
    case STATE_INIT:
        printf("Motors Winding Up; Next stage is Open-Loop\n");
        motors.motorEnable();
        motors.motorWindUp();
        *state = STATE_WINDING_UP;
        break;
    case STATE_WINDING_UP:
        //Start Neural FPGA and Feeding muscle length data        
        *state = STATE_OPEN_LOOP;
        printf("Open-Loop ; Next stage is Closed-Loop\n");
        break;
    case STATE_OPEN_LOOP:
        //Connect the Neural FPGA and DAQ, Start controlling muscle force
        motors.motorControllerStart();
        Sleep(1000);
        motors.resetMuscleLength = TRUE;
        printf("Closed-Loop ; Next stage is Experiment Paradigm\n");
        *state = STATE_CLOSED_LOOP;
        break;
    case STATE_CLOSED_LOOP:        
        printf("\n\nWhat Paradigm do you want to run?\n");
        printf("\t[0] Exit Program\n");
        printf("\t[1] Muscle Length Calibration\n");
        printf("\t[2] Servo Perturbation\n");
        printf("\t[3] Manual Perturbation\n");
        printf("\t[4] Voluntary Movement\n");
        printf("\t[5] CDMRP Implant\n\n User Input:");
        do{
            scanf("%d", &menu);
            if (!((menu <= 5) || (menu >= 0)))
                printf("Wrong input! try Again.\n");
        }while (!((menu <= 5) || (menu >= 0)));
        switch(menu)
        {
        case 1:
            *state = STATE_PARADIGM_LENGTH_CALIBRATION;
            printf("Muscle Length Calibration Selected\n");
            printf("Press Space to continue\n");
            break;
        case 2:
            *state = STATE_RUN_PARADIGM_SERVO_PERTURBATION;
            printf("Servo Perturbation Selected\n");
            printf("Press Space to continue\n");
            break;
        case 3:
            *state = STATE_RUN_PARADIGM_MANUAL_PERTURBATION;
            printf("Manual Perturbation Selected\n");
            printf("Press Space to continue\n");
            break;
        case 4:
            *state = STATE_RUN_PARADIGM_VOLUNTARY_MOVEMENT;
            printf("Voluntary Movement Selected\n");
            printf("Press Space to continue\n");
            break;
        case 5:
            *state = STATE_RUN_PARADIGM_CDMRP_IMPLANT;
            printf("CDMRP implant Selected\n");
            printf("Press Space to continue\n");
            break;
        case 0:
            *state = STATE_SHUTTING_DOWN;
            printf("\nPress space to shutdown\n");
            printf("Press Space to continue\n");
            break;
        default: break;
        }
//        Sleep(500);
//        paradigm.startParadigm(&bicepFPGA, &tricepFPGA, &motors);
        break;
    case STATE_PARADIGM_LENGTH_CALIBRATION:
        //retVal = paradigmMuscleLengthCalibration.startParadigm(&motors);
        if(retVal != -1)
            *state = STATE_CLOSED_LOOP;
            else {
                *state = STATE_SHUTTING_DOWN;
                printf("\nPress space to shutdown\n");
            }
        break;
    case STATE_RUN_PARADIGM_SERVO_PERTURBATION:
        //retVal = paradigmServoPerturbation.startParadigm(&bicepFPGA, &tricepFPGA, &motors);
        if(retVal != -1)
            *state = STATE_CLOSED_LOOP;
            else {
                *state = STATE_SHUTTING_DOWN;
                printf("\nPress space to shutdown\n");
            }
        break;
    case STATE_RUN_PARADIGM_MANUAL_PERTURBATION:
        //retVal = paradigmManualPerturbation.startParadigm(&bicepFPGA, &tricepFPGA, &motors);
        if(retVal != -1)
            *state = STATE_CLOSED_LOOP;
            else {
                *state = STATE_SHUTTING_DOWN;
                printf("\nPress space to shutdown\n");
            }
        break;
    case STATE_RUN_PARADIGM_CDMRP_IMPLANT:
        //paradigmCDMRPimplant.readData();
        retVal = paradigmCDMRPimplant.startParadigm(&motors);
        if(retVal != -1)
            *state = STATE_CLOSED_LOOP;
            else {
                *state = STATE_SHUTTING_DOWN;
                printf("\nPress space to shutdown\n");
            }
        break;
    case STATE_RUN_PARADIGM_VOLUNTARY_MOVEMENT:
        //retVal = paradigmVoluntaryMovement.startParadigm(&bicepFPGA, &tricepFPGA, &motors);
        if(retVal != -1)
            *state = STATE_CLOSED_LOOP;
            else {
                *state = STATE_SHUTTING_DOWN;
                printf("\nPress space to shutdown\n");
            }
        break;
    case STATE_SHUTTING_DOWN:
        printf("Shutting Down\n");
        motors.motorControllerEnd();
        printf("Press Enter to Exit\n");
        exit(0);
        break;
    }
    return 0;
}
void waitkey() {
	getchar();
}


int ReInterpret(float32 in, int32 *out)
{
    memcpy(out, &in, sizeof(int32));
    return 0;
}

int ReInterpret(int32 in, int32 *out)
{
    memcpy(out, &in, sizeof(int32));
    return 0;
}

int ReInterpret(int in, float *out)
{
    memcpy(out, &in, sizeof(float));
    return 0;
}
