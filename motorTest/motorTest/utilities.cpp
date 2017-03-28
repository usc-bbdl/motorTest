#include <utilities.h>
#include <stdio.h>
#include <conio.h>
#include "motorControl.h"
#include "scanMotorVoltage.h"

motorControl motors;
scanMotorVoltage scanMotorVoltageObject(& motors);
int proceedState(int *state)
{
    int menu = 0;
    float64 sinValues[4]; //These are the values used to specify the sin wave parameters
    static motorControl motors;

    switch(*state)
    {
    case STATE_INIT:
        printf("Motors Winding Up; Next stage is Open-Loop\n");
        motors.motorEnable();
        motors.motorWindUp();
        *state = STATE_OPEN_LOOP;
        break;
    case STATE_OPEN_LOOP:
        //Start data acquision
        motors.motorControllerStart();
        Sleep(1000);
        printf("Open-Loop ; Next stage is Selecting Experiment Paradigm\n");
        *state = STATE_CLOSED_LOOP;
        break;
    case STATE_CLOSED_LOOP:        
        printf("\n\nWhat Paradigm do you want to run?\n");
        printf("\t[0] Shut down\n");
        printf("\t[1] Sinusoidal Voltage\n");
        printf("\t[2] White Noise Voltage\n");
        
        do{
            scanf("%d", &menu);
            if (!((menu <= 1) || (menu >= 0)))
                printf("Wrong input! try Again.\n");
        }while (!((menu <= 1) || (menu >= 0)));
        switch(menu)
        {
        case 1:
            *state = STATE_SINUSOIDAL_VOLTAGE;
            printf("Sinusoidal Voltage Selected\n");
            printf("Press Space to continue\n");
            break;
        case 2:
            *state = STATE_WHITE_NOISE;
            printf("White Noice Selected\n");
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
        break;
    case STATE_SINUSOIDAL_VOLTAGE:
        sinValues[3] = true; //this is the flag that tells scanMotorVolage.cpp methods that a Sinusoidal Voltage scan should be done.
       
            printf("What frequency, amplitude and offset do you want (type three values separated by spaces)?\n");
            do{// this is a do-while loop that enforces limits on sin wave specification values.
                std::cin>>sinValues[0]>>sinValues[1]>>sinValues[2]; // frequency, amplitude, and offset.

                if (!((sinValues[0] <= 1000) && (sinValues[1] <=1000) && (sinValues[2] <=1000))) // need to secify limits if any.
                    printf("Wrong input! try Again.\n\a");

            }while (!((sinValues[0] <= 1000) && (sinValues[1] <=1000) && (sinValues[2] <=1000)));

        scanMotorVoltageObject.setSinValues(sinValues); // passes array address to function that sets the values.
        scanMotorVoltageObject.startScan();

        *state= STATE_CLOSED_LOOP;
        break;
    case STATE_WHITE_NOISE:
        sinValues[3] = false; //this is the flag that tells scanMotorVolage.cpp methods that a Sinusoidal Voltage scan should be done.

            printf("What amplitude and offset do you want (type two values separated by spaces)?\n");
            do{// this is a do-while loop that enforces limits on sin wave specification values.
                sinValues[0] = 0.0; // setting to zero because this feature does not frequency.
                std::cin>>sinValues[1]>>sinValues[2]; // frequency, amplitude, and offset.

                if (!((sinValues[0] <= 1000) && (sinValues[1] <=1000) && (sinValues[2] <=1000))) // need to secify limits if any.
                    printf("Wrong input! try Again.\n\a");

            }while (!((sinValues[0] <= 1000) && (sinValues[1] <=1000) && (sinValues[2] <=1000)));

        scanMotorVoltageObject.setSinValues(sinValues); // passes array address to function that sets the values.
        scanMotorVoltageObject.startScan();

    *state = STATE_CLOSED_LOOP;
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


