#include "customizedExperimentalParadigm.h"
#include "scanMotorVoltage.h"
#include "motorControl.h"
void customizedExperimentalParadigm::initialize(motorControl* temp)
{
    motors = temp;
}
int customizedExperimentalParadigm::runExperiment()
{
    scanMotorVoltage scanMotorVoltageObject(motors);
    motorControl motorFunctions;
    const int STATE_EXIT = 0;
    const int STATE_SINUSOIDAL_VOLTAGE = 1;
    const int STATE_WHITE_NOISE = 2;
    const int STATE_CONTROLLER_TEST =3;
    int menu = 0;
    char answer;
    float64 sinValues[4]; //These are the values used to specify the sin wave parameters
    bool showMenu = true;
    while (showMenu)
    {
        printf("\n\nWhat Paradigm do you want to run?\n");
        printf("\t[0] Shut down\n");
        printf("\t[1] Sinusoidal Voltage\n");
        printf("\t[2] White Noise Voltage\n");
        printf("\t[3] Controller Test\n");
        do{
            scanf("%d", &menu);
            if (!((menu <= 3) || (menu >= 0)))
                printf("Wrong input! try Again.\n");
        }while (!((menu <= 3) || (menu >= 0)));

        switch(menu)
        {
            case 1:
                experimentalState = STATE_SINUSOIDAL_VOLTAGE;
                printf("Sinusoidal Voltage Selected\n");
                scanMotorVoltageObject.setSinusoidalScan();
                scanMotorVoltageObject.startScan();
                printf("Press Space to continue\n");
                break;
            case 2:
                experimentalState = STATE_WHITE_NOISE;
                printf("White Noice Selected\n");
                scanMotorVoltageObject.setNoiseScan();
                scanMotorVoltageObject.startScan();
                printf("Press Space to continue\n");
                break;
            case 3:
                experimentalState = STATE_CONTROLLER_TEST;
                printf("Controller Test Selected\n");
                printf("\nWhat controller do you want to run\n");
                printf("\t[1] Lag\n");
                printf("\t[2] Integral\n");

                do{
                    scanf("%d", &menu);
                    if (!((menu <= 2) || (menu > 0)))
                        printf("Wrong input! try Again.\n");
                    }while (!((menu <= 2) || (menu > 0)));
                
                printf("\nReady to start impulse response analysis?\n");
               
                do{
                    std::cin>>answer;
                    if (!((answer == 'n') || (answer == 'y')))
                        printf("Wrong input! try Again.\n");
                    }while (!((answer == 'n') || (answer == 'y')));
               // motorFunctions.setControlLaw(menu);
                motorFunctions.setClosedLoop();
                scanMotorVoltageObject.setNoiseScan();
                scanMotorVoltageObject.startScan();
                break;

            case 0:
                experimentalState = STATE_EXIT;
                printf("\nPress space to exit\n");
                printf("Press Space to continue\n");
                exit(0);
                break;
            default: 
                std::cout<<"Error, bad input, quitting\n";
                break;
        }
    }
   
    return 0;
}
