//
#include <stdio.h>
#include <windows.h>
#include <iostream>
#include <conio.h>
#include <utilities.h>
//#include <servoControl.h>
int main()
{ 
    printf("Press 'Spc' to move forward\n\n");
    printf("Press 'Esc' to terminate\n");
    printf("\nInitialization; Next stage is Motors Winding up\n");
    int gExperimentState = STATE_INIT;
    bool stayInTheLoop = TRUE;
    while(stayInTheLoop)
    {
        char key = 0;
        if (kbhit()!=0){
            key = getch();
            switch ( key ) 
            {
                case 27:        // Terminate Anytime when Escape Is Pressed...
                    stayInTheLoop = FALSE;
                    gExperimentState = STATE_SHUTTING_DOWN;
                    proceedState(&gExperimentState);
                    break;
                case ' ':       // Move forward in the state machine
                    proceedState(&gExperimentState);
                    break;
           }
        }
    }
   return 0;
}