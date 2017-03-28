#include <stdio.h>
#include <windows.h>
#include <iostream>
#include <conio.h>
#include "generalExperimentalParadigm.h"
int main()
{ 
    generalExperimentalParadigm experiment;
    bool stayInTheLoop = TRUE;
    while(stayInTheLoop)
    {
        char key = 0;
        if (kbhit()!=0){
            key = getch();
            switch ( key ) 
            {
                case 27:        //Terminate Anytime when Escape Is Pressed...
                    experiment.endExperiment();
                    break;
                case ' ':       //Move forward in running the experiment
                    experiment.nextState();
                    break;
           }
        }
    }
   return 0;
}