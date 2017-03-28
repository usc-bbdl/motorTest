#ifndef CUST_EXP_H
#define CUST_EXP_H
#include "motorControl.h"

class customizedExperimentalParadigm{
    motorControl *motors;
public:
    void initialize(motorControl*);
    int runExperiment();
};
#endif