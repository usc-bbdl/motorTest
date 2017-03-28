#ifndef GEN_EXP_H
#define GEN_EXP_H
#include "motorControl.h"
#include "customizedExperimentalParadigm.h"
class generalExperimentalParadigm
{
    int motorError;
    motorControl motors;
    customizedExperimentalParadigm customizedExperiment;
    int motorState;
public:
    generalExperimentalParadigm();
    ~generalExperimentalParadigm(void);
    void nextState();
    void endExperiment();
};
#endif