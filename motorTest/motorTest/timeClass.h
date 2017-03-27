#ifndef TIME_CLASS_H
#define TIME_CLASS_H
#include <iostream>
#include <windows.h>
#include <process.h>
#include <ctime>
#include <timeClass.h>
class timeClass
{
      LARGE_INTEGER initialTick, currentTick, frequency;
public:

    double actualTime;
    timeClass(void);	
    ~timeClass(void);
    int resetTimer();
    double getCurrentTime(void);
};
#endif
