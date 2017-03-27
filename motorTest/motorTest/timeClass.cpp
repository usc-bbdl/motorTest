#include "timeClass.h"
#include <Windows.h>
#include <stdio.h>
#include <math.h>
#include <algorithm>
#include <ctime>
timeClass::timeClass(void)
{
    QueryPerformanceFrequency(&frequency);
    QueryPerformanceCounter(&initialTick);
}


timeClass::~timeClass(void)
{
}

// Reset the timer
int timeClass::resetTimer(void)
{
    QueryPerformanceCounter(&initialTick);
    return 0;
}


// Get current time in seconds
double timeClass::getCurrentTime(void)
{
    QueryPerformanceCounter(&currentTick);
    actualTime = (double)(currentTick.QuadPart - initialTick.QuadPart);
    actualTime /= (double)frequency.QuadPart;
    return actualTime;
}