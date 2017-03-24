#ifndef TIME_CLASS_H
#define TIME_CLASS_H
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
