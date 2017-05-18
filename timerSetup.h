#ifndef H_TIMERSETUP
#define H_TIMERSETUP
#include <Arduino.h>
#include <SPI.h>

#define CPU_HZ 48000000
#define TIMER_PRESCALER_DIV 1024


void startTimer(int frequencyHz);
void setTimerFrequency(int frequencyHz);

#endif // H_TIMERSETUP
