#include "mcu/timer.h"
#include "timershandler.h"
#include "datahelper.h"
#include <iostream>

TimersHandler timerTickers;

uint32_t timerTimes[10];
bool timerInUse[10] = {false, false, false, false, false, false, false, false, false, false};

// External functions

void TimerConfig(void)
{
	/// \todo Nothing to do here for ESP32
}

//void TimerInit(TimerEvent_t *obj, void (*callback)(void))
void TimerInit(TimerEvent_t *obj, std::function<void(void)> callback)
{
	// Look for an available Ticker
	for (int idx = 0; idx < 10; idx++)
	{
		if (timerInUse[idx] == false)
		{
			timerInUse[idx] = true;
			obj->timerNum = idx;
			obj->Callback = callback;
			return;
		}
	}
	LOG_LIB("TIM", "No more timers available!");
	/// \todo We run out of tickers, what do we do now???
}

void timerCallback(TimerEvent_t *obj)
{
	// Nothing to do here for the ESP32
}

void TimerStart(TimerEvent_t *obj)
{
	int idx = obj->timerNum;
	if (obj->oneShot)
	{
        timerTickers.StartShot(idx, timerTimes[idx], &(obj->Callback));
	}
	else
	{
        timerTickers.Start(idx, timerTimes[idx], &(obj->Callback));
	}
}

void TimerStop(TimerEvent_t *obj)
{
	int idx = obj->timerNum;
    //std::cout << "IDX: " << timerInUse[idx] << std::endl;
    //timerTickers[idx].detach();
    timerTickers.Stop(idx);
}

void TimerReset(TimerEvent_t *obj)
{
	int idx = obj->timerNum;
    //timerTickers[idx].detach();
    timerTickers.Stop(idx);
	if (obj->oneShot)
	{
        timerTickers.StartShot(idx, timerTimes[idx], &(obj->Callback));
	}
	else
	{
        timerTickers.Start(idx, timerTimes[idx], &(obj->Callback));
	}
}

void TimerSetValue(TimerEvent_t *obj, uint32_t value)
{
	int idx = obj->timerNum;
	timerTimes[idx] = value;
}

TimerTime_t TimerGetCurrentTime(void)
{
    return Timestamp();
}

TimerTime_t TimerGetElapsedTime(TimerTime_t past)
{
    uint32_t nowInTicks = Timestamp();
	uint32_t pastInTicks = past;
	TimerTime_t diff = nowInTicks - pastInTicks;

	return diff;
}

//#endif
