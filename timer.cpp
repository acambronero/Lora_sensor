#include "timer.h"
#include "timershandler.h"
#include "datahelper.h"


#ifdef ARDUINO
Ticker timerTickers[10];
#else
TimersHandler timerTickers;
#endif

uint32_t timerTimes[10];
bool timerInUse[10] = {false, false, false, false, false, false, false, false, false, false};

#ifdef ARDUINO
void TimerInit(TimerEvent_t *obj, void (*callback)(void));
#else
void TimerInit(TimerEvent_t *obj, std::function<void(void)> callback)
#endif
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
	/// \todo We run out of tickers, what do we do now???
}

void TimerStart(TimerEvent_t *obj)
{
    int idx = obj->timerNum;
#ifdef ARDUINO
    timerTickers[idx].attach_ms(timerTimes[idx], obj->Callback);
#else
    timerTickers.Start(idx, timerTimes[idx], &(obj->Callback));
#endif
}

void TimerStop(TimerEvent_t *obj)
{
	int idx = obj->timerNum;
    //std::cout << "IDX: " << timerInUse[idx] << std::endl;
#ifdef ARDUINO
    timerTickers[idx].detach();
#else
    timerTickers.Stop(idx);
#endif
}

void TimerReset(TimerEvent_t *obj)
{
	int idx = obj->timerNum;
#ifdef ARDUINO
    timerTickers[idx].detach();
    timerTickers[idx].attach_ms(timerTimes[idx], obj->Callback);
#else
    timerTickers.Stop(idx);
    timerTickers.Start(idx, timerTimes[idx], &(obj->Callback));
#endif

}

void TimerSetValue(TimerEvent_t *obj, uint32_t value)
{
	int idx = obj->timerNum;
	timerTimes[idx] = value;
}

TimerTime_t TimerGetCurrentTime(void)
{
#ifdef ARDUINO
    return millis();
#else
    return Timestamp();
#endif
}

TimerTime_t TimerGetElapsedTime(TimerTime_t past)
{
#ifdef ARDUINO
    uint32_t nowInTicks = millis();
#else
    uint32_t nowInTicks = Timestamp();
#endif
	uint32_t pastInTicks = past;
	TimerTime_t diff = nowInTicks - pastInTicks;

	return diff;
}
