#include "timershandler.h"
#include "loratimersloop.h"
#include "boards/mcu/timer.h"
#include <iostream>

TimersHandler::TimersHandler()
{

}

TimersHandler::~TimersHandler()
{
    for(int i = 0; i < 10 ; i++){
        if (threads[i]){
            if (timersloop[i].IsRunning()){
                timersloop[i].NotifyExit();
            }
            if (threads[i]->joinable())
                threads[i]->join();
            delete threads[i];
            threads[i] = nullptr;
        }
    }
}

void TimersHandler::Start(uint8_t idx, uint32_t timeout, std::function<void(void)> *callback)
{
    timersloop[idx].SetInterval(1);
    timersloop[idx].SetTimeOut(timeout);
    timersloop[idx].SetTimeoutCallback(*callback);

    threads[idx] = new std::thread(std::bind(&loratimersloop::Run, &timersloop[idx]));
}

/*void TimersHandler::StartShot(uint8_t idx, uint32_t timeout, std::function<void(void)> *callback)
{
    timersloop[idx].SetInterval(1);
    timersloop[idx].SetTimeOut(timeout);
    timersloop[idx].SetShot(true);
    timersloop[idx].SetTimeoutCallback(*callback);

    threads[idx] = new std::thread(std::bind(&loratimersloop::Run, &timersloop[idx]));
}*/

void TimersHandler::Stop(uint8_t idx)
{
    timersloop[idx].NotifyExit();
    if (threads[idx]->joinable()){
        threads[idx]->join();
    }
    delete threads[idx];
    threads[idx] = nullptr;
}

