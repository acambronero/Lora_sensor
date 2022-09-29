#include "loratimersloop.h"
#include "datahelper.h"
#include <chrono>
#include <thread>

bool loratimersloop::CheckTimeout()
{
    uint64_t time = Timestamp();
    if(Abs(time - lastTime) >= timeoutMs)
        return true;
    return false;
}

loratimersloop::loratimersloop()
{
    this->exit = false;
    this->running = true;
}

void loratimersloop::SetTimeoutCallback(std::function<void(void)> timeoutCallback)
{
    this->timeoutCallback = timeoutCallback;
}

void loratimersloop::Run()
{
    lastTime = Timestamp();
    while(!exit){
        if (CheckTimeout()){
            timeoutCallback();
            lastTime = Timestamp();

        }
        sleep_milliseconds(interval);
    }
    running = false;
    /*if (Shot == true){
        running = true;
        lastTime = 0;
    }
    else
        running = false;*/
}


