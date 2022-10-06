#ifndef LORATIMERSLOOP_H
#define LORATIMERSLOOP_H

#include <functional>
//#include "itools.h"
#include "loop.h"
#include <stdint.h>

class loratimersloop : public Loop
{
private:
    uint64_t timeoutMs;
    std::function<void(void)> timeoutCallback;
    uint64_t lastTime;
    bool Shot = false;

    bool CheckTimeout();

public:
    loratimersloop();
    void SetTimeOut(uint32_t timeout){
        timeoutMs = timeout;
    }
    void SetTimeoutCallback(std::function<void(void)> timeoutCallback);

    void Run();

};

#endif // LORATIMERSLOOP_H
