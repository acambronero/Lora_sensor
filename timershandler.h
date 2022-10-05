#ifndef TIMERSHANDLER_H
#define TIMERSHANDLER_H

#include "stdint.h"
#include <thread>
#include "loratimersloop.h"

class TimersHandler
{
public:
    TimersHandler();
    ~TimersHandler();
    void Start(uint8_t idx, uint32_t timeout, std::function<void(void)> *callback);
    //void StartShot(uint8_t idx, uint32_t timeout, std::function<void(void)> *callback);
    void Stop(uint8_t idx);
protected:
    std::thread *threads[10];
    loratimersloop timersloop[10];
};

#endif // TIMERSHANDLER_H
