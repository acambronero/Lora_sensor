#include "monitoringhandler.h"
#include "SX126x/radio/radiohandler.h"
#include "unistd.h"
#include "timershandler.h"
#include <thread>
#include <chrono>
#include "datahelper.h"
#include <iostream>

int a = 1;

#define BUFFER_SIZE 127 // Define the payload size here

int main(){
    MonitoringHandler monitoring;
    monitoring.Initialize();
    uint8_t TxdBuffer[BUFFER_SIZE];
    TxdBuffer[0] = 0x50;//P
    TxdBuffer[1] = 0x4F;//O
    TxdBuffer[2] = 0x4E;//N
    TxdBuffer[3] = 0x47;//G
    TxdBuffer[4] = 0x50;//P
    TxdBuffer[5] = 0x4F;//O
    TxdBuffer[6] = 0x4E;//N
    TxdBuffer[7] = 0x47;//G
    TxdBuffer[8] = 0x50;//P
    TxdBuffer[9] = 0x4F;//O
    TxdBuffer[10] = 0x4E;//N
    TxdBuffer[11] = 0x47;//G
    TxdBuffer[12] = 0x50;//P
    TxdBuffer[13] = 0x4F;//O
    TxdBuffer[14] = 0x4E;//N
    TxdBuffer[15] = 0x47;//G

    std::array<uint8_t, 255> data = {0};
    uint16_t size = 0;

    while (a){
            monitoring.Run();
            if (monitoring.dataReady) {
                data = monitoring.lora->radioHandler->GetPayloadData(size);

                std::cout << "Data Received: " << std::endl;
                std::cout << data[0] << std::endl;
                std::cout << data[1] << std::endl;
                std::cout << data[2] << std::endl;
                std::cout << data[3] << std::endl;
                monitoring.dataReady = 0;
                monitoring.sendReady = 1;
            }
            if (monitoring.sendReady){
                std::cout << "Data Send: " <<  rand() << std::endl;
                monitoring.Send(TxdBuffer, 16, 0, 0);
                monitoring.sendReady = 0;
            }
            sleep_milliseconds(150);
    }
}




