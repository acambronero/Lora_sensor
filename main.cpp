#include "monitoringhandler.h"
#include "unistd.h"
#include "timershandler.h"

int a = 1;

#define BUFFER_SIZE 127 // Define the payload size here


int main(){
    MonitoringHandler monitoring;
    monitoring.Initialize();
    uint8_t TxdBuffer[BUFFER_SIZE];
    TxdBuffer[0] = 0x50;//P
    TxdBuffer[1] = 0x4F;//O
    //TxdBuffer[1] = 0x49;//I
    TxdBuffer[2] = 0x4E;//N
    TxdBuffer[3] = 0x47;//G
    //int i = 0;

    std::array<uint8_t, 255> data = {0};
    uint16_t size = 0;

    while (a){
        //monitoring.Send(TxdBuffer, 4);
        //if (!monitoring.lora->radioHandler->TimerTxFired){
            /*if (monitoring.firstTime){
                monitoring.firstTime = false;
                monitoring.lora->radioHandler->TimerTxTimeout = false;
            }*/
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

                /*for (int i = 4; i < BUFFER_SIZE; i++){
                    TxdBuffer[i] = i - 4;
                }*/
                //monitoring.Send(TxdBuffer, 4);
            }
            if (monitoring.sendReady){
                /*for (int i = 0; i < 10; i++){
                    monitoring.Send(TxdBuffer, 4);
                    usleep(5000);
                }*/
                monitoring.Send(TxdBuffer, 4);
                monitoring.sendReady = 0;
            }
                usleep(10000);
        //}
    }
}




