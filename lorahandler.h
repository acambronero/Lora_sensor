#ifndef LORAHANDLER_H
#define LORAHANDLER_H

#include <iostream>
#include <string>
#include "datahelper.h"
#include "radiodefinitions.h"
#include <array>

#define BUFFER_SIZE 64

class RadioHandler;
class SX126Handler;
class SX126xDriver;
class SPIBase;

class LoraHandler
{
public:
    LoraHandler();

    //void TestSpi(SPILora* lora);
    void SetSpiLora(SPIBase* lora);
    uint32_t HardwareInit();
    void Init();
    void SetChannel(uint32_t freq);
    void SetTxConfig(int8_t power, uint32_t bandwidth, uint32_t datarate,
                     uint8_t coderate, uint16_t preambleLen,
                     bool fixLen, bool crcOn, bool iqInverted, uint32_t TxTimeout);

    void SetRxConfig(uint32_t bandwidth, uint32_t datarate, uint8_t coderate,
                     uint16_t preambleLen, uint16_t symbTimeout, bool fixLen,
                     uint8_t payloadLen, bool crcOn, bool iqInverted,
                     bool rxContinuous, uint32_t RxTimeout);

    void SetRx(uint32_t timeout);
    void SetRxBoosted(uint32_t timeout);
    void Send(uint8_t *buffer, uint8_t size, uint32_t TxTimeout, uint32_t RxTimeout);
    void IrqProcess(uint8_t *dataReady);
    std::array<uint8_t, 255> GetPayloadData(uint16_t size);

    RadioStatus_t Get_Status();
    RadioError_t Get_DeviceErrors();

    uint16_t BufferSize = BUFFER_SIZE;
    uint8_t RcvBuffer[BUFFER_SIZE];
    uint8_t RcvSerialBuffer[BUFFER_SIZE];
    uint8_t TxdBuffer[BUFFER_SIZE];
    bool sensor = false;
    SX126Handler *sxHandler = nullptr;
    RadioHandler *radioHandler = nullptr;
    SX126xDriver *sxDriver = nullptr;

private:

};

#endif // LORAHANDLER_H
