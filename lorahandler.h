#ifndef LORAHANDLER_H
#define LORAHANDLER_H

#include "radio/radiohandler.h"       //if Arduino --> radio/radio.h
#include "radio/sx126x/sx126x.h"
#include "datahelper.h"
#include "stdint.h"


#define BUFFER_SIZE 64

class SX126Handler;
class SPILora;


class lorahandler
{
public:
    lorahandler();

    void TestSpi(SPILora* lora);
    void SetSpiLora(SPIBase* lora);
    void SetIrqsEnable(std::vector<IrqsActivated> irq);
    uint32_t HardwareInit();
    uint32_t HardwareReInit();
    void Init();
    void ReInit();
    void SetChannel(uint32_t freq);
    void SetTxConfig(int8_t power, uint32_t fdev,
                     uint32_t bandwidth, uint32_t datarate,
                     uint8_t coderate, uint16_t preambleLen,
                     bool fixLen, bool crcOn, bool freqHopOn,
                     uint8_t hopPeriod, bool iqInverted, uint32_t timeout);
    void SetRxConfig(uint32_t bandwidth,
                     uint32_t datarate, uint8_t coderate,
                     uint32_t bandwidthAfc, uint16_t preambleLen,
                     uint16_t symbTimeout, bool fixLen,
                     uint8_t payloadLen,
                     bool crcOn, bool freqHopOn, uint8_t hopPeriod,
                     bool iqInverted, bool rxContinuous);
    void SetRx(uint32_t timeout);
    void SetRxBoosted(uint32_t timeout);
    void Send(uint8_t *buffer, uint8_t size);
    void IrqProcess(uint8_t *dataReady);
    void GetPayloadData(uint8_t& payload, uint16_t size);

    RadioStatus_t Get_Status();
    RadioError_t Get_DeviceErrors();

    uint16_t BufferSize = BUFFER_SIZE;
    uint8_t RcvBuffer[BUFFER_SIZE];
    uint8_t RcvSerialBuffer[BUFFER_SIZE];
    uint8_t TxdBuffer[BUFFER_SIZE];
    bool sensor = false;
    SX126Handler *sxHandler;
    RadioHandler *radioHandler;
    SX126xDriver *sxDriver;

private:

};

#endif // LORAHANDLER_H
