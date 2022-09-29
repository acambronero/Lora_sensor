#include "lorahandler.h"
#include "radio/sx126x/sx126x.h"
#include <string>
#include "SX126xHardware.h"
#include "spilora.h"
#include <iostream>

lorahandler::lorahandler()
{
    sxHandler = new SX126Handler();
    radioHandler = new RadioHandler();
    sxDriver = new SX126xDriver();

    sxHandler->sxDriver = sxDriver;
    radioHandler->sxHandler = sxHandler;
}

void lorahandler::TestSpi(SPILora *lora)
{
    uint8_t a[] = {0x23, 0x60};
    uint8_t b[] = {0, 0};

    lora->Transfer(a, b, 2);
}

void lorahandler::SetSpiLora(SPIBase *lora)
{
    sxHandler->SetSpiLora(lora);
}

void lorahandler::SetIrqsEnable(std::vector<IrqsActivated> irq)
{
    radioHandler->SetIrqsEnable(irq);
}

void lorahandler::Init()
{
    radioHandler->Init(sxHandler);
}

void lorahandler::ReInit()
{
    radioHandler->ReInit(sxHandler);
}

void lorahandler::SetChannel(uint32_t freq)
{
    radioHandler->SetChannel(freq, sxHandler);
}

void lorahandler::SetTxConfig(int8_t power, uint32_t fdev, uint32_t bandwidth, uint32_t datarate, uint8_t coderate, uint16_t preambleLen, bool fixLen, bool crcOn, bool freqHopOn, uint8_t hopPeriod, bool iqInverted, uint32_t timeout)
{
    radioHandler->SetTxConfig(power, fdev, bandwidth, datarate, coderate, preambleLen, fixLen, crcOn, freqHopOn, hopPeriod, iqInverted, timeout, sxHandler);
}

void lorahandler::SetRxConfig(uint32_t bandwidth, uint32_t datarate, uint8_t coderate, uint32_t bandwidthAfc, uint16_t preambleLen, uint16_t symbTimeout, bool fixLen, uint8_t payloadLen, bool crcOn, bool freqHopOn, uint8_t hopPeriod, bool iqInverted, bool rxContinuous)
{
    radioHandler->SetRxConfig(bandwidth, datarate, coderate, bandwidthAfc, preambleLen, symbTimeout, fixLen, payloadLen, crcOn, freqHopOn, hopPeriod, iqInverted, rxContinuous, sxHandler);
 }

void lorahandler::SetRx(uint32_t timeout)
{
    radioHandler->Rx(timeout, sxHandler);
}

void lorahandler::SetRxBoosted(uint32_t timeout)
{
    radioHandler->RxBoosted(timeout, sxHandler);
}

void lorahandler::Send(uint8_t *buffer, uint8_t size)
{
    //std::cout << "lorahandler::Send" << std::endl;


    radioHandler->Send(buffer, size, sxHandler);
}

void lorahandler::IrqProcess(uint8_t *dataReady)
{
    radioHandler->BgIrqProcess(dataReady, sxHandler);
}

/*void lorahandler::GetPayloadData(uint8_t& payload, uint16_t size)
{
    std::cout << "lorahandler::GetPayloadData" << std::endl;
    radioHandler->GetPayloadData(&payload, size);
    //std::cout << payload[0] << std::endl;
}*/

uint32_t lorahandler::HardwareInit()
{
    TimerConfig();

    sxHandler->DIOInit();

    // After power on the sync word should be 2414. 4434 could be possible on a restart
    // If we got something else, something is wrong.
    uint16_t readSyncWord = 0;
    sxHandler->ReadRegisters(REG_LR_SYNCWORD, (uint8_t *)&readSyncWord, 2);

    LOG_LIB("BRD", "SyncWord = %04X", readSyncWord);
    std::cout << "lora_hardware_init -> readSyncWord: " << readSyncWord << std::endl;

    if ((readSyncWord == 0x2414) || (readSyncWord == 0x4434))
    {
        return 0;
    }
    return 1;
}

uint32_t lorahandler::HardwareReInit()
{
    TimerConfig();

    sxHandler->DIOReInit();

    // After power on the sync word should be 2414. 4434 could be possible on a restart
    // If we got something else, something is wrong.
    uint16_t readSyncWord = 0;
    sxHandler->ReadRegisters(REG_LR_SYNCWORD, (uint8_t *)&readSyncWord, 2);

    LOG_LIB("BRD", "SyncWord = %04X", readSyncWord);

    if ((readSyncWord == 0x2414) || (readSyncWord == 0x4434))
    {
        return 0;
    }
    return 1;
}



RadioStatus_t lorahandler::Get_Status()
{
    RadioStatus_t st;
    st = sxDriver->GetStatus(sxHandler);

    return st;
}

RadioError_t lorahandler::Get_DeviceErrors()
{
    RadioError_t e;
    e = sxDriver->GetDeviceErrors(sxHandler);

    return e;
}






