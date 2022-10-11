#include <string>
#include "lorahandler.h"
#include "SX126x/radio/sx126x/sx126x.h"
#include "SX126x/radio/radiohandler.h"
#include "SX126xHardware.h"
#include <iostream>

lorahandler::lorahandler()
{
    sxHandler = new SX126Handler();
    radioHandler = new RadioHandler();
    sxDriver = new SX126xDriver();

    sxHandler->sxDriver = sxDriver;
    radioHandler->sxHandler = sxHandler;
}

/*void lorahandler::TestSpi(SPILora *lora)
{
    uint8_t a[] = {0x23, 0x60};
    uint8_t b[] = {0, 0};

    lora->Transfer(a, b, 2);
}*/

void lorahandler::SetSpiLora(SPIBase *lora)
{
    sxHandler->SetSpiLora(lora);
}

void lorahandler::Init()
{
    radioHandler->Init(sxHandler);
}

void lorahandler::SetChannel(uint32_t freq)
{
    radioHandler->SetChannel(freq, sxHandler);
}

void lorahandler::SetTxConfig(int8_t power, uint32_t bandwidth, uint32_t datarate, uint8_t coderate, uint16_t preambleLen, bool fixLen, bool crcOn, bool iqInverted, uint32_t TxTimeout)
{
    radioHandler->SetTxConfig(power, bandwidth, datarate, coderate, preambleLen, fixLen, crcOn, iqInverted, TxTimeout, sxHandler);
}

void lorahandler::SetRxConfig(uint32_t bandwidth, uint32_t datarate, uint8_t coderate, uint16_t preambleLen, uint16_t symbTimeout, bool fixLen, uint8_t payloadLen, bool crcOn, bool iqInverted, bool rxContinuous, uint32_t RxTimeout)
{
    radioHandler->SetRxConfig(bandwidth, datarate, coderate, preambleLen, symbTimeout, fixLen, payloadLen, crcOn, iqInverted, rxContinuous, RxTimeout, sxHandler);
 }

void lorahandler::SetRx(uint32_t timeout)
{
    radioHandler->Rx(timeout, sxHandler);
}

void lorahandler::SetRxBoosted(uint32_t timeout)
{
    radioHandler->RxBoosted(timeout, sxHandler);
}

void lorahandler::Send(uint8_t *buffer, uint8_t size, uint32_t TxTimeout, uint32_t RxTimeout)
{
    //std::cout << "lorahandler::Send" << std::endl;

    radioHandler->Send(buffer, size, TxTimeout, RxTimeout, sxHandler);
}

void lorahandler::IrqProcess(uint8_t *dataReady)
{
    radioHandler->BgIrqProcess(dataReady, sxHandler);
}

uint32_t lorahandler::HardwareInit()
{
    sxHandler->DIOInit();

    // After power on the sync word should be 2414. 4434 could be possible on a restart
    // If we got something else, something is wrong.
    uint16_t readSyncWord = 0;
    sxHandler->ReadRegisters(REG_LR_SYNCWORD, (uint8_t *)&readSyncWord, 2);

    std::cout << "lora_hardware_init -> readSyncWord: " << readSyncWord << std::endl;

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






