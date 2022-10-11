#include "lorahandler.h"
#include "sx126x.h"
#include "radiohandler.h"
#include "SX126xHardware.h"

LoraHandler::LoraHandler()
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

void LoraHandler::SetSpiLora(SPIBase *lora)
{
    sxHandler->SetSpiLora(lora);
}

void LoraHandler::Init()
{
    radioHandler->Init(sxHandler);
}

void LoraHandler::SetChannel(uint32_t freq)
{
    radioHandler->SetChannel(freq, sxHandler);
}

void LoraHandler::SetTxConfig(int8_t power, uint32_t bandwidth, uint32_t datarate, uint8_t coderate, uint16_t preambleLen, bool fixLen, bool crcOn, bool iqInverted, uint32_t TxTimeout)
{
    radioHandler->SetTxConfig(power, bandwidth, datarate, coderate, preambleLen, fixLen, crcOn, iqInverted, TxTimeout, sxHandler);
}

void LoraHandler::SetRxConfig(uint32_t bandwidth, uint32_t datarate, uint8_t coderate, uint16_t preambleLen, uint16_t symbTimeout, bool fixLen, uint8_t payloadLen, bool crcOn, bool iqInverted, bool rxContinuous, uint32_t RxTimeout)
{
    radioHandler->SetRxConfig(bandwidth, datarate, coderate, preambleLen, symbTimeout, fixLen, payloadLen, crcOn, iqInverted, rxContinuous, RxTimeout, sxHandler);
 }

void LoraHandler::SetRx(uint32_t timeout)
{
    radioHandler->Rx(timeout, sxHandler);
}

void LoraHandler::SetRxBoosted(uint32_t timeout)
{
    radioHandler->RxBoosted(timeout, sxHandler);
}

void LoraHandler::Send(uint8_t *buffer, uint8_t size, uint32_t TxTimeout, uint32_t RxTimeout)
{
    //std::cout << "lorahandler::Send" << std::endl;

    radioHandler->Send(buffer, size, TxTimeout, RxTimeout, sxHandler);
}

void LoraHandler::IrqProcess(uint8_t *dataReady)
{
    radioHandler->BgIrqProcess(dataReady, sxHandler);
}

std::array<uint8_t, 255> LoraHandler::GetPayloadData(uint16_t size)
{
    std::array<uint8_t, 255> data;
    data = radioHandler->GetPayloadData(size);


    std::cout << "Data Received: " << std::endl;
    std::cout << data[0] << std::endl;
    std::cout << data[1] << std::endl;
    std::cout << data[2] << std::endl;
    std::cout << data[3] << std::endl;

}

uint32_t LoraHandler::HardwareInit()
{
    sxHandler->DIOInit();

    // After power on the sync word should be 2414. 4434 could be possible on a restart
    // If we got something else, something is wrong.
    uint16_t readSyncWord = 0;
    sxHandler->ReadRegisters(REG_LR_SYNCWORD, (uint8_t *)&readSyncWord, 2);

    std::cout << "lora_hardware_init -> readSyncWord: " << std::hex << readSyncWord << std::endl;

    if ((readSyncWord == 0x2414) || (readSyncWord == 0x4434)){
        return 1;
    }
    return 0;
}

RadioStatus_t LoraHandler::Get_Status()
{
    RadioStatus_t st;
    st = sxDriver->GetStatus(sxHandler);

    return st;
}

RadioError_t LoraHandler::Get_DeviceErrors()
{
    RadioError_t e;
    e = sxDriver->GetDeviceErrors(sxHandler);

    return e;
}






