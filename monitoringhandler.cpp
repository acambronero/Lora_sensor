#include "monitoringhandler.h"
#include "radiohandler.h"


MonitoringHandler::MonitoringHandler()
{
    lora = new lorahandler();
}

bool MonitoringHandler::Initialize()
{
    SPI_Lora = new SPILora(SPIDEV1);
    spi_config settings = {0, 8, int(10e6), 0};
    SPI_Lora->SetConfig(&settings);
    SPI_Lora->Begin();
    lora->SetSpiLora(SPI_Lora);
    lora_setup();
}

bool MonitoringHandler::IsInitialized()
{
    return this->Initialize();
}

void MonitoringHandler::lora_setup()
{
    std::cout << "setup" << std::endl;
    if ( lora->HardwareInit() < 0) {
        printf("Something is wrong !!\n");
    }

    //irqs.push_back(IRQ_ENABLE_PREAMBLE_DETECTED);
    irqs.push_back(IRQ_ENABLE_RX_DONE);
    irqs.push_back(IRQ_ENABLE_TX_DONE);

    lora->SetIrqsEnable(irqs);

    lora->Init();


    lora->SetChannel(RF_FREQUENCY);

    lora->SetTxConfig(TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                      LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                      LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                      true, 0, 0, LORA_IQ_INVERSION_ON, TX_TIMEOUT_VALUE);

    lora->SetRxConfig(LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                      LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                      LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                      0, true, 0, 0, LORA_IQ_INVERSION_ON, true);

    lora->SetRx(0);
}

void MonitoringHandler::Send(uint8_t *tx_buffer, uint16_t len)
{
    //std::cout << "MonitoringHandler::Send" << std::endl;

    //lora->radioHandler->Sleep(lora->sxHandler);
    //lora->radioHandler->sxHandler->Wakeup();
    //lora->radioHandler->sxHandler->Reset();


    /*lora->SetTxConfig(TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                      LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                      LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                      true, 0, 0, LORA_IQ_INVERSION_ON, TX_TIMEOUT_VALUE);*/

    lora->Send(tx_buffer, len);



    //SetRx();

}

void MonitoringHandler::Run()
{
    lora->IrqProcess(&dataReady);
}

/*void MonitoringHandler::GetPayloadData(uint8_t *payload, uint16_t size)
{
    std::cout << "MonitoringHandler::GetPayloadData" << std::endl;
    //lora->GetPayloadData(payload, size);
    lora->radioHandler->GetPayloadData(payload, size);
    std::cout << payload[0] << std::endl;
}*/

void MonitoringHandler::SetRx(){
    lora->SetRx(0);
}
