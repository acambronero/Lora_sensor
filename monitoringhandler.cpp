#include "monitoringhandler.h"
#include "lorahandler.h"
#ifdef ARDUINO
#include "spiarduino.h"
#include "Arduino.h"
#else
#include "spilora.h"
#endif

MonitoringHandler::MonitoringHandler()
{
    lora = new LoraHandler();
}

bool MonitoringHandler::Initialize()
{
#ifdef ARDUINO
    hw_config config_hw;
    config_hw.PIN_LORA_RESET = 0;
    config_hw.PIN_LORA_DIO_1 = 5;
    config_hw.PIN_LORA_BUSY = 4;
    config_hw.PIN_LORA_NSS = 15;
    config_hw.PIN_LORA_SCLK = 14;
    config_hw.PIN_LORA_MISO = 12;
    config_hw.PIN_LORA_MOSI = 13;
    SPI_Lora = new SPIArduino(config_hw);
#else
    SPI_Lora = new SPILora(SPIDEV1);
    spi_config settings = {0, 8, int(10e6), 0};
    SPI_Lora->SetConfig(&settings);
    SPI_Lora->Begin();
#endif
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

    lora->Init();


    lora->SetChannel(RF_FREQUENCY);

    lora->SetTxConfig(TX_OUTPUT_POWER, LORA_BANDWIDTH,
                      LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                      LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                      true, LORA_IQ_INVERSION_ON, TX_TIMEOUT_VALUE);

    lora->SetRxConfig(LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                      LORA_CODINGRATE, LORA_PREAMBLE_LENGTH,
                      LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                      0, true, LORA_IQ_INVERSION_ON, false, RX_TIMEOUT_VALUE);

    sendReady = 1;
}

void MonitoringHandler::Send(uint8_t *tx_buffer, uint16_t len, uint32_t TxTimeout, uint32_t RxTimeout)
{
    //std::cout << "MonitoringHandler::Send" << std::endl;

    lora->Send(tx_buffer, len, TxTimeout, RxTimeout);
}

void MonitoringHandler::decode_message()
{
    int index = 0;
    std::string SFD;
    SFD.push_back((char)empty_buffer(RcvBuffer, 1, index));
    SFD.push_back((char)empty_buffer(RcvBuffer, 1, index));

    if (SFD != "FS") return;

    /*message_type = (uint8_t)empty_buffer(RcvBuffer, 1, index);
    command = (uint8_t)empty_buffer(RcvBuffer, 1, index);
    deviceTag = (uint32_t) empty_buffer(RcvBuffer, 4, index);
    version = (uint8_t)empty_buffer(RcvBuffer, 1, index);
    timestamp = empty_buffer(RcvBuffer, 8, index);
    deviceId = (uint8_t)empty_buffer(RcvBuffer, 1, index);*/

    switch (command)
    {
        case COMMAND_ACQUISITION_DATA:
        {
            //DataHelperResponse msg;
            //prepare_acquisition_data(msg);
            //set_buffer_data(TxdBuffer, &msg);
            break;
        }
        case COMMAND_WRITE_CONFIGURATION:
        {
            DataHelperRequest msg = {
                (uint8_t)MESSAGE_TYPE_REQUEST,
                (uint8_t)COMMAND_ACQUISITION_DATA,
                0x260B269E,
                0x1,
                0x260B269EDE32ABCE,
                0x1,
            };
            set_buffer_data(TxdBuffer, &msg);
            break;
        }
        default:
            break;
    }

}

void MonitoringHandler::Run()
{
    lora->IrqProcess(&dataReady);
}

std::array<uint8_t, 255> MonitoringHandler::GetPayloadData(uint16_t size)
{

    std::array<uint8_t, 255> data;
    //std::copy(std::begin(payload), std::end(payload), std::begin(data));
    data = lora->GetPayloadData(size);
    return data;
}

void MonitoringHandler::CheckSerialData()
{
#ifdef ARDUINO
    if (!Serial.available()) return;
    if (!Serial.readBytesUntil('\r', RcvSerialBuffer, BUFFER_SIZE)) return;

    //lora.prepare_message();
    //lora.Send(lora.TxdBuffer, lora.BufferSize);
#endif

}

void MonitoringHandler::SetRx(uint32_t timeout){
    lora->SetRx(timeout);
}
