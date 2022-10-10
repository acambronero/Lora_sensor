#ifndef MONITORINGHANDLER_H
#define MONITORINGHANDLER_H

#include "spibase.h"
#include "lorahandler.h"
//#include "itools.h"
#include <string>
#include <vector>

#define RF_FREQUENCY 868000000	// Hz
#define TX_OUTPUT_POWER 22		// dBm
#define LORA_BANDWIDTH 1		// [0: 125 kHz, 1: 250 kHz, 2: 500 kHz, 3: Reserved]
#define LORA_SPREADING_FACTOR 5 // [SF7..SF12]
#define LORA_CODINGRATE 1		// [1: 4/5, 2: 4/6,  3: 4/7,  4: 4/8]
#define LORA_PREAMBLE_LENGTH 12	// Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT 0	// Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false
#define RX_TIMEOUT_VALUE 5000
#define TX_TIMEOUT_VALUE 5000

#define BUFFER_SIZE 127 // Define the payload size here


class MonitoringHandler
{
public:
    MonitoringHandler();
    bool Initialize();
    bool IsInitialized();
    void lora_setup();
    void Send(uint8_t *tx_buffer, uint16_t len, uint32_t TxTimeout, uint32_t RxTimeout);
    void decode_message();
    void Run();
    void CheckSerialData();
    void GetPayloadData(uint8_t *payload, uint16_t size);
    void SetRx(uint32_t timeout);

    lorahandler *lora;
    bool firstTime = true;
    uint8_t dataReady = 0;
    uint8_t sendReady = 0;
    uint32_t TxTimeout = TX_TIMEOUT_VALUE;
    uint32_t RxTimeout = RX_TIMEOUT_VALUE;

protected:

    SPIBase *SPI_Lora;
    std::vector<IrqsActivated> irqs;

    uint16_t BufferSize;
    uint8_t RcvSerialBuffer[BUFFER_SIZE];
    uint8_t RcvBuffer[BUFFER_SIZE];
    uint8_t TxdBuffer[BUFFER_SIZE];

    uint8_t message_type = 0;
    uint8_t command = 0;
    uint32_t deviceTag = 0;
    uint8_t version = 0;
    uint64_t timestamp = 0;
    uint8_t deviceId = 0;


};

#endif // MONITORINGHANDLER_H
