#ifndef DATAHELPER_H
#define DATAHELPER_H
#include <stdint.h>

enum CommandType {
    COMMAND_DEVICE_CONNECTED = 1,
    COMMAND_ACQUISITION_DATA = 5,
    COMMAND_WRITE_CONFIGURATION = 8
};

enum MessageType {
    MESSAGE_TYPE_REQUEST = 0,
    MESSAGE_TYPE_RESPONSE = 1
};

typedef enum {
    IRQ_ENABLE_TX_DONE = 0,
    IRQ_ENABLE_RX_DONE,
    IRQ_ENABLE_CRC_ERROR,
    IRQ_ENABLE_CAD_DONE,
    IRQ_ENABLE_RX_TX_TIMEOUT,
    IRQ_ENABLE_PREAMBLE_DETECTED,
    IRQ_ENABLE_SYNCWORD_VALID,
    IRQ_ENABLE_HEADER_VALID,
    IRQ_ENABLE_HEADER_ERROR
} IrqsActivated;

struct hw_config
{
    int PIN_LORA_RESET;				  // LORA RESET
    int PIN_LORA_NSS;				  // LORA SPI CS
    int PIN_LORA_SCLK;				  // LORA SPI CLK
    int PIN_LORA_MISO;				  // LORA SPI MISO
    int PIN_LORA_DIO_1;				  // LORA DIO_1
    int PIN_LORA_BUSY;				  // LORA SPI BUSY
    int PIN_LORA_MOSI;				  // LORA SPI MOSI
    int RADIO_TXEN = -1;			  // LORA ANTENNA TX ENABLE (eByte E22 module only)
    int RADIO_RXEN = -1;			  // LORA ANTENNA RX ENABLE (eByte E22 module only)
    bool USE_DIO2_ANT_SWITCH = false; // Whether DIO2 is used to control the antenna
    bool USE_DIO3_TCXO = false;		  // Whether DIO3 is used to control the oscillator
    bool USE_DIO3_ANT_SWITCH = false; // Whether DIO2 is used to control the antenna
    bool USE_LDO = false;			  // Whether SX126x uses LDO or DCDC power regulator
    bool USE_RXEN_ANT_PWR = false;	  // Whether RX_EN is used as antenna power
};

struct DataHelperDeviceConnected
{
    uint8_t type;
    uint8_t command;
    uint32_t deviceTag;
    uint8_t version;
    uint64_t timestamp;
    uint8_t deviceId;
    uint8_t jobId;
    uint16_t settingsId;
    uint8_t channel;
    uint16_t payload;
};
struct DataHelperResponse
{
    uint8_t type;
    uint8_t command;
    uint32_t deviceTag;
    uint8_t version;
    uint64_t timestamp;
    uint8_t deviceId;
    uint8_t jobId;
    uint16_t settingsId;
    uint8_t channel;
    uint16_t amp1;
    uint32_t tof1;
    uint16_t amp2;
    uint32_t tof2;
    uint16_t amp3;
    uint32_t tof3;
    uint8_t nSamples;
};

struct DataHelperRequest {
    uint8_t type;
    uint8_t command;
    uint32_t deviceTag;
    uint8_t version;
    uint64_t timestamp;
    uint8_t deviceId;
    uint8_t jobId;
    uint16_t settingsId;
    uint8_t channel;
};

double get_random(int min, int max);
void fill_buffer(uint8_t *buf, uint32_t *index, int size, uint64_t var);
void set_buffer_data(uint8_t *buf, DataHelperRequest *data_send);
uint64_t empty_buffer(uint8_t *buf, int len, int& index);


template <typename T>
T Abs(T x){
    if (x < 0) return -1*x;
        return x;
}

uint64_t Timestamp();
void sleep_milliseconds(double time);
void sleep_microseconds(double time);


#endif
