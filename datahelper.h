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
