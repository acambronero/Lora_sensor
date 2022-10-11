#include "datahelper.h"
#include <math.h>
#include <thread>
#include <chrono>

double get_random(int min, int max){
    return rand() * (max - min) + min;
}

void fill_buffer(uint8_t *buf, uint32_t *index, int size, uint64_t var){
    for (size_t i = 0; i < size; i++){
        buf[(*index)++] = (uint8_t)var >> (8 * i) & 0xFF;
    }
}

void set_buffer_data(uint8_t *buf, DataHelperRequest *data_send){
    uint32_t index = 0;
    fill_buffer(buf, &index, 1, 0x46);
    fill_buffer(buf, &index, 1, 0x53);
    fill_buffer(buf, &index, 1, data_send->type);
    fill_buffer(buf, &index, 1, data_send->command);
    fill_buffer(buf, &index, 4, data_send->deviceTag);
    fill_buffer(buf, &index, 4, data_send->version);
    fill_buffer(buf, &index, 4, data_send->timestamp);
    fill_buffer(buf, &index, 1, data_send->deviceId);
    fill_buffer(buf, &index, 1, data_send->jobId);
    fill_buffer(buf, &index, 1, data_send->settingsId);
    fill_buffer(buf, &index, 1, data_send->channel);
    fill_buffer(buf, &index, 1, 0x0D);
    fill_buffer(buf, &index, 1, 0x0A);

    //BufferSize = index;
}


uint64_t empty_buffer(uint8_t *buf, int len, int& index){
    uint64_t recv_data = 0;
    for (size_t i = 0; i < len; i++){
        recv_data = recv_data << (8 * i);
        recv_data = buf[index++];
    }
    return recv_data;
}


/*template <typename T>
T Abs(T x){
    if (x < 0) return -1*x;
    return x;
}*/

uint64_t Timestamp(){
    std::chrono::milliseconds ms = std::chrono::duration_cast< std::chrono::milliseconds >( std::chrono::system_clock::now().time_since_epoch() );
    uint64_t num = ms.count();
    return num;
}

#ifndef ARDUINO
void sleep_milliseconds(double time){
    #ifndef ARDUINO
        std::this_thread::sleep_for( std::chrono::milliseconds((long long)time));
#else
    delay(time);

#endif
}

void sleep_microseconds(double time){
#ifndef ARDUINO
    std::this_thread::sleep_for (std::chrono::microseconds((long long)time));
#else
    delayMicroseconds(time);

#endif
}
#endif

void println(std::string msg)
{
#ifdef ARDUINO
    Serial.println(msg.c_str());
#else
    std::cout << msg.c_str() << std::endl;
#endif
}
