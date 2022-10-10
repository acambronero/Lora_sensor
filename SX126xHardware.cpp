#include "SX126xHardware.h"
#include "spilora.h"
#include <algorithm>
#include <iostream>
#include <chrono>
#include <thread>
#include <string>
#include <fstream>

#ifdef RASPI
#include "wiringPi.h"
#endif
#ifdef ARDUINO
#include <SPI.h>
#endif

#define BUSY 24
#define DIO1 25
#define RESET 22


class lorahandler;

SX126Handler::SX126Handler()
{

}

SPIBase* SX126Handler::GetSPI(){
    return this->spiLora;
}

void SX126Handler::DIOInit(void)
{
#ifdef RASPI
    wiringPiSetup();
    pinMode(DIO1, INPUT);
    pinMode(BUSY, INPUT);
    pinMode(RESET, OUTPUT);
    Reset();
#elif defined ARDUINO
    pinMode(_hwConfig.PIN_LORA_NSS, OUTPUT);
    digitalWrite(_hwConfig.PIN_LORA_NSS, HIGH);
    pinMode(_hwConfig.PIN_LORA_BUSY, INPUT);
    pinMode(_hwConfig.PIN_LORA_DIO_1, INPUT);
    pinMode(_hwConfig.PIN_LORA_RESET, OUTPUT);
    Reset();
#else
    ConfigureGPIO();
    Reset();
#endif

}

void SX126Handler::ConfigureGPIO()
{
   //configure reset
   system("echo 973 > /sys/class/gpio/export");
   system("echo in > /sys/class/gpio/gpio973/direction");

   //configure Busy
   system("echo 974 > /sys/class/gpio/export");
   system("echo out > /sys/class/gpio/gpio974/direction");
}

uint8_t SX126Handler::ReadGPIO()
{
    std::string data;
    system("cat /sys/class/gpio/gpio974/value >> output.txt");
    std::ifstream output("output.txt");
    getline(output, data);

    return std::stoi(data);

}

void SX126Handler::WriteGPIO(uint8_t value)
{
    std::string data = "echo " + std::to_string(value) + " > /sys/class/gpio/gpio973/value";
    system(data.c_str());
}

void SX126Handler::Reset(void)
{
#if defined RASPI || defined ARDUINO
    digitalWrite(RESET, LOW);
    digitalWrite(RESET, HIGH);
#else
    WriteGPIO(0);
    WriteGPIO(1);
#endif
}

void SX126Handler::WaitOnBusy(void)
{
    auto start = std::chrono::steady_clock::now();
#if defined RASPI || defined ARDUINO
    while (digitalRead(BUSY) == HIGH)
#else
    while (ReadGPIO() == 1)
#endif
    {
        auto end = std::chrono::steady_clock::now();
        auto diff = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        //if(std::chrono::duration_cast<std::chrono::microseconds>(diff).count() > 5000){
        if(std::chrono::duration_cast<std::chrono::microseconds>(diff).count() > 50000){
            std::cout << "BUSY TIMEOUT" << std::endl;
            return;
        }
    }
}

void SX126Handler::Wakeup(void)
{
    uint8_t tx_buffer[] = {0xC0, 0x00};
    uint8_t rx_buffer[] = {0, 0};

    spiLora->Transfer(tx_buffer, rx_buffer, 2);

    // Wait for chip to be ready.
    WaitOnBusy();

}

void SX126Handler::WriteCommand(RadioCommands_t command, uint8_t *buffer, uint16_t size)
{
    //std::cout << "SX126Handler::WriteCommand" << std::endl;
    sxDriver->CheckDeviceReady(this);

    uint8_t tx_buf[size + 1];
    tx_buf[0] = (uint8_t)command;

    for (uint16_t i = 0; i < size; i++)
    {
        tx_buf[i + 1] = buffer[i];
    }

    spiLora->Write(tx_buf, size+1);

    if (command != RADIO_SET_SLEEP)
    {
        WaitOnBusy();
    }
}

void SX126Handler::ReadCommand(RadioCommands_t command, uint8_t *buffer, uint16_t size)
{
    sxDriver->CheckDeviceReady(this);

    uint8_t buf[size + 2];
    uint8_t aux_buffer[size + 2];

    /*for (uint8_t i = 0; i < size + 1; i++){
        buf[i] = 0;
        aux_buffer[i] = 0;
    }
    aux_buffer[size+2] = 0;*/

    buf[0] = (uint8_t)command;
    buf[1] = (uint8_t)0x00;

    if (command == RADIO_GET_STATUS){
        spiLora->Transfer(buf, aux_buffer, size+1);
        buffer[0] = aux_buffer[0];
    }else{
        for (uint16_t i = 0; i < size; i++)
        {
            buf[i + 2] = 0x00;
        }
        spiLora->Transfer(buf, aux_buffer, size+2);
        for (uint8_t j = 0; j < size; j++){
            buffer[j] = aux_buffer[j+2];
        }
    }
    WaitOnBusy();
}

void SX126Handler::WriteRegisters(uint16_t address, uint8_t *buffer, uint16_t size)
{
    sxDriver->CheckDeviceReady(this);

    uint8_t buf[size + 3];

    buf[0] = (uint8_t)RADIO_WRITE_REGISTER;
    buf[1] = ((address & 0xFF00) >> 8);
    buf[2] = (address & 0x00FF);

    for (uint16_t i = 0; i < size; i++)
    {
        buf[i + 3] = buffer[i];
    }

    spiLora->Write(buf, size+3);

    WaitOnBusy();
}

void SX126Handler::WriteRegister(uint16_t address, uint8_t value)
{
    WriteRegisters(address, &value, 1);
}

void SX126Handler::ReadRegisters(uint16_t address, uint8_t *buffer, uint16_t size)
{

    sxDriver->CheckDeviceReady(this);

    uint8_t buf[size + 4];
    uint8_t aux[size + 4];

    for (uint8_t i = 0; i < size + 4; i++){
        buf[i] = 0;
        aux[i] = 0;
    }

    buf[0] = (uint8_t)RADIO_READ_REGISTER;
    buf[1] = ((address & 0xFF00) >> 8);
    buf[2] = (address & 0x00FF);
    buf[3] = 0x00;
    for (uint16_t i = 0; i < size; i++)
    {
        buf[i+4] = 0x00;
    }

    spiLora->Transfer(buf, aux, size+4);

    for (uint16_t j = 0; j < size; j++){
        buffer[j] = aux[j+4];
    }

    WaitOnBusy();
}

uint8_t SX126Handler::ReadRegister(uint16_t address)
{
    uint8_t data;
    ReadRegisters(address, &data, 1);
    return data;
}

void SX126Handler::WriteBuffer(uint8_t offset, uint8_t *buffer, uint8_t size)
{
    //std::cout << "SX126Handler::WriteBuffer" << std::endl;
    sxDriver->CheckDeviceReady(this);

    uint8_t buf[size + 2];

    buf[0] = (uint8_t)(RADIO_WRITE_BUFFER);
    buf[1] = (offset);
    for (uint16_t i = 0; i < size; i++)
    {
        buf[i + 2] = buffer[i];
    }

    spiLora->Write(buf, size+2);

    WaitOnBusy();
}

void SX126Handler::ReadBuffer(uint8_t offset, uint8_t* &buffer, uint8_t size)
{
    sxDriver->CheckDeviceReady(this);

    uint8_t buf[size + 3];
    uint8_t aux_buffer[size +3];

    buf[0] = (uint8_t)(RADIO_READ_BUFFER);
    buf[1] = (offset);
    buf[2] = 0x00;
    for (uint16_t i = 0; i < size; i++)
    {
        buf[i + 3] = 0x00;
    }

    spiLora->Transfer(buf, aux_buffer, size+3);

    for (uint8_t j = 0; j < size; j++)
        buffer[j] = aux_buffer[j + 3];

    WaitOnBusy();
}

void SX126Handler::SetRfTxPower(int8_t power)
{
    sxDriver->SetTxParams(power, RADIO_RAMP_40_US, this);
}

uint8_t SX126Handler::GetPaSelect(uint32_t channel)
{
    return SX1262;
}

void SX126Handler::GetStats(uint16_t *nb_pkt_received, uint16_t *nb_pkt_crc_error, uint16_t *nb_pkt_length_error)
{
    uint8_t buf[6];

    ReadCommand(RADIO_GET_STATS, buf, 6);

    *nb_pkt_received = (buf[0] << 8) | buf[1];
    *nb_pkt_crc_error = (buf[2] << 8) | buf[3];
    *nb_pkt_length_error = (buf[4] << 8) | buf[5];
}

void SX126Handler::ResetStats(void)
{
    uint8_t buf[6] = {0x00};

    WriteCommand(RADIO_RESET_STATS, buf, 6);
}
