#ifndef SPIBASE_H
#define SPIBASE_H

#include <stdint.h>
#include <string>
#include "datahelper.h"
#ifdef ARDUINO
#include <SPI.h>
#endif

#define SPIDEV0 "/dev/spidev32766.0"
#define SPIDEV1 "/dev/spidev0.0"
#define SPIDEV2 "/dev/spidev0.1"

typedef struct {
    uint8_t mode;
    uint8_t numBits_Word;
    uint32_t speed;
    uint16_t delay;
} spi_config;

enum SPI_MODE {
    SPI_MODE0 = 0x00,   // rest = 0, latch on rise
    SPI_MODE1 = 0x01,  // rest = 0, latch on fall
    SPI_MODE2 = 0x02,  // rest = 1, latch on fall
    SPI_MODE3 = 0x03  // rest = 1, latch on rise
};

class SPIBase
{
protected:
    std::string spi_dev;
    int spi_fd;
    spi_config spi_settings;
#ifdef ARDUINO
    SPIClass spiLora;
    SPISettings spiSettings;
#endif
    hw_config config_hw;
    bool spi_open = false;

public:
    SPIBase();
    virtual bool IsInitialized()=0;
    virtual bool Begin()=0;
    virtual int Transfer(uint8_t *p_txbuffer, uint8_t *p_rxbuffer, uint16_t len)=0;
    virtual int Write(uint8_t *p_txbuffer, uint16_t len)=0;
    virtual int Read(uint8_t *p_rxbuffer, uint16_t len)=0;
    virtual bool SetConfig(spi_config *settings)=0;
    virtual bool SetMode(uint8_t mode)=0;
    virtual bool SetSpeed(uint32_t speed)=0;
    virtual bool SetBitsPerWord(uint8_t bits_per_word)=0;

    bool debug = false;
};

#endif // SPIBASE_H
