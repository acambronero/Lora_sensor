#ifndef SPIARDUINO_H
#define SPIARDUINO_H

#ifdef ARDUINO
#include <SPI.h>
#endif
#include "spibase.h"
#include "datahelper.h"

class SPIArduino : public SPIBase
{
public:
    SPIArduino(hw_config config);
    virtual bool IsInitialized();
    virtual bool Begin();
    virtual int Transfer(uint8_t *p_txbuffer, uint8_t *p_rxbuffer, uint16_t len);
    virtual int Write(uint8_t *p_txbuffer, uint16_t len);
    virtual int Read(uint8_t *p_rxbuffer, uint16_t len);
    virtual bool SetConfig(spi_config *settings);
    virtual bool SetMode(uint8_t mode);
    virtual bool SetSpeed(uint32_t speed);
    virtual bool SetBitsPerWord(uint8_t bits_per_word);
protected:
};

#endif // SPIARDUINO_H
