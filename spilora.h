#ifndef SPILORA_H
#define SPILORA_H

#include <spibase.h>
#include <fcntl.h>
#include <unistd.h>

#ifndef ARDUINO
#include <sys/ioctl.h>
#endif
#include <linux/spi/spidev.h>
#include <iostream>
#include "string.h"
#include "linux/types.h"
#include "errno.h"


#define DELAY_TX   50

class SPILora : public SPIBase
{
public:
    SPILora(std::string spi_p);
    SPILora(std::string spi_p, spi_config *local_settings);
    ~SPILora();
    virtual bool IsInitialized();
    virtual bool Begin();
    virtual int Transfer(uint8_t *p_txbuffer, uint8_t *p_rxbuffer, uint16_t len);
    virtual int Write(uint8_t *p_txbuffer, uint16_t len);
    virtual int Read(uint8_t *p_rxbuffer, uint16_t len);
    virtual bool SetConfig(spi_config *settings);
    virtual bool SetMode(uint8_t mode);
    virtual bool SetSpeed(uint32_t speed);
    virtual bool SetBitsPerWord(uint8_t bits_per_word);
    bool debug = false;
};

#endif // SPILORA_H
