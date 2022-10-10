#include "spiarduino.h"

SPIArduino::SPIArduino(hw_config config) {
#ifdef ARDUINO
    spiSettings = SPISettings(2000000, MSBFIRST, SPI_MODE0);
    config_hw = config;
#endif
}

bool SPIArduino::IsInitialized() {
    return spi_open;
}

bool SPIArduino::Begin() {
#ifdef ARDUINO
    SPI_LORA.pins(config_hw.PIN_LORA_SCLK, config_hw.PIN_LORA_MISO, config_hw.PIN_LORA_MOSI, config_hw.PIN_LORA_NSS);
    SPI_LORA.begin();
    SPI_LORA.setHwCs(false);
#endif
}

int SPIArduino::Transfer(uint8_t *p_txbuffer, uint8_t *p_rxbuffer, uint16_t len) {
#ifdef ARDUINO
    digitalWrite(config_hw.PIN_LORA_NSS, LOW);

    SPI_LORA.beginTransaction(spiSettings);

    for (uint16_t i = 0; i < len; i++)
    {
        p_rxbuffer[i] = SPI_LORA.transfer(p_txbuffer[i]);
    }

    SPI_LORA.endTransaction();
    digitalWrite(_hwConfig.PIN_LORA_NSS, HIGH);
#endif
}

int SPIArduino::Write(uint8_t *p_txbuffer, uint16_t len) {
#ifdef ARDUINO
    digitalWrite(config_hw.PIN_LORA_NSS, LOW);

    SPI_LORA.beginTransaction(spiSettings);

    for (uint16_t i = 0; i < len; i++)
    {
        SPI_LORA.transfer(p_txbuffer[i]);
    }

    SPI_LORA.endTransaction();
    digitalWrite(config_hw.PIN_LORA_NSS, HIGH);
#endif
}

int SPIArduino::Read(uint8_t *p_rxbuffer, uint16_t len)
{

}

bool SPIArduino::SetConfig(spi_config *settings)
{

}

bool SPIArduino::SetMode(uint8_t mode)
{

}

bool SPIArduino::SetSpeed(uint32_t speed)
{

}

bool SPIArduino::SetBitsPerWord(uint8_t bits_per_word)
{

}