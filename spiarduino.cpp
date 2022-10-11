#include "spiarduino.h"

SPIArduino::SPIArduino(): SPIBase()
{
    config_hw.PIN_LORA_RESET = 0;
    config_hw.PIN_LORA_DIO_1 = 5;
    config_hw.PIN_LORA_BUSY = 4;
    config_hw.PIN_LORA_NSS = 15;
    config_hw.PIN_LORA_SCLK = 14;
    config_hw.PIN_LORA_MISO = 12;
    config_hw.PIN_LORA_MOSI = 13;

#ifdef ARDUINO
    spiSettings = new SPISettings(2000000, MSBFIRST, SPI_MODE0);
    spiLora = new SPIClass();
#endif
}

bool SPIArduino::IsInitialized() {
    return pins_ok;
}

bool SPIArduino::Begin() {
#ifdef ARDUINO
    pins_ok = spiLora->pins(config_hw.PIN_LORA_SCLK, config_hw.PIN_LORA_MISO, config_hw.PIN_LORA_MOSI, config_hw.PIN_LORA_NSS);
    spiLora->begin();
    spiLora->setHwCs(false);
#endif
    return pins_ok;
}

int SPIArduino::Transfer(uint8_t *p_txbuffer, uint8_t *p_rxbuffer, uint16_t len) {
#ifdef ARDUINO
    digitalWrite(config_hw.PIN_LORA_NSS, LOW);

    spiLora->beginTransaction(*spiSettings);

    for (uint16_t i = 0; i < len; i++) {
        p_rxbuffer[i] = spiLora->transfer(p_txbuffer[i]);
    }

    spiLora->endTransaction();
    digitalWrite(config_hw.PIN_LORA_NSS, HIGH);
#endif
    return 0;
}

int SPIArduino::Write(uint8_t *p_txbuffer, uint16_t len) {
#ifdef ARDUINO
    digitalWrite(config_hw.PIN_LORA_NSS, LOW);

    spiLora->beginTransaction(*spiSettings);

    for (uint16_t i = 0; i < len; i++) {
        spiLora->transfer(p_txbuffer[i]);
    }

    spiLora->endTransaction();
    digitalWrite(config_hw.PIN_LORA_NSS, HIGH);
#endif
    return 0;
}

int SPIArduino::Read(uint8_t *p_rxbuffer, uint16_t len)
{
   return 0;
}

bool SPIArduino::SetConfig(spi_config *settings)
{
  return true;
}

bool SPIArduino::SetMode(uint8_t mode)
{
   return true;
}

bool SPIArduino::SetSpeed(uint32_t speed)
{
  return true;
}

bool SPIArduino::SetBitsPerWord(uint8_t bits_per_word)
{
return true;
}
