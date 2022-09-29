#if defined ESP8266 || defined ESP32
#include <SPI.h>
#include "boards/mcu/board.h"

SPIClass SPI_LORA;

void initSPI(void)
{
#ifdef ESP8266
	SPI_LORA.pins(config_hw.PIN_LORA_SCLK, config_hw.PIN_LORA_MISO, config_hw.PIN_LORA_MOSI, config_hw.PIN_LORA_NSS);
	SPI_LORA.begin();
	SPI_LORA.setHwCs(false);
#else
	SPI_LORA.begin(config_hw.PIN_LORA_SCLK, config_hw.PIN_LORA_MISO, config_hw.PIN_LORA_MOSI, config_hw.PIN_LORA_NSS);
#endif
}
#endif