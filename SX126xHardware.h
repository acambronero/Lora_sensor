#ifndef SX126XHARDWARE_H
#define SX126XHARDWARE_H
#include "stdint.h"
#include "SX126x/radio/sx126x/sx126x.h"
#include "spibase.h"

class SX126Handler {

public:
    SX126Handler();

    SPIBase* spiLora;

    SX126xDriver *sxDriver;

    void SetSpiLora(SPIBase* spiLora){
        this->spiLora = spiLora;
    }

    SPIBase* GetSPI();

/*!
 * @brief Initializes the radio I/Os pins interface
 */
    void DIOInit(void);

    void ConfigureGPIO();

    uint8_t ReadGPIO();

    void WriteGPIO(uint8_t value);

/*!
 * @brief De-initializes the radio I/Os pins interface.
 *
 * \remark Useful when going in MCU low power modes
 */
void IoDeInit(void);

/*!
 * @brief HW Reset of the radio
 */
void Reset(void);

/*!
 * @brief Blocking loop to wait while the Busy pin in high
 */
void WaitOnBusy(void);

/*!
 * @brief Wakes up the radio
 */
void Wakeup(void);

/*!
 * @brief Send a command that write data to the radio
 *
 * \param   opcode        Opcode of the command
 * \param   buffer        Buffer to be send to the radio
 * \param   size          Size of the buffer to send
 */
void WriteCommand(RadioCommands_t opcode, uint8_t *buffer, uint16_t size);

/*!
 * @brief Send a command that read data from the radio
 *
 * \param   opcode        Opcode of the command
 * \param  buffer        Buffer holding data from the radio
 * \param   size          Size of the buffer
 */
void ReadCommand(RadioCommands_t opcode, uint8_t *buffer, uint16_t size);

/*!
 * \brief Write data to the radio memory
 *
 * \param   address       The address of the first byte to write in the radio
 * \param   buffer        The data to be written in radio's memory
 * \param   size          The number of bytes to write in radio's memory
 */
void WriteRegisters(uint16_t address, uint8_t *buffer, uint16_t size);

/*!
 * @brief Write a single byte of data to the radio memory
 *
 * \param   address       The address of the first byte to write in the radio
 * \param   value         The data to be written in radio's memory
 */
void WriteRegister(uint16_t address, uint8_t value);

/*!
 * \brief Read data from the radio memory
 *
 * \param   address       The address of the first byte to read from the radio
 * \param  buffer        The buffer that holds data read from radio
 * \param   size          The number of bytes to read from radio's memory
 */
void ReadRegisters(uint16_t address, uint8_t *buffer, uint16_t size);

/*!
 * @brief Read a single byte of data from the radio memory
 *
 * \param   address       The address of the first byte to write in the radio
 *
 * \retval      value         The value of the byte at the given address in radio's memory
 */
uint8_t ReadRegister(uint16_t address);

/*!
 * \brief Write data to the buffer holding the payload in the radio
 *
 * \param   offset        The offset to start writing the payload
 * \param   buffer        The data to be written (the payload)
 * \param   size          The number of byte to be written
 */
void WriteBuffer(uint8_t offset, uint8_t *buffer, uint8_t size);

/*!
 * \brief Read data from the buffer holding the payload in the radio
 *
 * \param   offset        The offset to start reading the payload
 * \param  buffer        A pointer to a buffer holding the data from the radio
 * \param   size          The number of byte to be read
 */
void ReadBuffer(uint8_t offset, uint8_t* &buffer, uint8_t size);

/*!
 * @brief Sets the radio output power.
 *
 * \param  power Sets the RF output power
 */
void SetRfTxPower(int8_t power);

/*!
 * @brief Gets the board PA selection configuration
 *
 * \param  channel Channel frequency in Hz
 * \retval PaSelect RegPaConfig PaSelect value
 */
uint8_t GetPaSelect(uint32_t channel);

/*!
 * @brief Gets info on the number of packets received
 *
 * \param  nb_pkt_received     Number of received packets with CRC OK
 * \param  nb_pkt_crc_error    Number of received packets with CRC error
 * \param  nb_pkt_length_error Number of received packets with length error
 */
void GetStats(uint16_t *nb_pkt_received, uint16_t *nb_pkt_crc_error, uint16_t *nb_pkt_length_error);

/*!
 * @brief Resets values read by GetStats
 */
void ResetStats(void);

protected:


};

#endif // SX126XHARDWARE_H
