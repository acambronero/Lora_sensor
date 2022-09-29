/*!
 * \file      radio.h
 *
 * \brief     Radio driver API definition
 *
 * \copyright Revised BSD License, see file LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 */
#ifndef __RADIO2_H__
#define __RADIO2_H__
#include "stdint.h"
#include "mcu/timer.h"
#include "SX126xHardware.h"
class SX126Handler;

/*!
 * Radio driver supported modems
 */
typedef enum
{
	MODEM_FSK = 0,
	MODEM_LORA,
} RadioModems_t;

/*!
 * Radio driver internal state machine states definition
 */
typedef enum
{
	RF_IDLE = 0,   //!< The radio is idle
	RF_RX_RUNNING, //!< The radio is in reception state
	RF_TX_RUNNING, //!< The radio is in transmission state
	RF_CAD,		   //!< The radio is doing channel activity detection
} RadioState_t;

/*!
 * Holds the current network type for the radio
 */
typedef struct
{
    bool Previous;
    bool Current;
} RadioPublicNetwork_t;

/*!
 * \brief Radio driver callback functions
 */
typedef struct
{
	/*!
     * \brief  Tx Done callback prototype.
     */
	void (*TxDone)(void);
	/*!
     * \brief  Tx Timeout callback prototype.
     */
	void (*TxTimeout)(void);
	/*!
     * \brief Rx Done callback prototype.
     *
     * \param  payload Received buffer pointer
     * \param  size    Received buffer size
     * \param  rssi    RSSI value computed while receiving the frame [dBm]
     * \param  snr     SNR value computed while receiving the frame [dB]
     *                     FSK : N/A ( set to 0 )
     *                     LoRa: SNR value in dB
     */
	void (*RxDone)(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);
	/*!
     * \brief  Rx Timeout callback prototype.
     */
	void (*RxTimeout)(void);
	/*!
     * \brief Rx Error callback prototype.
     */
	void (*RxError)(void);
	/*!
     * \brief Preamble detected callback prototype.
     */
	void (*PreAmpDetect)(void);
	/*!
     * \brief  FHSS Change Channel callback prototype.
     *
     * \param  currentChannel   Index number of the current channel
     */
	void (*FhssChangeChannel)(uint8_t currentChannel);

	/*!
     * \brief CAD Done callback prototype.
     *
     * \param  channelDetected    Channel Activity detected during the CAD
     */
	void (*CadDone)(bool channelActivityDetected);
} RadioEvents_t;

/*!
 * \brief Radio driver definition
 */
class RadioHandler
{

public:
    RadioHandler();
    void Init(RadioEvents_t *events, SX126Handler *sxHandler);
    void ReInit(RadioEvents_t *events, SX126Handler *sxHandler);
    RadioState_t GetStatus(SX126Handler *sxHandler);
    void SetModem(RadioModems_t modem, SX126Handler *sxHandler);
    void SetChannel(uint32_t freq, SX126Handler *sxHandler);
    bool IsChannelFree(uint32_t freq, int16_t rssiThresh, uint32_t maxCarrierSenseTime, SX126Handler *sxHandler);
    uint32_t Random(SX126Handler *sxHandler);
    void SetRxConfig(uint32_t bandwidth, uint32_t datarate, uint8_t coderate, uint32_t bandwidthAfc, uint16_t preambleLen, uint16_t symbTimeout, bool fixLen, uint8_t payloadLen, bool crcOn, bool freqHopOn, uint8_t hopPeriod, bool iqInverted, bool rxContinuous, SX126Handler *sxHandler);
    void SetTxConfig(int8_t power, uint32_t fdev, uint32_t bandwidth, uint32_t datarate, uint8_t coderate, uint16_t preambleLen, bool fixLen, bool crcOn, bool freqHopOn, uint8_t hopPeriod, bool iqInverted, uint32_t timeout, SX126Handler *sxHandler);
    bool CheckRfFrequency(uint32_t frequency);
    uint32_t TimeOnAir(uint8_t pktLen);
    void Send(uint8_t *buffer, uint8_t size, SX126Handler *sxHandler);
    void Sleep(SX126Handler *sxHandler);
    void Standby(SX126Handler *sxHandler);
    void Rx(uint32_t timeout, SX126Handler *sxHandler);
    void RxBoosted(uint32_t timeout, SX126Handler *sxHandler);
    void SetRxDutyCycle(uint32_t rxTime, uint32_t sleepTime, SX126Handler *sxHandler);
    void SetCadParams(uint8_t cadSymbolNum, uint8_t cadDetPeak, uint8_t cadDetMin, uint8_t cadExitMode, uint32_t cadTimeout, SX126Handler *sxHandler);
    void StartCad(SX126Handler *sxHandler);
    void Tx(uint32_t timeout, SX126Handler *sxHandler);
    void SetTxContinuousWave(uint32_t freq, int8_t power, uint16_t time, SX126Handler *sxHandler);
    int16_t Rssi(RadioModems_t modem, SX126Handler *sxHandler);
    void Write(uint16_t addr, uint8_t data, SX126Handler *sxHandler);
    uint8_t Read(uint16_t addr, SX126Handler *sxHandler);
    void WriteBuffer(uint16_t addr, uint8_t *buffer, uint8_t size, SX126Handler *sxHandler);
    void ReadBuffer(uint16_t addr, uint8_t *buffer, uint8_t size, SX126Handler *sxHandler);
    void WriteFifo(uint8_t *buffer, uint8_t size, SX126Handler *sxHandler);
    void ReadFifo(uint8_t *buffer, uint8_t size, SX126Handler *sxHandler);
    void SetMaxPayloadLength(uint8_t max, SX126Handler *sxHandler);
    void SetPublicNetwork(bool enable, SX126Handler *sxHandler);
    uint32_t GetWakeupTime();
    void OnTxTimeoutIrq();
    void OnRxTimeoutIrq();
    static void OnDioIrq();
    void BgIrqProcess(SX126Handler *sxHandler);
    void IrqProcess(SX126Handler *sxHandler);
    void IrqProcessAfterDeepSleep(SX126Handler *sxHandler);

protected:

    TimerEvent_t TxTimeoutTimer;
    TimerEvent_t RxTimeoutTimer;
    const RadioLoRaBandwidths_t Bandwidths[10] = {LORA_BW_125, LORA_BW_250, LORA_BW_500, LORA_BW_062, LORA_BW_041, LORA_BW_031, LORA_BW_020, LORA_BW_015, LORA_BW_010, LORA_BW_007};

    //                                  SF12    SF11    SF10    SF9    SF8    SF7
    double RadioLoRaSymbTime[3][6] = {{32.768, 16.384, 8.192, 4.096, 2.048, 1.024}, //125KHz
                                     {16.384, 8.192, 4.096, 2.048, 1.024, 0.512},   //250KHz
                                     {8.192, 4.096, 2.048, 1.024, 0.512, 0.256}};   //500KHz
    uint8_t MaxPayloadLength = 0xFF;
    uint32_t TxTimeout = 0;
    uint32_t RxTimeout = 0;
    bool RxContinuous = false;
    PacketStatus_t RadioPktStatus;
    uint8_t RadioRxPayload[255];

    static bool IrqFired;
    bool TimerRxTimeout = false;
    bool TimerTxTimeout = false;

    RadioModems_t _modem;
    RadioPublicNetwork_t RadioPublicNetwork = {false};
    /*!
     * Radio callbacks variable
     */
    RadioEvents_t *RadioEvents;
    SX126x_t SX126x;

};

/*!
 * \brief Radio driver
 *
 * \remark This variable is defined and initialized in the specific radio
 *         board implementation
 */
//extern const struct Radio_s Radio;

#endif // __RADIO2_H__
