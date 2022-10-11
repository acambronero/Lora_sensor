#ifndef __RADIOHANDLER_H__
#define __RADIOHANDLER_H__

#include "stdint.h"
#include "SX126x/boards/mcu/timer.h"
#include "SX126xHardware.h"
#include <vector>
#include "datahelper.h"

#define BUFFER_SIZE 64


//class SX126xDriver;
//class SX126Handler;




/*!
 * \brief Radio driver definition
 */
class RadioHandler
{

public:
    RadioHandler();
    ~RadioHandler();
    void Init(SX126Handler *sxHandler);
    RadioState_t GetStatus(SX126Handler *sxHandler);
    void SetModem(RadioModems_t modem, SX126Handler *sxHandler);
    void SetChannel(uint32_t freq, SX126Handler *sxHandler);
    bool IsChannelFree(uint32_t freq, int16_t rssiThresh, uint32_t maxCarrierSenseTime, SX126Handler *sxHandler);
    uint32_t Random(SX126Handler *sxHandler);
    void SetRxConfig(uint32_t bandwidth, uint32_t datarate, uint8_t coderate, uint16_t preambleLen, uint16_t symbTimeout, bool fixLen, uint8_t payloadLen, bool crcOn, bool iqInverted, bool rxContinuous, uint32_t RxTimeout, SX126Handler *sxHandler);
    void SetTxConfig(int8_t power, uint32_t bandwidth, uint32_t datarate, uint8_t coderate, uint16_t preambleLen, bool fixLen, bool crcOn, bool iqInverted, uint32_t timeout, SX126Handler *sxHandler);
    uint32_t TimeOnAir(uint8_t pktLen);
    void Send(uint8_t *buffer, uint8_t size, uint32_t TxTimeout, uint32_t RxTimeout, SX126Handler *sxHandler);
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
    void SetMaxPayloadLength(uint8_t max, SX126Handler *sxHandler);
    void SetPublicNetwork(bool enable, SX126Handler *sxHandler);
    uint32_t GetWakeupTime();
#if ARDUINO
    static void OnTxTimeoutIrq();
    static void OnRxTimeoutIrq();
#else
    void OnTxTimeoutIrq();
    void OnRxTimeoutIrq();
#endif
    static void OnDioIrq();
    void BgIrqProcess(uint8_t *dataReady, SX126Handler *sxHandler);
    void IrqProcess(uint8_t *dataReady, SX126Handler *sxHandler);
    void IrqProcessAfterDeepSleep(uint8_t *dataReady, SX126Handler *sxHandler);

    /*!
     * \brief  Tx Done callback prototype.
     */
    void OnTxDone(void);
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
    void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);
    /*!
     * \brief  Tx Timeout callback prototype.
     */
    void OnTxTimeout(void);
    /*!
     * \brief  Rx Timeout callback prototype.
     */
    void OnRxTimeout(void);
    /*!
     * \brief Rx Error callback prototype.
     */
    void OnRxError(void);
    /*!
     * \brief Preamble detected callback prototype.
     */
    void OnPreAmpDetect(void);
    /*!
     * \brief  FHSS Change Channel callback prototype.
     *
     * \param  currentChannel   Index number of the current channel
     */
    void OnFhssChangeChannel(uint8_t currentChannel);

    /*!
     * \brief CAD Done callback prototype.
     *
     * \param  channelDetected    Channel Activity detected during the CAD
     */
    void OnCadDone(bool channelActivityDetected);
    std::array<uint8_t, 255> GetPayloadData(uint16_t size);
    void decode_message();
    DataHelperRequest prepare_acquisition_data();

    SX126Handler *sxHandler;
    uint8_t RadioRxPayload[255] = {0};
#ifdef ARDUINO
    static bool TimerTxTimeout;
    static bool TimerRxTimeout;
#else
    bool TimerTxTimeout = false;
    bool TimerRxTimeout = false;
#endif

protected:

    TimerEvent_t TxTimeoutTimer;
    TimerEvent_t RxTimeoutTimer;
    const RadioLoRaBandwidths_t Bandwidths[10] = {LORA_BW_125, LORA_BW_250, LORA_BW_500, LORA_BW_062, LORA_BW_041, LORA_BW_031, LORA_BW_020, LORA_BW_015, LORA_BW_010, LORA_BW_007};

    //                                  SF12    SF11    SF10    SF9    SF8    SF7
    double RadioLoRaSymbTime[3][6] = {{32.768, 16.384, 8.192, 4.096, 2.048, 1.024}, //125KHz
                                     {16.384, 8.192, 4.096, 2.048, 1.024, 0.512},   //250KHz
                                     {8.192, 4.096, 2.048, 1.024, 0.512, 0.256}};   //500KHz
    uint8_t MaxPayloadLength = 0xFF;
    //uint8_t MaxPayloadLength = 0x7F;

    bool RxContinuous = false;
    PacketStatus_t RadioPktStatus;

    uint32_t TxTimeout = 0;
    uint32_t RxTimeout = 0;
    static bool IrqFired;


    RadioModems_t _modem;
    RadioPublicNetwork_t RadioPublicNetwork = {false};
    std::vector<IrqsActivated> irqsEnable;
    uint8_t RcvBuffer[BUFFER_SIZE];
    uint8_t TxdBuffer[BUFFER_SIZE];
    SX126x_t SX126x;
    SX126xDriver *sxDriver = nullptr;
    uint8_t triesToSend = 0;

    uint8_t message_type = 0;
    uint8_t command = 0;
    uint32_t deviceTag = 0;
    uint8_t version = 0;
    uint64_t timestamp = 0;
    uint8_t deviceId = 0;

};

#endif // __RADIOHANDLER_H__
