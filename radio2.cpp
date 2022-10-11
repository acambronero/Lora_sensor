/*!
 * @file      radio.cpp
 *
 * @brief     Radio driver API definition
 *
 * @copyright Revised BSD License, see file LICENSE.
 *
 * @code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 * @endcode
 *´
 * @author    Miguel Luis ( Semtech )
 *
 * @author    Gregory Cristian ( Semtech )
 */
#include <math.h>
#include <string.h>
#include <unistd.h>
#include "SX126xHardware.h"
#include "mcu/timer.h"
#include "radio2.h"
//#include "itools.h"
#include "datahelper.h"
#include "wiringPi.h"
#include <iostream>
#include <functional>

bool RadioHandler::IrqFired = false;

RadioHandler::RadioHandler()
{
}

/*!
 * @brief Initializes the radio
 *
 * @param  events Structure containing the driver callback functions
 */
void RadioHandler::Init(RadioEvents_t *events, SX126Handler *sxHandler) {
	RadioEvents = events;
    //SX126xInit(std::bind(&RadioHandler::RadioOnDioIrq, this), sxHandler);
    SX126xInit(OnDioIrq, sxHandler);

    //wiringPiISR(25, INT_EDGE_RISING, OnDioIrq);

    SX126xSetStandby(STDBY_RC, sxHandler);
    SX126xSetBufferBaseAddress(0x00, 0x00, sxHandler);
    SX126xSetTxParams(0, RADIO_RAMP_200_US, sxHandler);
    SX126xSetDioIrqParams(IRQ_RADIO_ALL, IRQ_RADIO_ALL, IRQ_RADIO_NONE, IRQ_RADIO_NONE, sxHandler);

	// Initialize driver timeout timers
	TxTimeoutTimer.oneShot = true;
	RxTimeoutTimer.oneShot = true;

    TimerInit(&TxTimeoutTimer, std::bind(&RadioHandler::OnTxTimeoutIrq, this));
    TimerInit(&RxTimeoutTimer, std::bind(&RadioHandler::OnRxTimeoutIrq, this));

	IrqFired = false;
}

/*!
 * @brief Initializes the radio after deep sleep of the CPU
 *
 * @param  events Structure containing the driver callback functions
 */
void RadioHandler::ReInit(RadioEvents_t *events, SX126Handler *sxHandler)
{
	RadioEvents = events;
    //SX126xReInit(std::bind(&RadioHandler::RadioOnDioIrq, this), sxHandler);
    SX126xReInit(OnDioIrq, sxHandler);
	// Initialize driver timeout timers
	TxTimeoutTimer.oneShot = true;
	RxTimeoutTimer.oneShot = true;
    TimerInit(&TxTimeoutTimer, std::bind(&RadioHandler::OnTxTimeoutIrq, this));
    TimerInit(&RxTimeoutTimer, std::bind(&RadioHandler::OnRxTimeoutIrq, this));

	IrqFired = false;
}

/*!
 * Return current radio status
 *
 * @retval Radio status.[RF_IDLE, RF_RX_RUNNING, RF_TX_RUNNING]
 */
RadioState_t RadioHandler::GetStatus(SX126Handler *sxHandler)
{
	switch (SX126xGetOperatingMode())
	{
	case MODE_TX:
		return RF_TX_RUNNING;
	case MODE_RX:
		return RF_RX_RUNNING;
	case MODE_CAD:
		return RF_CAD;
	default:
		return RF_IDLE;
	}
}

/*!
 * @brief Configures the radio with the given modem
 *
 * @param  modem Modem to be used [0: FSK, 1: LoRa]
 */
void RadioHandler::SetModem(RadioModems_t modem, SX126Handler *sxHandler)
{
    SX126xSetPacketType(PACKET_TYPE_LORA, sxHandler);
    // Public/Private network register is reset when switching modems
    if (RadioPublicNetwork.Current != RadioPublicNetwork.Previous)
    {
        RadioPublicNetwork.Current = RadioPublicNetwork.Previous;
        SetPublicNetwork(RadioPublicNetwork.Current, sxHandler);
    }
    _modem = modem;

}

/*!
 * @brief Sets the channel frequency
 *
 * @param  freq         Channel RF frequency
 */
void RadioHandler::SetChannel(uint32_t freq, SX126Handler *sxHandler)
{
    SX126xSetRfFrequency(freq, sxHandler);
}

/*!
 * @brief Checks if the channel is free for the given time
 *
 * @param  modem      Radio modem to be used [0: FSK, 1: LoRa]
 * @param  freq       Channel RF frequency
 * @param  rssiThresh RSSI threshold
 * @param  maxCarrierSenseTime Max time while the RSSI is measured
 *
 * @retval isFree         [true: Channel is free, false: Channel is not free]
 */
bool RadioHandler::IsChannelFree(uint32_t freq, int16_t rssiThresh, uint32_t maxCarrierSenseTime, SX126Handler *sxHandler)
{
	bool status = true;
	int16_t rssi = 0;
	uint32_t carrierSenseTime = 0;

    if (GetStatus(sxHandler) != RF_IDLE)
	{
		return false;
	}

    SetModem(MODEM_LORA, sxHandler);

    SetChannel(freq, sxHandler);

    Rx(0, sxHandler);

    sleep_microseconds(1);


	carrierSenseTime = TimerGetCurrentTime();

	// Perform carrier sense for maxCarrierSenseTime
	while (TimerGetElapsedTime(carrierSenseTime) < maxCarrierSenseTime)
	{
        rssi = Rssi(MODEM_LORA, sxHandler);

		if (rssi > rssiThresh)
		{
			status = false;
			break;
		}
	}
    Sleep(sxHandler);
	return status;
}

/*!
 * @brief Generates a 32 bits random value based on the RSSI readings
 *
 * @remark This function sets the radio in LoRa modem mode and disables
 *         all interrupts.
 *         After calling this function either Radio.SetRxConfig or
 *         Radio.SetTxConfig functions must be called.
 *
 * @retval randomValue    32 bits random value
 */
uint32_t RadioHandler::Random(SX126Handler *sxHandler)
{
	uint32_t rnd = 0;

	/*
     * Radio setup for random number generation
     */
	// Set LoRa modem ON
    SetModem(MODEM_LORA, sxHandler);

	// Set radio in continuous reception
    SX126xSetRx(0, sxHandler);

    rnd = SX126xGetRandom(sxHandler);
    Sleep(sxHandler);

	return rnd;
}

/*!
 * @brief Sets the reception parameters
 *
 * @param  modem        Radio modem to be used [0: FSK, 1: LoRa]
 * @param  bandwidth    Sets the bandwidth
 *                          FSK : >= 2600 and <= 250000 Hz
 *                          LoRa: [0: 125 kHz, 1: 250 kHz,
 *                                 2: 500 kHz, 3: Reserved]
 * @param  datarate     Sets the Datarate
 *                          FSK : 600..300000 bits/s
 *                          LoRa: [6: 64, 7: 128, 8: 256, 9: 512,
 *                                10: 1024, 11: 2048, 12: 4096  chips]
 * @param  coderate     Sets the coding rate (LoRa only)
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
 * @param  bandwidthAfc Sets the AFC Bandwidth (FSK only)
 *                          FSK : >= 2600 and <= 250000 Hz
 *                          LoRa: N/A ( set to 0 )
 * @param  preambleLen  Sets the Preamble length
 *                          FSK : Number of bytes
 *                          LoRa: Length in symbols (the hardware adds 4 more symbols)
 * @param  symbTimeout  Sets the RxSingle timeout value
 *                          FSK : timeout in number of bytes
 *                          LoRa: timeout in symbols
 * @param  fixLen       Fixed length packets [0: variable, 1: fixed]
 * @param  payloadLen   Sets payload length when fixed length is used
 * @param  crcOn        Enables/Disables the CRC [0: OFF, 1: ON]
 * @param  FreqHopOn    Enables disables the intra-packet frequency hopping
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: [0: OFF, 1: ON]
 * @param  HopPeriod    Number of symbols between each hop
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: Number of symbols
 * @param  iqInverted   Inverts IQ signals (LoRa only)
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: [0: not inverted, 1: inverted]
 * @param  rxContinuous Sets the reception in continuous mode
 *                          [false: single mode, true: continuous mode]
 */
void RadioHandler::SetRxConfig (uint32_t bandwidth,
					  uint32_t datarate, uint8_t coderate,
					  uint32_t bandwidthAfc, uint16_t preambleLen,
					  uint16_t symbTimeout, bool fixLen,
					  uint8_t payloadLen,
					  bool crcOn, bool freqHopOn, uint8_t hopPeriod,
                      bool iqInverted, bool rxContinuous, SX126Handler* sxHandler)
{

	RxContinuous = rxContinuous;
	if (rxContinuous == true)
	{
		symbTimeout = 0;
	}
	if (fixLen == true)
	{
		MaxPayloadLength = payloadLen;
	}
	else
	{
		MaxPayloadLength = 0xFF;
	}


    SX126xSetStopRxTimerOnPreambleDetect(false, sxHandler);
    SX126xSetLoRaSymbNumTimeout(symbTimeout, sxHandler);
    SX126x.ModulationParams.PacketType = PACKET_TYPE_LORA;
    SX126x.ModulationParams.Params.LoRa.SpreadingFactor = (RadioLoRaSpreadingFactors_t)datarate;
    SX126x.ModulationParams.Params.LoRa.Bandwidth = Bandwidths[bandwidth];
    SX126x.ModulationParams.Params.LoRa.CodingRate = (RadioLoRaCodingRates_t)coderate;

    if (((bandwidth == 0) && ((datarate == 11) || (datarate == 12))) ||
            ((bandwidth == 1) && (datarate == 12)))
    {
        SX126x.ModulationParams.Params.LoRa.LowDatarateOptimize = 0x01;
    }
    else
    {
        SX126x.ModulationParams.Params.LoRa.LowDatarateOptimize = 0x00;
    }

    SX126x.PacketParams.PacketType = PACKET_TYPE_LORA;

    if ((SX126x.ModulationParams.Params.LoRa.SpreadingFactor == LORA_SF5) ||
            (SX126x.ModulationParams.Params.LoRa.SpreadingFactor == LORA_SF6))
    {
        if (preambleLen < 12)
        {
            SX126x.PacketParams.Params.LoRa.PreambleLength = 12;
        }
        else
        {
            SX126x.PacketParams.Params.LoRa.PreambleLength = preambleLen;
        }
    }
    else
    {
        SX126x.PacketParams.Params.LoRa.PreambleLength = preambleLen;
    }

    SX126x.PacketParams.Params.LoRa.HeaderType = (RadioLoRaPacketLengthsMode_t)fixLen;

    SX126x.PacketParams.Params.LoRa.PayloadLength = MaxPayloadLength;
    SX126x.PacketParams.Params.LoRa.CrcMode = (RadioLoRaCrcModes_t)crcOn;
    SX126x.PacketParams.Params.LoRa.InvertIQ = (RadioLoRaIQModes_t)iqInverted;

    SetModem(MODEM_LORA, sxHandler);
    SX126xSetModulationParams(&SX126x.ModulationParams, sxHandler);
    SX126xSetPacketParams(&SX126x.PacketParams, sxHandler);

    // WORKAROUND - Optimizing the Inverted IQ Operation, see DS_SX1261-2_V1.2 datasheet chapter 15.4
    if (SX126x.PacketParams.Params.LoRa.InvertIQ == LORA_IQ_INVERTED)
    {
        // RegIqPolaritySetup = @address 0x0736
        sxHandler->SX126xWriteRegister(0x0736, sxHandler->SX126xReadRegister(0x0736) & ~(1 << 2));
    }
    else
    {
        // RegIqPolaritySetup @address 0x0736
        sxHandler->SX126xWriteRegister(0x0736, sxHandler->SX126xReadRegister(0x0736) | (1 << 2));
    }
    // WORKAROUND END

    // Timeout Max, Timeout handled directly in SetRx function
    RxTimeout = 0xFA0;
}

/*!
 * \brief Sets the transmission parameters
 *
 * @param  modem        Radio modem to be used [0: FSK, 1: LoRa]
 * @param  power        Sets the output power [dBm]
 * @param  fdev         Sets the frequency deviation (FSK only)
 *                          FSK : [Hz]
 *                          LoRa: 0
 * @param  bandwidth    Sets the bandwidth (LoRa only)
 *                          FSK : 0
 *                          LoRa: [0: 125 kHz, 1: 250 kHz,
 *                                 2: 500 kHz, 3: Reserved]
 * @param  datarate     Sets the Datarate
 *                          FSK : 600..300000 bits/s
 *                          LoRa: [6: 64, 7: 128, 8: 256, 9: 512,
 *                                10: 1024, 11: 2048, 12: 4096  chips]
 * @param  coderate     Sets the coding rate (LoRa only)
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
 * @param  preambleLen  Sets the preamble length
 *                          FSK : Number of bytes
 *                          LoRa: Length in symbols (the hardware adds 4 more symbols)
 * @param  fixLen       Fixed length packets [0: variable, 1: fixed]
 * @param  crcOn        Enables disables the CRC [0: OFF, 1: ON]
 * @param  FreqHopOn    Enables disables the intra-packet frequency hopping
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: [0: OFF, 1: ON]
 * @param  HopPeriod    Number of symbols between each hop
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: Number of symbols
 * @param  iqInverted   Inverts IQ signals (LoRa only)
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: [0: not inverted, 1: inverted]
 * @param  timeout      Transmission timeout [ms]
 */
void RadioHandler::SetTxConfig(int8_t power, uint32_t fdev,
					  uint32_t bandwidth, uint32_t datarate,
					  uint8_t coderate, uint16_t preambleLen,
					  bool fixLen, bool crcOn, bool freqHopOn,
                      uint8_t hopPeriod, bool iqInverted, uint32_t timeout, SX126Handler* sxHandler)
{

    SX126x.ModulationParams.PacketType = PACKET_TYPE_LORA;
    SX126x.ModulationParams.Params.LoRa.SpreadingFactor = (RadioLoRaSpreadingFactors_t)datarate;
    SX126x.ModulationParams.Params.LoRa.Bandwidth = Bandwidths[bandwidth];
    SX126x.ModulationParams.Params.LoRa.CodingRate = (RadioLoRaCodingRates_t)coderate;

    if (((bandwidth == 0) && ((datarate == 11) || (datarate == 12))) ||
            ((bandwidth == 1) && (datarate == 12)))
    {
        SX126x.ModulationParams.Params.LoRa.LowDatarateOptimize = 0x01;
    }
    else
    {
        SX126x.ModulationParams.Params.LoRa.LowDatarateOptimize = 0x00;
    }

    SX126x.PacketParams.PacketType = PACKET_TYPE_LORA;

    if ((SX126x.ModulationParams.Params.LoRa.SpreadingFactor == LORA_SF5) ||
            (SX126x.ModulationParams.Params.LoRa.SpreadingFactor == LORA_SF6))
    {
        if (preambleLen < 12)
        {
            SX126x.PacketParams.Params.LoRa.PreambleLength = 12;
        }
        else
        {
            SX126x.PacketParams.Params.LoRa.PreambleLength = preambleLen;
        }
    }
    else
    {
        SX126x.PacketParams.Params.LoRa.PreambleLength = preambleLen;
    }

    SX126x.PacketParams.Params.LoRa.HeaderType = (RadioLoRaPacketLengthsMode_t)fixLen;
    SX126x.PacketParams.Params.LoRa.PayloadLength = MaxPayloadLength;
    SX126x.PacketParams.Params.LoRa.CrcMode = (RadioLoRaCrcModes_t)crcOn;
    SX126x.PacketParams.Params.LoRa.InvertIQ = (RadioLoRaIQModes_t)iqInverted;

    Standby(sxHandler);
    SetModem((SX126x.ModulationParams.PacketType == PACKET_TYPE_GFSK) ? MODEM_FSK : MODEM_LORA, sxHandler);
    SX126xSetModulationParams(&SX126x.ModulationParams, sxHandler);
    SX126xSetPacketParams(&SX126x.PacketParams, sxHandler);

	// WORKAROUND - Modulation Quality with 500 kHz LoRa Bandwidth, see DS_SX1261-2_V1.2 datasheet chapter 15.1
    if (SX126x.ModulationParams.Params.LoRa.Bandwidth == LORA_BW_500)
	{
		// RegTxModulation = @address 0x0889
        sxHandler->SX126xWriteRegister(0x0889, sxHandler->SX126xReadRegister(0x0889) & ~(1 << 2));
	}
	else
	{
		// RegTxModulation = @address 0x0889
        sxHandler->SX126xWriteRegister(0x0889, sxHandler->SX126xReadRegister(0x0889) | (1 << 2));
	}
	// WORKAROUND END

    sxHandler->SX126xSetRfTxPower(power);
	TxTimeout = timeout;
}

/*!
 * @brief Checks if the given RF frequency is supported by the hardware
 *
 * @param  frequency RF frequency to be checked
 * @retval isSupported [true: supported, false: unsupported]
 */
bool RadioHandler::CheckRfFrequency(uint32_t frequency)
{
	return true;
}

/*!
 * @brief Computes the packet time on air in ms for the given payload
 *
 * @remark Can only be called once SetRxConfig or SetTxConfig have been called
 *
 * @param  modem      Radio modem to be used [0: FSK, 1: LoRa]
 * @param  pktLen     Packet payload length
 *
 * @retval airTime        Computed airTime (ms) for the given packet payload length
 */
uint32_t RadioHandler::TimeOnAir(uint8_t pktLen)
{
	uint32_t airTime = 0;

    double ts = RadioLoRaSymbTime[SX126x.ModulationParams.Params.LoRa.Bandwidth - 4][12 - SX126x.ModulationParams.Params.LoRa.SpreadingFactor];
    // time of preamble
    double tPreamble = (SX126x.PacketParams.Params.LoRa.PreambleLength + 4.25) * ts;
    // Symbol length of payload and time
    double tmp = ceil((8 * pktLen - 4 * SX126x.ModulationParams.Params.LoRa.SpreadingFactor +
                       28 + 16 * SX126x.PacketParams.Params.LoRa.CrcMode -
                       ((SX126x.PacketParams.Params.LoRa.HeaderType == LORA_PACKET_FIXED_LENGTH) ? 20 : 0)) /
                      (double)(4 * (SX126x.ModulationParams.Params.LoRa.SpreadingFactor -
                                    ((SX126x.ModulationParams.Params.LoRa.LowDatarateOptimize > 0) ? 2 : 0)))) *
            ((SX126x.ModulationParams.Params.LoRa.CodingRate % 4) + 4);
    double nPayload = 8 + ((tmp > 0) ? tmp : 0);
    double tPayload = nPayload * ts;
    // Time on air
    double tOnAir = tPreamble + tPayload;
    // return milli seconds
    airTime = floor(tOnAir + 0.999);
    return airTime;
}

/*!
 * @brief Sends the buffer of size. Prepares the packet to be sent and sets
 *        the radio in transmission
 *
 * @param : buffer     Buffer pointer
 * @param : size       Buffer size
 */
void RadioHandler::Send(uint8_t *buffer, uint8_t size, SX126Handler *sxHandler)
{
	SX126xSetDioIrqParams(IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT,
						  IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT,
						  IRQ_RADIO_NONE,
                          IRQ_RADIO_NONE, sxHandler);

	if (SX126xGetPacketType() == PACKET_TYPE_LORA)
	{
		SX126x.PacketParams.Params.LoRa.PayloadLength = size;
	}
	else
	{
		SX126x.PacketParams.Params.Gfsk.PayloadLength = size;
	}
    SX126xSetPacketParams(&SX126x.PacketParams, sxHandler);

    SX126xSendPayload(buffer, size, 0, sxHandler);
	TimerSetValue(&TxTimeoutTimer, TxTimeout);
	TimerStart(&TxTimeoutTimer);
}

/*!
 * @brief Sets the radio in sleep mode
 */
void RadioHandler::Sleep(SX126Handler *sxHandler)
{
	SleepParams_t params = {0};

	params.Fields.WarmStart = 1;
    SX126xSetSleep(params, sxHandler);

    sleep_microseconds(2);
}

/*!
 * @brief Sets the radio in standby mode
 */
void RadioHandler::Standby(SX126Handler *sxHandler)
{
    SX126xSetStandby(STDBY_RC, sxHandler);
}

/*!
 * @brief Sets the radio in reception mode for the given time
 * @param  timeout Reception timeout [ms]
 *                     [0: continuous, others timeout]
 */
void RadioHandler::Rx(uint32_t timeout, SX126Handler *sxHandler)
{

    SX126xSetDioIrqParams(IRQ_RADIO_ALL, //IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT,
						  IRQ_RADIO_ALL, //IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT,
						  IRQ_RADIO_NONE,
                          IRQ_RADIO_NONE, sxHandler);
    /*SX126xSetDioIrqParams(IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT,
                          IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT,
                          IRQ_RADIO_NONE,
                          IRQ_RADIO_NONE);*/

	LOG_LIB("RADIO", "RX window timeout = %ld", timeout);
	if (RxContinuous == true)
	{
		// Even Continous mode is selected, put a timeout here
		if (timeout != 0)
		{
			TimerSetValue(&RxTimeoutTimer, timeout);
			TimerStart(&RxTimeoutTimer);
		}
        SX126xSetRx(0xFFFFFF, sxHandler); // Rx Continuous
	}
	else
	{
        SX126xSetRx(RxTimeout << 6, sxHandler);
	}
}

/*!
 * @brief Sets the radio in reception mode with Max LNA gain for the given time
 * @param  timeout Reception timeout [ms]
 *                     [0: continuous, others timeout]
 */
void RadioHandler::RxBoosted(uint32_t timeout, SX126Handler *sxHandler)
{
	SX126xSetDioIrqParams(IRQ_RADIO_ALL, //IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT,
						  IRQ_RADIO_ALL, //IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT,
						  IRQ_RADIO_NONE,
                          IRQ_RADIO_NONE, sxHandler);

	if (RxContinuous == true)
	{
		// Even Continous mode is selected, put a timeout here
		if (timeout != 0)
		{
			TimerSetValue(&RxTimeoutTimer, timeout);
			TimerStart(&RxTimeoutTimer);
		}
        SX126xSetRxBoosted(0xFFFFFF, sxHandler); // Rx Continuous
	}
	else
	{
        SX126xSetRxBoosted(RxTimeout << 6, sxHandler);
	}
}

/*!
 * @brief Sets the Rx duty cycle management parameters
 *
 * @param   rxTime        Structure describing reception timeout value
 * @param   sleepTime     Structure describing sleep timeout value
 */
void RadioHandler::SetRxDutyCycle(uint32_t rxTime, uint32_t sleepTime, SX126Handler *sxHandler)
{
	SX126xSetDioIrqParams(IRQ_RADIO_ALL | IRQ_RX_TX_TIMEOUT,
						  IRQ_RADIO_ALL | IRQ_RX_TX_TIMEOUT,
                          IRQ_RADIO_NONE, IRQ_RADIO_NONE, sxHandler);
    SX126xSetRxDutyCycle(rxTime, sleepTime, sxHandler);
}

/*!
 * @brief Set Channel Activity Detection parameters
 *
 * \param   cadSymbolNum   The number of symbol to use for CAD operations
 *                             [LORA_CAD_01_SYMBOL, LORA_CAD_02_SYMBOL,
 *                              LORA_CAD_04_SYMBOL, LORA_CAD_08_SYMBOL,
 *                              LORA_CAD_16_SYMBOL]
 * \param   cadDetPeak     Limit for detection of SNR peak used in the CAD
 * \param   cadDetMin      Set the minimum symbol recognition for CAD
 * \param   cadExitMode    Operation to be done at the end of CAD action
 *                             [LORA_CAD_ONLY, LORA_CAD_RX, LORA_CAD_LBT]
 * \param   cadTimeout     Defines the timeout value to abort the CAD activity
 */
void RadioHandler::SetCadParams(uint8_t cadSymbolNum, uint8_t cadDetPeak, uint8_t cadDetMin, uint8_t cadExitMode, uint32_t cadTimeout, SX126Handler *sxHandler)
{
    SX126xSetCadParams((RadioLoRaCadSymbols_t)cadSymbolNum, cadDetPeak, cadDetMin, (RadioCadExitModes_t)cadExitMode, cadTimeout, sxHandler);
}

/*!
 * @brief Start a Channel Activity Detection
 *
 * @remark Before calling this function CAD parameters need to be set first! \
 * 		Use RadioSetCadParams to setup the parameters or directly in the SX126x low level functions
 *      RadioSetCadParams(uint8_t cadSymbolNum, uint8_t cadDetPeak, uint8_t cadDetMin, uint8_t cadExitMode, uint32_t cadTimeout);
 *              IRQ_CAD_DONE | IRQ_CAD_ACTIVITY_DETECTED, \
 *              IRQ_RADIO_NONE, IRQ_RADIO_NONE); \
 */
void RadioHandler::StartCad(SX126Handler *sxHandler)
{
	SX126xSetDioIrqParams(IRQ_CAD_DONE | IRQ_CAD_ACTIVITY_DETECTED,
						  IRQ_CAD_DONE | IRQ_CAD_ACTIVITY_DETECTED,
                          IRQ_RADIO_NONE, IRQ_RADIO_NONE, sxHandler);
    SX126xSetCad(sxHandler);
}

void RadioHandler::Tx(uint32_t timeout, SX126Handler *sxHandler)
{
    SX126xSetTx(timeout << 6, sxHandler);
}

/*!
 * \brief Sets the radio in continuous wave transmission mode
 *
 * \param freq       Channel RF frequency
 * \param power      Sets the output power [dBm]
 * \param time       Transmission mode timeout [s]
 */
void RadioHandler::SetTxContinuousWave(uint32_t freq, int8_t power, uint16_t time, SX126Handler *sxHandler)
{
    SX126xSetRfFrequency(freq, sxHandler);
    sxHandler->SX126xSetRfTxPower(power);
    SX126xSetTxContinuousWave(sxHandler);

	TimerSetValue(&TxTimeoutTimer, time * 1e3);
	TimerStart(&TxTimeoutTimer);
}

/*!
 * @brief Reads the current RSSI value
 *
 * @retval rssiValue Current RSSI value in [dBm]
 */
int16_t RadioHandler::Rssi(RadioModems_t modem, SX126Handler *sxHandler)
{
    return SX126xGetRssiInst(sxHandler);
}

/*!
 * \brief Writes the radio register at the specified address
 *
 * \param  addr Register address
 * \param  data New register value
 */
void RadioHandler::Write(uint16_t addr, uint8_t data, SX126Handler *sxHandler)
{
    sxHandler->SX126xWriteRegister(addr, data);
}

/*!
 * @brief Reads the radio register at the specified address
 *
 * @param : addr Register address
 * @retval data Register value
 */
uint8_t RadioHandler::Read(uint16_t addr, SX126Handler *sxHandler)
{
    return sxHandler->SX126xReadRegister(addr);
}

/*!
 * @brief Writes multiple radio registers starting at address
 *
 * @param  addr   First Radio register address
 * @param  buffer Buffer containing the new register's values
 * @param  size   Number of registers to be written
 */
void RadioHandler::WriteBuffer(uint16_t addr, uint8_t *buffer, uint8_t size, SX126Handler *sxHandler)
{
    sxHandler->SX126xWriteRegisters(addr, buffer, size);
}

/*!
 * @brief Reads multiple radio registers starting at address
 *
 * @param  addr First Radio register address
 * @param  buffer Buffer where to copy the registers data
 * @param  size Number of registers to be read
 */
void RadioHandler::ReadBuffer(uint16_t addr, uint8_t *buffer, uint8_t size, SX126Handler *sxHandler)
{
    sxHandler->SX126xReadRegisters(addr, buffer, size);
}

void RadioHandler::WriteFifo(uint8_t *buffer, uint8_t size, SX126Handler *sxHandler)
{
    sxHandler->SX126xWriteBuffer(0, buffer, size);
}

void RadioHandler::ReadFifo(uint8_t *buffer, uint8_t size, SX126Handler *sxHandler)
{
    sxHandler->SX126xReadBuffer(0, buffer, size);
}

/*!
 * @brief Sets the maximum payload length.
 *
 * @param  modem      Radio modem to be used [0: FSK, 1: LoRa]
 * @param  max        Maximum payload length in bytes
 */
void RadioHandler::SetMaxPayloadLength(uint8_t max, SX126Handler *sxHandler)
{
		SX126x.PacketParams.Params.LoRa.PayloadLength = MaxPayloadLength = max;
        SX126xSetPacketParams(&SX126x.PacketParams, sxHandler);
}

/*!
 * @brief Sets the network to public or private. Updates the sync byte.
 *
 * @remark Applies to LoRa modem only
 *
 * @param  enable if true, it enables a public network
 */
void RadioHandler::SetPublicNetwork(bool enable, SX126Handler *sxHandler)
{
	RadioPublicNetwork.Current = RadioPublicNetwork.Previous = enable;

    SetModem(MODEM_LORA, sxHandler);
	if (enable == true)
	{
		// Change LoRa modem SyncWord
        sxHandler->SX126xWriteRegister(REG_LR_SYNCWORD, (LORA_MAC_PUBLIC_SYNCWORD >> 8) & 0xFF);
        sxHandler->SX126xWriteRegister(REG_LR_SYNCWORD + 1, LORA_MAC_PUBLIC_SYNCWORD & 0xFF);
	}
	else
	{
		// Change LoRa modem SyncWord
        sxHandler->SX126xWriteRegister(REG_LR_SYNCWORD, (LORA_MAC_PRIVATE_SYNCWORD >> 8) & 0xFF);
        sxHandler->SX126xWriteRegister(REG_LR_SYNCWORD + 1, LORA_MAC_PRIVATE_SYNCWORD & 0xFF);
	}
}

/*!
 * @brief Gets the time required for the board plus radio to get out of sleep.[ms]
 *
 * @retval time Radio plus board wakeup time in ms.
 */
uint32_t RadioHandler::GetWakeupTime(void)
{
    return (RADIO_WAKEUP_TIME);
}

/*!
 * @brief Tx timeout timer callback
 */
void RadioHandler::OnTxTimeoutIrq(void)
{
    // if ((RadioEvents != NULL) && (RadioEvents->TxTimeout != NULL))
    // {
    //    RadioEvents->TxTimeout();
    //}
	TimerTxTimeout = true;
    //TimerStop(&TxTimeoutTimer);
}

/*!
 * @brief Rx timeout timer callback
 */
void RadioHandler::OnRxTimeoutIrq(void)
{
    // if ((RadioEvents != NULL) && (RadioEvents->RxTimeout != NULL))
    // {
    //    RadioEvents->RxTimeout();
    // }
	TimerRxTimeout = true;
    //TimerStop(&RxTimeoutTimer);
}

/*!
 * @brief DIO 0 IRQ callback
 */
void RadioHandler::OnDioIrq()
{
    std::cout << "RadioOnDioIrq" << std::endl;
    IrqFired = true;
}

/*!
 * @brief Process radio irq in background task (nRF52 & ESP32)
 */
void RadioHandler::BgIrqProcess(SX126Handler *sxHandler)
{
	bool rx_timeout_handled = false;
	bool tx_timeout_handled = false;
    RadioStatus_t st;

    uint16_t irqRegs = SX126xGetIrqStatus(sxHandler);
    //std::cout << "Interrupts: " << irqRegs << std::endl;
    st = SX126xGetStatus(sxHandler);
    std::cout << "Estado radio: " << std::hex << (int)st.Value << std::endl;
    if (irqRegs != 0){
        IrqFired = true;
    }
    SX126xClearIrqStatus(IRQ_RADIO_ALL, sxHandler);
    if (IrqFired)
	{
		IrqFired = false;

        //uint16_t irqRegs = SX126xGetIrqStatus(sxHandler);
        //SX126xClearIrqStatus(IRQ_RADIO_ALL, sxHandler);

		if ((irqRegs & IRQ_TX_DONE) == IRQ_TX_DONE)
		{
            std::cout << "IRQ_TX_DONE" << std::endl;

			tx_timeout_handled = true;
			TimerStop(&TxTimeoutTimer);
			//!< Update operating mode state to a value lower than \ref MODE_STDBY_XOSC
			SX126xSetOperatingMode(MODE_STDBY_RC);
			if ((RadioEvents != NULL) && (RadioEvents->TxDone != NULL))
			{
				RadioEvents->TxDone();
			}
		}

		if ((irqRegs & IRQ_RX_DONE) == IRQ_RX_DONE)
		{
            std::cout << "IRQ_RX_DONE" << std::endl;

			uint8_t size;

			rx_timeout_handled = true;
            //TimerStop(&RxTimeoutTimer);
			if (RxContinuous == false)
			{
				//!< Update operating mode state to a value lower than \ref MODE_STDBY_XOSC
				SX126xSetOperatingMode(MODE_STDBY_RC);

				// WORKAROUND - Implicit Header Mode Timeout Behavior, see DS_SX1261-2_V1.2 datasheet chapter 15.3
				// RegRtcControl = @address 0x0902
                sxHandler->SX126xWriteRegister(0x0902, 0x00);
				// RegEventMask = @address 0x0944
                sxHandler->SX126xWriteRegister(0x0944, sxHandler->SX126xReadRegister(0x0944) | (1 << 1));
				// WORKAROUND END
			}
			memset(RadioRxPayload, 0, 255);

			if ((irqRegs & IRQ_CRC_ERROR) == IRQ_CRC_ERROR)
			{
                std::cout << "IRQ_CRC_ERROR" << std::endl;

				uint8_t size;
				// Discard buffer
				memset(RadioRxPayload, 0, 255);
                SX126xGetPayload(RadioRxPayload, &size, 255, sxHandler);
                SX126xGetPacketStatus(&RadioPktStatus, sxHandler);
				if ((RadioEvents != NULL) && (RadioEvents->RxError))
				{
					RadioEvents->RxError();
				}
			}
			else
			{
                SX126xGetPayload(RadioRxPayload, &size, 255, sxHandler);
                std::cout << RadioRxPayload[0] << std::endl;
                std::cout << RadioRxPayload[1] << std::endl;
                std::cout << RadioRxPayload[2] << std::endl;
                std::cout << RadioRxPayload[3] << std::endl;
                SX126xGetPacketStatus(&RadioPktStatus, sxHandler);
                /*std::cout << "PACKET STATUS: "
                          << (int)RadioPktStatus.Params.LoRa.FreqError << std::endl
                          << (int)RadioPktStatus.Params.LoRa.RssiPkt << std::endl
                          << (int)RadioPktStatus.Params.LoRa.SignalRssiPkt << std::endl
                          << (int)RadioPktStatus.Params.LoRa. SnrPkt << std::endl;*/


				if ((RadioEvents != NULL) && (RadioEvents->RxDone != NULL))
				{
					RadioEvents->RxDone(RadioRxPayload, size, RadioPktStatus.Params.LoRa.RssiPkt, RadioPktStatus.Params.LoRa.SnrPkt);
				}
			}
		}

		if ((irqRegs & IRQ_CAD_DONE) == IRQ_CAD_DONE)
		{
            std::cout << "IRQ_CAD_DONE" << std::endl;

			//!< Update operating mode state to a value lower than \ref MODE_STDBY_XOSC
			SX126xSetOperatingMode(MODE_STDBY_RC);
			if ((RadioEvents != NULL) && (RadioEvents->CadDone != NULL))
			{
				RadioEvents->CadDone(((irqRegs & IRQ_CAD_ACTIVITY_DETECTED) == IRQ_CAD_ACTIVITY_DETECTED));
			}
		}

		if ((irqRegs & IRQ_RX_TX_TIMEOUT) == IRQ_RX_TX_TIMEOUT)
		{
            std::cout << "IRQ_RX_TX_TIMEOUT" << std::endl;

			if (SX126xGetOperatingMode() == MODE_TX)
			{
				tx_timeout_handled = true;
				TimerStop(&TxTimeoutTimer);
				//!< Update operating mode state to a value lower than \ref MODE_STDBY_XOSC
				SX126xSetOperatingMode(MODE_STDBY_RC);
				if ((RadioEvents != NULL) && (RadioEvents->TxTimeout != NULL))
				{
					RadioEvents->TxTimeout();
				}
			}
			else if (SX126xGetOperatingMode() == MODE_RX)
			{
				rx_timeout_handled = true;
				TimerStop(&RxTimeoutTimer);
				//!< Update operating mode state to a value lower than \ref MODE_STDBY_XOSC
				SX126xSetOperatingMode(MODE_STDBY_RC);
				if ((RadioEvents != NULL) && (RadioEvents->RxTimeout != NULL))
				{
					RadioEvents->RxTimeout();
				}
			}
		}

		if ((irqRegs & IRQ_PREAMBLE_DETECTED) == IRQ_PREAMBLE_DETECTED)
		{
            std::cout << "IRQ_PREAMBLE_DETECTED" << std::endl;

			if ((RadioEvents != NULL) && (RadioEvents->PreAmpDetect != NULL))
			{
				RadioEvents->PreAmpDetect();
			}
		}

		if ((irqRegs & IRQ_SYNCWORD_VALID) == IRQ_SYNCWORD_VALID)
		{
            std::cout << "IRQ_SYNCWORD_VALID" << std::endl;
			//__NOP( );
		}

		if ((irqRegs & IRQ_HEADER_VALID) == IRQ_HEADER_VALID)
		{
            std::cout << "IRQ_HEADER_VALID" << std::endl;
			//__NOP( );
		}

		if ((irqRegs & IRQ_HEADER_ERROR) == IRQ_HEADER_ERROR)
		{
            std::cout << "IRQ_HEADER_ERROR" << std::endl;

			TimerStop(&RxTimeoutTimer);
			if (RxContinuous == false)
			{
				//!< Update operating mode state to a value lower than \ref MODE_STDBY_XOSC
				SX126xSetOperatingMode(MODE_STDBY_RC);
			}
			if ((RadioEvents != NULL) && (RadioEvents->RxTimeout != NULL))
			{
				RadioEvents->RxTimeout();
			}
		}
	}
	if (TimerRxTimeout)
	{
		TimerRxTimeout = false;
		if (!rx_timeout_handled)
		{
			TimerStop(&RxTimeoutTimer);
			if ((RadioEvents != NULL) && (RadioEvents->RxTimeout != NULL))
			{
				RadioEvents->RxTimeout();
            }
		}
	}
	if (TimerTxTimeout)
	{
		TimerTxTimeout = false;
		if (!tx_timeout_handled)
		{
			TimerStop(&TxTimeoutTimer);
			if ((RadioEvents != NULL) && (RadioEvents->TxTimeout != NULL))
			{
				RadioEvents->TxTimeout();
			}
		}
	}
}

/*!
 * @brief Process radio irq
 */
void RadioHandler::IrqProcess(SX126Handler *sxHandler)
{
    BgIrqProcess(sxHandler);
}

/*!
 * @brief Process radio irq after deep sleep of the CPU
 */
void RadioHandler::IrqProcessAfterDeepSleep(SX126Handler *sxHandler)
{
	IrqFired = true;
    BgIrqProcess(sxHandler);
}

