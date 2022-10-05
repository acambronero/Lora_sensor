#include <math.h>
#include <string.h>
#include <unistd.h>
#include "SX126xHardware.h"
#include "mcu/timer.h"
#include "radiohandler.h"
//#include "itools.h"
#ifdef RASPI
#include "wiringPi.h"
#endif
#include <iostream>
#include <functional>

#define TX_BUFFER_BASE_ADDRESS 0x00
#define RX_BUFFER_BASE_ADDRESS 0x80

bool RadioHandler::IrqFired = false;

RadioHandler::RadioHandler()
{
    sxDriver = new SX126xDriver();
}

/*!
 * @brief Initializes the radio
 *
 * @param  events Structure containing the driver callback functions
 */
void RadioHandler::Init(SX126Handler *sxHandler) {

    sxDriver->Init(sxHandler);
    sxDriver->SetStandby(STDBY_RC, sxHandler);
    sxDriver->SetBufferBaseAddress(TX_BUFFER_BASE_ADDRESS, RX_BUFFER_BASE_ADDRESS, sxHandler);
    sxDriver->SetTxParams(0, RADIO_RAMP_200_US, sxHandler);

    //Initialize driver timeout timers

    TimerInit(&RxTimeoutTimer, std::bind(&RadioHandler::OnRxTimeoutIrq, this));
    TimerInit(&TxTimeoutTimer, std::bind(&RadioHandler::OnTxTimeoutIrq, this));


	IrqFired = false;
}

void RadioHandler::SetIrqsEnable(std::vector<IrqsActivated> irq)
{
    irqsEnable = irq;
}

/*!
 * Return current radio status
 *
 * @retval Radio status.[RF_IDLE, RF_RX_RUNNING, RF_TX_RUNNING]
 */
RadioState_t RadioHandler::GetStatus(SX126Handler *sxHandler)
{
    switch (sxDriver->GetOperatingMode())
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
    sxDriver->SetPacketType(PACKET_TYPE_LORA, sxHandler);
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
    sxDriver->SetRfFrequency(freq, sxHandler);
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
    sxDriver->SetRx(0, sxHandler);

    rnd = sxDriver->GetRandom(sxHandler);
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
void RadioHandler::SetRxConfig (uint32_t bandwidth, uint32_t datarate, uint8_t coderate,
                                uint16_t preambleLen, uint16_t symbTimeout, bool fixLen,
                                uint8_t payloadLen, bool crcOn, bool iqInverted,
                                bool rxContinuous, uint32_t RxTimeout, SX126Handler* sxHandler)
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


    sxDriver->SetStopRxTimerOnPreambleDetect(false, sxHandler);
    sxDriver->SetLoRaSymbNumTimeout(symbTimeout, sxHandler);
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
    sxDriver->SetModulationParams(&SX126x.ModulationParams, sxHandler);
    sxDriver->SetPacketParams(&SX126x.PacketParams, sxHandler);

    // WORKAROUND - Optimizing the Inverted IQ Operation, see DS_SX1261-2_V1.2 datasheet chapter 15.4
    if (SX126x.PacketParams.Params.LoRa.InvertIQ == LORA_IQ_INVERTED)
    {
        // RegIqPolaritySetup = @address 0x0736
        sxHandler->WriteRegister(0x0736, sxHandler->ReadRegister(0x0736) & ~(1 << 2));
    }
    else
    {
        // RegIqPolaritySetup @address 0x0736
        sxHandler->WriteRegister(0x0736, sxHandler->ReadRegister(0x0736) | (1 << 2));
    }
    // WORKAROUND END

    // Timeout Max, Timeout handled directly in SetRx function
    //RxTimeout = 0xFA0;
    this->RxTimeout = RxTimeout;
    //std::cout << "timeout:RadioHandler::Rxconfig: " << RxTimeout << std::endl;

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
void RadioHandler::SetTxConfig(int8_t power, uint32_t bandwidth, uint32_t datarate,
                               uint8_t coderate, uint16_t preambleLen, bool fixLen,
                               bool crcOn, bool iqInverted, uint32_t timeout,
                               SX126Handler* sxHandler)
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
    sxDriver->SetModulationParams(&SX126x.ModulationParams, sxHandler);
    sxDriver->SetPacketParams(&SX126x.PacketParams, sxHandler);

	// WORKAROUND - Modulation Quality with 500 kHz LoRa Bandwidth, see DS_SX1261-2_V1.2 datasheet chapter 15.1
    if (SX126x.ModulationParams.Params.LoRa.Bandwidth == LORA_BW_500)
	{
		// RegTxModulation = @address 0x0889
        sxHandler->WriteRegister(0x0889, sxHandler->ReadRegister(0x0889) & ~(1 << 2));
	}
	else
	{
		// RegTxModulation = @address 0x0889
        sxHandler->WriteRegister(0x0889, sxHandler->ReadRegister(0x0889) | (1 << 2));
	}
	// WORKAROUND END

    sxHandler->SetRfTxPower(power);
    this->TxTimeout = timeout;
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
void RadioHandler::Send(uint8_t *buffer, uint8_t size, uint32_t TxTimeout, uint32_t RxTimeout, SX126Handler *sxHandler)
{
    //std::cout << "RadioHandler::Send" << std::endl;

    sxDriver->SetDioIrqParams(IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT,
                              IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT,
                              IRQ_RADIO_NONE,
                              IRQ_RADIO_NONE, sxHandler);

    if (sxDriver->GetPacketType() == PACKET_TYPE_LORA)
	{
		SX126x.PacketParams.Params.LoRa.PayloadLength = size;
	}
	else
	{
		SX126x.PacketParams.Params.Gfsk.PayloadLength = size;
	}
    sxDriver->SetPacketParams(&SX126x.PacketParams, sxHandler);

    if (TxTimeout != 0){
        this->TxTimeout = TxTimeout;
    }
    if (RxTimeout != 0){
        this->RxTimeout = RxTimeout;
    }
    sxDriver->SendPayload(buffer, size, this->TxTimeout, sxHandler);
    TimerSetValue(&TxTimeoutTimer, this->TxTimeout);
    TimerStart(&TxTimeoutTimer);
    //TimerTxFired = true;
}

/*!
 * @brief Sets the radio in sleep mode
 */
void RadioHandler::Sleep(SX126Handler *sxHandler)
{
	SleepParams_t params = {0};

	params.Fields.WarmStart = 1;
    sxDriver->SetSleep(params, sxHandler);

    sleep_microseconds(2);
}

/*!
 * @brief Sets the radio in standby mode
 */
void RadioHandler::Standby(SX126Handler *sxHandler)
{
    sxDriver->SetStandby(STDBY_RC, sxHandler);
}

/*!
 * @brief Sets the radio in reception mode for the given time
 * @param  timeout Reception timeout [ms]
 *                     [0: continuous, others timeout]
 */
void RadioHandler::Rx(uint32_t timeout, SX126Handler *sxHandler)
{
	if (RxContinuous == true)
	{
		// Even Continous mode is selected, put a timeout here
		if (timeout != 0)
		{
			TimerSetValue(&RxTimeoutTimer, timeout);
			TimerStart(&RxTimeoutTimer);
		}
        sxDriver->SetRx(0xFFFFFF, sxHandler); // Rx Continuous
	}
	else
	{
        TimerSetValue(&RxTimeoutTimer, timeout);
        TimerStart(&RxTimeoutTimer);
        sxDriver->SetRx(timeout << 6, sxHandler);
	}
}

/*!
 * @brief Sets the radio in reception mode with Max LNA gain for the given time
 * @param  timeout Reception timeout [ms]
 *                     [0: continuous, others timeout]
 */
void RadioHandler::RxBoosted(uint32_t timeout, SX126Handler *sxHandler)
{
    sxDriver->SetDioIrqParams(IRQ_RADIO_ALL, //IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT,
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
        sxDriver->SetRxBoosted(0xFFFFFF, sxHandler); // Rx Continuous
	}
	else
	{
        sxDriver->SetRxBoosted(RxTimeout << 6, sxHandler);
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
    sxDriver->SetDioIrqParams(IRQ_RADIO_ALL | IRQ_RX_TX_TIMEOUT,
						  IRQ_RADIO_ALL | IRQ_RX_TX_TIMEOUT,
                          IRQ_RADIO_NONE, IRQ_RADIO_NONE, sxHandler);
    sxDriver->SetRxDutyCycle(rxTime, sleepTime, sxHandler);
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
    sxDriver->SetCadParams((RadioLoRaCadSymbols_t)cadSymbolNum, cadDetPeak, cadDetMin, (RadioCadExitModes_t)cadExitMode, cadTimeout, sxHandler);
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
    sxDriver->SetDioIrqParams(IRQ_CAD_DONE | IRQ_CAD_ACTIVITY_DETECTED,
						  IRQ_CAD_DONE | IRQ_CAD_ACTIVITY_DETECTED,
                          IRQ_RADIO_NONE, IRQ_RADIO_NONE, sxHandler);
    sxDriver->SetCad(sxHandler);
}

void RadioHandler::Tx(uint32_t timeout, SX126Handler *sxHandler)
{
    sxDriver->SetTx(timeout << 6, sxHandler);
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
    sxDriver->SetRfFrequency(freq, sxHandler);
    sxHandler->SetRfTxPower(power);
    sxDriver->SetTxContinuousWave(sxHandler);

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
    return sxDriver->GetRssiInst(sxHandler);
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
        sxDriver->SetPacketParams(&SX126x.PacketParams, sxHandler);
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
        sxHandler->WriteRegister(REG_LR_SYNCWORD, (LORA_MAC_PUBLIC_SYNCWORD >> 8) & 0xFF);
        sxHandler->WriteRegister(REG_LR_SYNCWORD + 1, LORA_MAC_PUBLIC_SYNCWORD & 0xFF);
	}
	else
	{
		// Change LoRa modem SyncWord
        sxHandler->WriteRegister(REG_LR_SYNCWORD, (LORA_MAC_PRIVATE_SYNCWORD >> 8) & 0xFF);
        sxHandler->WriteRegister(REG_LR_SYNCWORD + 1, LORA_MAC_PRIVATE_SYNCWORD & 0xFF);
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
    std::cout << "RadioHandler::OnTxTimeoutIrq" << std::endl;

    TimerTxTimeout = true;
    //TimerTxFired = false;
}

/*!
 * @brief Rx timeout timer callback
 */
void RadioHandler::OnRxTimeoutIrq(void)
{
    std::cout << "RadioHandler::OnRxTimeoutIrq" << std::endl;

    TimerRxTimeout = true;
    //TimerRxFired = false;
}


/*!
 * @brief Process radio irq in background task (nRF52 & ESP32)
 */
void RadioHandler::BgIrqProcess(uint8_t *dataReady, SX126Handler *sxHandler)
{
	bool rx_timeout_handled = false;
	bool tx_timeout_handled = false;
    RadioStatus_t st;
    uint16_t irqRegs = 0;
    irqRegs = sxDriver->GetIrqStatus(sxHandler);
    //std::cout << "Estado radio: " << irqRegs << std::endl;
    st = sxDriver->GetStatus(sxHandler);
    std::cout << "Estado radio: " << std::hex << (int)st.Value << std::endl;
    sxDriver->ClearIrqStatus(IRQ_RADIO_ALL, sxHandler);
    if (irqRegs == 0 ) return;

    if ((irqRegs & IRQ_TX_DONE) == IRQ_TX_DONE) {
        std::cout << "IRQ_TX_DONE" << std::endl;
        tx_timeout_handled = true;
        TimerStop(&TxTimeoutTimer);
        //!< Update operating mode state to a value lower than \ref MODE_STDBY_XOSC
        sxDriver->SetOperatingMode(MODE_STDBY_RC);
        for(auto irq : irqsEnable) {
            if (irq == IRQ_ENABLE_TX_DONE){
                OnTxDone();
            }
        }
    }

    if ((irqRegs & IRQ_RX_DONE) == IRQ_RX_DONE)
    {
        std::cout << "IRQ_RX_DONE" << std::endl;

        uint8_t size;

        rx_timeout_handled = true;

        if (RxContinuous == false)
        {
            //std::cout << "TIMER RX DONE" << std::endl;

            TimerStop(&RxTimeoutTimer);
            //!< Update operating mode state to a value lower than \ref MODE_STDBY_XOSC
            sxDriver->SetOperatingMode(MODE_STDBY_RC);

            // WORKAROUND - Implicit Header Mode Timeout Behavior, see DS_SX1261-2_V1.2 datasheet chapter 15.3
            // RegRtcControl = @address 0x0902
            sxHandler->WriteRegister(0x0902, 0x00);
            // RegEventMask = @address 0x0944
            sxHandler->WriteRegister(0x0944, sxHandler->ReadRegister(0x0944) | (1 << 1));
            // WORKAROUND END
        }
        memset(RadioRxPayload, 0, 255);

        if ((irqRegs & IRQ_CRC_ERROR) == IRQ_CRC_ERROR)
        {
            std::cout << "IRQ_CRC_ERROR" << std::endl;

            uint8_t size;
            // Discard buffer
            memset(RadioRxPayload, 0, 255);
            sxDriver->GetPayload(RadioRxPayload, &size, 255, dataReady, sxHandler);
            sxDriver->GetPacketStatus(&RadioPktStatus, sxHandler);
            for(auto irq : irqsEnable) {
                if (irq == IRQ_ENABLE_CRC_ERROR){
                    OnRxError();
                }
            }
        }
        else
        {
            sxDriver->GetPayload(RadioRxPayload, &size, 255, dataReady, sxHandler);
            sxDriver->GetPacketStatus(&RadioPktStatus, sxHandler);
            for(auto irq : irqsEnable) {
                if (irq == IRQ_ENABLE_RX_DONE){
                    OnRxDone(RadioRxPayload, size, RadioPktStatus.Params.LoRa.RssiPkt, RadioPktStatus.Params.LoRa.SnrPkt);
                }
            }
        }
    }

    if ((irqRegs & IRQ_CAD_DONE) == IRQ_CAD_DONE)
    {
        //std::cout << "IRQ_CAD_DONE" << std::endl;

        //!< Update operating mode state to a value lower than \ref MODE_STDBY_XOSC
        sxDriver->SetOperatingMode(MODE_STDBY_RC);
        for(auto irq : irqsEnable) {
            if (irq == IRQ_ENABLE_CAD_DONE){
                OnCadDone(((irqRegs & IRQ_CAD_ACTIVITY_DETECTED) == IRQ_CAD_ACTIVITY_DETECTED));
            }
        }
    }

    if ((irqRegs & IRQ_RX_TX_TIMEOUT) == IRQ_RX_TX_TIMEOUT)
    {
        std::cout << "IRQ_RX_TX_TIMEOUT" << std::endl;

        if (sxDriver->GetOperatingMode() == MODE_TX)
        {
            tx_timeout_handled = true;
            TimerStop(&TxTimeoutTimer);
            //!< Update operating mode state to a value lower than \ref MODE_STDBY_XOSC
            sxDriver->SetOperatingMode(MODE_STDBY_RC);
            for(auto irq : irqsEnable) {
                if (irq == IRQ_ENABLE_RX_TX_TIMEOUT){
                    OnTxTimeout();
                }
            }
        }
        else if (sxDriver->GetOperatingMode() == MODE_RX)
        {
            rx_timeout_handled = true;
            TimerStop(&RxTimeoutTimer);
            //!< Update operating mode state to a value lower than \ref MODE_STDBY_XOSC
            sxDriver->SetOperatingMode(MODE_STDBY_RC);
            for(auto irq : irqsEnable) {
                if (irq == IRQ_ENABLE_RX_TX_TIMEOUT){
                    OnRxTimeout();
                }
            }
        }
    }

    if ((irqRegs & IRQ_PREAMBLE_DETECTED) == IRQ_PREAMBLE_DETECTED)
    {
        std::cout << "IRQ_PREAMBLE_DETECTED" << std::endl;

        for(auto irq : irqsEnable) {
            if (irq == IRQ_ENABLE_PREAMBLE_DETECTED){
                //OnPreAmpDetect();
            }
        }
    }

    if ((irqRegs & IRQ_SYNCWORD_VALID) == IRQ_SYNCWORD_VALID)
    {
        //std::cout << "IRQ_SYNCWORD_VALID" << std::endl;
        //__NOP( );
    }

    if ((irqRegs & IRQ_HEADER_VALID) == IRQ_HEADER_VALID)
    {
        //std::cout << "IRQ_HEADER_VALID" << std::endl;
        //__NOP( );
    }

    if ((irqRegs & IRQ_HEADER_ERROR) == IRQ_HEADER_ERROR)
    {
        //std::cout << "IRQ_HEADER_ERROR" << std::endl;

        TimerStop(&RxTimeoutTimer);
        if (RxContinuous == false)
        {
            //!< Update operating mode state to a value lower than \ref MODE_STDBY_XOSC
            sxDriver->SetOperatingMode(MODE_STDBY_RC);
        }

        for(auto irq : irqsEnable) {
            if (irq == IRQ_ENABLE_HEADER_ERROR){
                OnRxTimeout();
            }
        }

    }
    std::cout << "TIMERS" << std::endl;
    if (TimerRxTimeout)
    {
        TimerRxTimeout = false;
        if (!rx_timeout_handled)
        {
            std::cout << "TimerRxTimeout::IrqProcess" << std::endl;
            TimerStop(&RxTimeoutTimer);
            for(auto irq : irqsEnable) {
                if (irq == IRQ_ENABLE_RX_TX_TIMEOUT){
                    OnRxTimeout();
                }
            }
        }
    }
    if (TimerTxTimeout)
	{
        //std::cout << "TimerTxTimeout::IrqProcess" << std::endl;
		TimerTxTimeout = false;
		if (!tx_timeout_handled)
		{
			TimerStop(&TxTimeoutTimer);
            for(auto irq : irqsEnable) {
                if (irq == IRQ_ENABLE_RX_TX_TIMEOUT){
                    OnTxTimeout();
                }
            }
		}
	}
}

/*!
 * @brief Process radio irq
 */
void RadioHandler::IrqProcess(uint8_t *dataReady, SX126Handler *sxHandler)
{
    BgIrqProcess(dataReady, sxHandler);
}

/*!
 * @brief Process radio irq after deep sleep of the CPU
 */
void RadioHandler::IrqProcessAfterDeepSleep(uint8_t *dataReady, SX126Handler *sxHandler)
{
	IrqFired = true;
    BgIrqProcess(dataReady, sxHandler);
}

void RadioHandler::OnTxDone()
{
    //std::cout << "OnTxDone" << std::endl;
    TimerTxTimeout = true;
    //TimerTxFired = false;

    sxDriver->ClearIrqStatus(IRQ_RADIO_ALL, sxHandler);
    sxDriver->SetDioIrqParams(IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT,
                              IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT,
                              IRQ_RADIO_NONE, IRQ_RADIO_NONE, sxHandler);

    Rx(this->RxTimeout, sxHandler);
}

void RadioHandler::OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
    //sxDriver->ClearIrqStatus(IRQ_RADIO_ALL, sxHandler);
    //sxDriver->SetDioIrqParams(IRQ_PREAMBLE_DETECTED, IRQ_PREAMBLE_DETECTED, IRQ_RADIO_NONE, IRQ_RADIO_NONE, sxHandler);
    //Rx(0, sxHandler);

    /* Switch case depends on the payload receive
     *
     *
     *
     *
     *
     *
     *
     * */

    // then shut down the radio
}

void RadioHandler::OnTxTimeout()
{
    triesToSend++;
    //Something went wrong. Trying to send again
    std::cout << "Unable to send, let's try again. Attempt: " << triesToSend << std::endl;

    sxDriver->SetTx(this->TxTimeout, sxHandler);
    TimerSetValue(&TxTimeoutTimer, this->TxTimeout);
    TimerStart(&TxTimeoutTimer);
    //Let's try 3 times
}

void RadioHandler::OnRxTimeout()
{
    //we havent received anything at the RX window
    //shut down
    sxDriver->SetStandby(STDBY_RC, sxHandler);
}

void RadioHandler::OnRxError()
{

}

void RadioHandler::OnPreAmpDetect()
{

}

void RadioHandler::OnFhssChangeChannel(uint8_t currentChannel)
{

}

void RadioHandler::OnCadDone(bool channelActivityDetected)
{

}

std::array<uint8_t, 255> RadioHandler::GetPayloadData(uint16_t size)
{
    //std::cout << "RadioHandler::GetPayloadData" << std::endl;
    std::array<uint8_t, 255> data;
    std::copy(std::begin(RadioRxPayload), std::end(RadioRxPayload), std::begin(data));

    return data;
}

void RadioHandler::decode_message()
{
    int index = 0;
    std::string SFD;
    SFD.push_back((char)empty_buffer(RcvBuffer, 1, index));
    SFD.push_back((char)empty_buffer(RcvBuffer, 1, index));

    if (SFD != "FS") return;

    /*message_type = (uint8_t)empty_buffer(RcvBuffer, 1, index);
    command = (uint8_t)empty_buffer(RcvBuffer, 1, index);
    deviceTag = (uint32_t) empty_buffer(RcvBuffer, 4, index);
    version = (uint8_t)empty_buffer(RcvBuffer, 1, index);
    timestamp = empty_buffer(RcvBuffer, 8, index);
    deviceId = (uint8_t)empty_buffer(RcvBuffer, 1, index);*/

    //To define

    switch (command)
    {
        case COMMAND_ACQUISITION_DATA:
        {
            //DataHelperResponse msg;
            //prepare_acquisition_data(msg);
            //set_buffer_data(TxdBuffer, &msg);
            break;
        }
        case COMMAND_WRITE_CONFIGURATION:
        {
            DataHelperRequest msg = {
                (uint8_t)MESSAGE_TYPE_REQUEST,
                (uint8_t)COMMAND_ACQUISITION_DATA,
                0x260B269E,
                0x1,
                0x260B269EDE32ABCE,
                0x1,
            };
            set_buffer_data(TxdBuffer, &msg);
            break;
        }
        default:
            break;
    }
}

DataHelperRequest RadioHandler::prepare_acquisition_data()
{
    DataHelperRequest msg = {
        (uint8_t)message_type,
        (uint8_t)command,
        0x260B269E,
        0x1,
        0x260B269EDE32ABCE,
        0x1,
        0x1,
        0x1,
        0x1,
    };
    return msg;
}



