#include <math.h>
#include <string.h>
#include "sx126x.h"
#include "SX126xHardware.h"
#include <iostream>
class SX126Handler;

/*!
 * \brief Radio registers definition
 */
typedef struct
{
	uint16_t Addr; //!< The address of the register
	uint8_t Value; //!< The value of the register
} RadioRegisters_t;

/*!
 * \brief Holds the internal operating mode of the radio
 */
static RadioOperatingModes_t OperatingMode;

/*!
 * \brief Stores the current packet type set in the radio
 */
static RadioPacketTypes_t PacketType;

/*!
 * \brief Stores the last frequency error measured on LoRa received packet
 */
volatile uint32_t FrequencyError = 0;

/*!
 * \brief Hold the status of the Image calibration
 */
static bool ImageCalibrated = false;

/*
 * SX126x DIO IRQ callback functions prototype
 */

/*!
 * \brief DIO 0 IRQ callback
 */
void SX126xOnDioIrq(void);

/*!
 * \brief DIO 0 IRQ callback
 */
void SX126xSetPollingMode(void);

/*!
 * \brief DIO 0 IRQ callback
 */
void SX126xSetInterruptMode(void);

/*
 * \brief Process the IRQ if handled by the driver
 */
void SX126xProcessIrqs(void);

void SX126xDriver::Init(SX126Handler *sxHandler)
{
    sxHandler->Reset();

    sxHandler->Wakeup();
    SetStandby(STDBY_RC, sxHandler);

    SetOperatingMode(MODE_STDBY_RC);
}

RadioOperatingModes_t SX126xDriver::GetOperatingMode(void)
{
	return OperatingMode;
}

void SX126xDriver::SetOperatingMode(RadioOperatingModes_t mode)
{
	OperatingMode = mode;
}

void SX126xDriver::CheckDeviceReady(SX126Handler *sxHandler)
{
    //RadioOperatingModes_t a = SX126xGetOperatingMode();
    if ((GetOperatingMode() == MODE_SLEEP) || (GetOperatingMode() == MODE_RX_DC))
	{
        sxHandler->Wakeup();
		// Switch is turned off when device is in sleep mode and turned on is all other modes
        //SX126xAntSwOn();
	}
    sxHandler->WaitOnBusy();
}

void SX126xDriver::SetPayload(uint8_t *payload, uint8_t size, SX126Handler *sxHandler)
{
    //std::cout << "SX126xDriver::SetPayload" << std::endl;

    sxHandler->WriteBuffer(0x00, payload, size);
}

uint8_t SX126xDriver::GetPayload(uint8_t *buffer, uint8_t *size, uint8_t maxSize, uint8_t *dataReady, SX126Handler *sxHandler)
{
    //std::cout << "SX126xDriver::GetPayload" << std::endl;
	uint8_t offset = 0;

    GetRxBufferStatus(size, &offset, sxHandler);
    if (*size >  0){
        *dataReady = 1;
    }
	if (*size > maxSize)
	{
		return 1;
	}
    sxHandler->ReadBuffer(offset, buffer, *size);
	return 0;
}

void SX126xDriver::SendPayload(uint8_t *payload, uint8_t size, uint32_t timeout, SX126Handler *sxHandler)
{
    //std::cout << "SX126xDriver::SendPayload" << std::endl;


    SetPayload(payload, size, sxHandler);
    SetTx(timeout, sxHandler);
}

uint32_t SX126xDriver::GetRandom(SX126Handler *sxHandler)
{
	uint8_t buf[] = {0, 0, 0, 0};

    sxHandler->ReadRegisters(RANDOM_NUMBER_GENERATORBASEADDR, buf, 4);

	return (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3];
}

void SX126xDriver::SetSleep(SleepParams_t sleepConfig, SX126Handler *sxHandler)
{
    sxHandler->WriteCommand(RADIO_SET_SLEEP, &sleepConfig.Value, 1);
    SetOperatingMode(MODE_SLEEP);
}

void SX126xDriver::SetStandby(RadioStandbyModes_t standbyConfig,  SX126Handler *sxHandler)
{
    sxHandler->WriteCommand(RADIO_SET_STANDBY, (uint8_t *)&standbyConfig, 1);
	if (standbyConfig == STDBY_RC)
	{
        SetOperatingMode(MODE_STDBY_RC);
	}
	else
	{
        SetOperatingMode(MODE_STDBY_XOSC);
	}
}

void SX126xDriver::SetFs(SX126Handler *sxHandler)
{
    sxHandler->WriteCommand(RADIO_SET_FS, 0, 0);
    SetOperatingMode(MODE_FS);
}

void SX126xDriver::SetTx(uint32_t timeout, SX126Handler *sxHandler)
{
	uint8_t buf[3];

    SetOperatingMode(MODE_TX);

	buf[0] = (uint8_t)((timeout >> 16) & 0xFF);
	buf[1] = (uint8_t)((timeout >> 8) & 0xFF);
	buf[2] = (uint8_t)(timeout & 0xFF);
    sxHandler->WriteCommand(RADIO_SET_TX, buf, 3);
}

void SX126xDriver::SetRx(uint32_t timeout, SX126Handler *sxHandler)
{
	uint8_t buf[3];

    SetOperatingMode(MODE_RX);

	buf[0] = (uint8_t)((timeout >> 16) & 0xFF);
	buf[1] = (uint8_t)((timeout >> 8) & 0xFF);
	buf[2] = (uint8_t)(timeout & 0xFF);
    sxHandler->WriteCommand(RADIO_SET_RX, buf, 3);
}

void SX126xDriver::SetRxBoosted(uint32_t timeout, SX126Handler *sxHandler)
{
	uint8_t buf[3];

    SetOperatingMode(MODE_RX);

    sxHandler->WriteRegister(REG_RX_GAIN, 0x96); // max LNA gain, increase current by ~2mA for around ~3dB in sensivity

	buf[0] = (uint8_t)((timeout >> 16) & 0xFF);
	buf[1] = (uint8_t)((timeout >> 8) & 0xFF);
	buf[2] = (uint8_t)(timeout & 0xFF);
    sxHandler->WriteCommand(RADIO_SET_RX, buf, 3);
}

void SX126xDriver::SetRxDutyCycle(uint32_t rxTime, uint32_t sleepTime, SX126Handler *sxHandler)
{
	uint8_t buf[6];

	buf[0] = (uint8_t)((rxTime >> 16) & 0xFF);
	buf[1] = (uint8_t)((rxTime >> 8) & 0xFF);
	buf[2] = (uint8_t)(rxTime & 0xFF);
	buf[3] = (uint8_t)((sleepTime >> 16) & 0xFF);
	buf[4] = (uint8_t)((sleepTime >> 8) & 0xFF);
	buf[5] = (uint8_t)(sleepTime & 0xFF);
    sxHandler->WriteCommand(RADIO_SET_RXDUTYCYCLE, buf, 6);
    SetOperatingMode(MODE_RX_DC);
}

void SX126xDriver::SetCad(SX126Handler *sxHandler)
{
    sxHandler->WriteCommand(RADIO_SET_CAD, 0, 0);
    SetOperatingMode(MODE_CAD);
}

void SX126xDriver::SetTxContinuousWave(SX126Handler *sxHandler)
{
    sxHandler->WriteCommand(RADIO_SET_TXCONTINUOUSWAVE, 0, 0);
}

void SX126xDriver::SetTxInfinitePreamble(SX126Handler *sxHandler)
{
    sxHandler->WriteCommand(RADIO_SET_TXCONTINUOUSPREAMBLE, 0, 0);
}

void SX126xDriver::SetStopRxTimerOnPreambleDetect(bool enable, SX126Handler *sxHandler)
{
    sxHandler->WriteCommand(RADIO_SET_STOPRXTIMERONPREAMBLE, (uint8_t *)&enable, 1);
}

void SX126xDriver::SetLoRaSymbNumTimeout(uint8_t SymbNum, SX126Handler *sxHandler)
{
    sxHandler->WriteCommand(RADIO_SET_LORASYMBTIMEOUT, &SymbNum, 1);
}

void SX126xDriver::SetRegulatorMode(RadioRegulatorMode_t mode, SX126Handler *sxHandler)
{
    sxHandler->WriteCommand(RADIO_SET_REGULATORMODE, (uint8_t *)&mode, 1);
}

void SX126xDriver::Calibrate(CalibrationParams_t calibParam, SX126Handler *sxHandler)
{
    sxHandler->WriteCommand(RADIO_CALIBRATE, (uint8_t *)&calibParam, 1);
}

void SX126xDriver::CalibrateImage(uint32_t freq, SX126Handler *sxHandler)
{
	uint8_t calFreq[2];

	if (freq > 900000000)
	{
		calFreq[0] = 0xE1;
		calFreq[1] = 0xE9;
	}
	else if (freq > 850000000)
	{
		calFreq[0] = 0xD7;
		calFreq[1] = 0xDB;
	}
    else if (freq > 770000000)
	{
		calFreq[0] = 0xC1;
		calFreq[1] = 0xC5;
	}
	else if (freq > 460000000)
	{
		calFreq[0] = 0x75;
		calFreq[1] = 0x81;
	}
	else if (freq > 425000000)
	{
		calFreq[0] = 0x6B;
		calFreq[1] = 0x6F;
	}
    sxHandler->WriteCommand(RADIO_CALIBRATEIMAGE, calFreq, 2);
}

void SX126xDriver::SetPaConfig(uint8_t paDutyCycle, uint8_t hpMax, uint8_t deviceSel, uint8_t paLut, SX126Handler *sxHandler)
{
	uint8_t buf[4];

	buf[0] = paDutyCycle;
	buf[1] = hpMax;
	buf[2] = deviceSel;
	buf[3] = paLut;
    sxHandler->WriteCommand(RADIO_SET_PACONFIG, buf, 4);
}

void SX126xDriver::SetRxTxFallbackMode(uint8_t fallbackMode, SX126Handler *sxHandler)
{
    sxHandler->WriteCommand(RADIO_SET_TXFALLBACKMODE, &fallbackMode, 1);
}

void SX126xDriver::SetDioIrqParams(uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask, SX126Handler *sxHandler)
{
	uint8_t buf[8];

	buf[0] = (uint8_t)((irqMask >> 8) & 0x00FF);
	buf[1] = (uint8_t)(irqMask & 0x00FF);
	buf[2] = (uint8_t)((dio1Mask >> 8) & 0x00FF);
	buf[3] = (uint8_t)(dio1Mask & 0x00FF);
	buf[4] = (uint8_t)((dio2Mask >> 8) & 0x00FF);
	buf[5] = (uint8_t)(dio2Mask & 0x00FF);
	buf[6] = (uint8_t)((dio3Mask >> 8) & 0x00FF);
	buf[7] = (uint8_t)(dio3Mask & 0x00FF);
    sxHandler->WriteCommand(RADIO_CFG_DIOIRQ, buf, 8);
}

uint16_t SX126xDriver::GetIrqStatus(SX126Handler *sxHandler)
{
	uint8_t irqStatus[2];

    sxHandler->ReadCommand(RADIO_GET_IRQSTATUS, irqStatus, 2);
	return (irqStatus[0] << 8) | irqStatus[1];
}

void SX126xDriver::SetRfFrequency(uint32_t frequency, SX126Handler *sxHandler)
{
	uint8_t buf[4];
	uint32_t freq = 0;

	if (ImageCalibrated == false)
	{
        CalibrateImage(frequency, sxHandler);
		ImageCalibrated = true;
	}

	freq = (uint32_t)((double)frequency / (double)FREQ_STEP);
	buf[0] = (uint8_t)((freq >> 24) & 0xFF);
	buf[1] = (uint8_t)((freq >> 16) & 0xFF);
	buf[2] = (uint8_t)((freq >> 8) & 0xFF);
	buf[3] = (uint8_t)(freq & 0xFF);
    sxHandler->WriteCommand(RADIO_SET_RFFREQUENCY, buf, 4);
}

void SX126xDriver::SetPacketType(RadioPacketTypes_t packetType, SX126Handler *sxHandler)
{
	// Save packet type internally to avoid questioning the radio
	PacketType = packetType;
    sxHandler->WriteCommand(RADIO_SET_PACKETTYPE, (uint8_t *)&packetType, 1);
}

RadioPacketTypes_t SX126xDriver::GetPacketType(void)
{
	return PacketType;
}

void SX126xDriver::SetTxParams(int8_t power, RadioRampTimes_t rampTime, SX126Handler *sxHandler)
{
	uint8_t buf[2];

    // WORKAROUND - Better Resistance of the SX1262 Tx to Antenna Mismatch, see DS_SX1261-2_V1.2 datasheet chapter 15.2
    // RegTxClampConfig = @address 0x08D8
    sxHandler->WriteRegister(0x08D8, sxHandler->ReadRegister(0x08D8) | (0x0F << 1));
    // WORKAROUND END

    SetPaConfig(0x04, 0x07, 0x00, 0x01, sxHandler);
    if (power > 22)
    {
        power = 22;
    }
    else if (power < -9)
    {
        power = -9;
    }
    sxHandler->WriteRegister(REG_OCP, 0x38); // current max 160mA for the whole device
	buf[0] = power;
	buf[1] = (uint8_t)rampTime;
    sxHandler->WriteCommand(RADIO_SET_TXPARAMS, buf, 2);
}

void SX126xDriver::SetModulationParams(ModulationParams_t *modulationParams, SX126Handler *sxHandler)
{
	uint8_t n;
	uint32_t tempVal = 0;
	uint8_t buf[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

	// Check if required configuration corresponds to the stored packet type
	// If not, silently update radio packet type
	if (PacketType != modulationParams->PacketType)
	{
        SetPacketType(modulationParams->PacketType, sxHandler);
	}

	switch (modulationParams->PacketType)
	{
	case PACKET_TYPE_LORA:
		n = 4;
		buf[0] = modulationParams->Params.LoRa.SpreadingFactor;
		buf[1] = modulationParams->Params.LoRa.Bandwidth;
		buf[2] = modulationParams->Params.LoRa.CodingRate;
		buf[3] = modulationParams->Params.LoRa.LowDatarateOptimize;

        sxHandler->WriteCommand(RADIO_SET_MODULATIONPARAMS, buf, n);

		break;
	default:
	case PACKET_TYPE_NONE:
		return;
	}
}

void SX126xDriver::SetPacketParams(PacketParams_t *packetParams, SX126Handler *sxHandler)
{
	uint8_t n;
	uint8_t buf[9] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

	// Check if required configuration corresponds to the stored packet type
	// If not, silently update radio packet type
	if (PacketType != packetParams->PacketType)
	{
        SetPacketType(packetParams->PacketType, sxHandler);
	}

	switch (packetParams->PacketType)
	{
	case PACKET_TYPE_LORA:
		n = 6;
		buf[0] = (packetParams->Params.LoRa.PreambleLength >> 8) & 0xFF;
		buf[1] = packetParams->Params.LoRa.PreambleLength;
		buf[2] = packetParams->Params.LoRa.HeaderType;
		buf[3] = packetParams->Params.LoRa.PayloadLength;
		buf[4] = packetParams->Params.LoRa.CrcMode;
		buf[5] = packetParams->Params.LoRa.InvertIQ;
		break;
	default:
	case PACKET_TYPE_NONE:
		return;
	}
    sxHandler->WriteCommand(RADIO_SET_PACKETPARAMS, buf, n);
}

void SX126xDriver::SetCadParams(RadioLoRaCadSymbols_t cadSymbolNum, uint8_t cadDetPeak, uint8_t cadDetMin, RadioCadExitModes_t cadExitMode, uint32_t cadTimeout, SX126Handler *sxHandler)
{
	uint8_t buf[7];

	buf[0] = (uint8_t)cadSymbolNum;
	buf[1] = cadDetPeak;
	buf[2] = cadDetMin;
	buf[3] = (uint8_t)cadExitMode;
	buf[4] = (uint8_t)((cadTimeout >> 16) & 0xFF);
	buf[5] = (uint8_t)((cadTimeout >> 8) & 0xFF);
	buf[6] = (uint8_t)(cadTimeout & 0xFF);
    sxHandler->WriteCommand(RADIO_SET_CADPARAMS, buf, 7);
	OperatingMode = MODE_CAD;
}

void SX126xDriver::SetBufferBaseAddress(uint8_t txBaseAddress, uint8_t rxBaseAddress, SX126Handler *sxHandler)
{
	uint8_t buf[2];

	buf[0] = txBaseAddress;
	buf[1] = rxBaseAddress;
    sxHandler->WriteCommand(RADIO_SET_BUFFERBASEADDRESS, buf, 2);
}

RadioStatus_t SX126xDriver::GetStatus(SX126Handler *sxHandler)
{
	uint8_t stat = 0;
	RadioStatus_t status;
    //std::cout << "SX126xGetStatus: " << std::endl;
    sxHandler->ReadCommand(RADIO_GET_STATUS, (uint8_t *)&stat, 1);
	status.Value = stat;
	return status;
}

int8_t SX126xDriver::GetRssiInst(SX126Handler *sxHandler)
{
	uint8_t buf[1];
	int8_t rssi = 0;

    sxHandler->ReadCommand(RADIO_GET_RSSIINST, buf, 1);
	rssi = -buf[0] >> 1;
	return rssi;
}

void SX126xDriver::GetRxBufferStatus(uint8_t *payloadLength, uint8_t *rxStartBufferPointer, SX126Handler *sxHandler)
{
	uint8_t status[2];
    //std::cout << "SX126xGetRxBufferStatus" << std::endl;
    sxHandler->ReadCommand(RADIO_GET_RXBUFFERSTATUS, status, 2);

	// In case of LORA fixed header, the payloadLength is obtained by reading
	// the register REG_LR_PAYLOADLENGTH
    if ((GetPacketType() == PACKET_TYPE_LORA) && (sxHandler->ReadRegister(REG_LR_PACKETPARAMS) >> 7 == 1))
	{
        *payloadLength = sxHandler->ReadRegister(REG_LR_PAYLOADLENGTH);
	}
	else
	{
		*payloadLength = status[0];
	}
	*rxStartBufferPointer = status[1];
}

void SX126xDriver::GetPacketStatus(PacketStatus_t *pktStatus, SX126Handler *sxHandler)
{
	uint8_t status[3];

    sxHandler->ReadCommand(RADIO_GET_PACKETSTATUS, status, 3);

    pktStatus->packetType = GetPacketType();
	switch (pktStatus->packetType)
	{
	case PACKET_TYPE_LORA:
		pktStatus->Params.LoRa.RssiPkt = -status[0] >> 1;
		// Returns SNR value [dB] rounded to the nearest integer value
		pktStatus->Params.LoRa.SnrPkt = (((int8_t)status[1]) + 2) >> 2;
		pktStatus->Params.LoRa.SignalRssiPkt = -status[2] >> 1;
		pktStatus->Params.LoRa.FreqError = FrequencyError;
		break;

	default:
	case PACKET_TYPE_NONE:
		// In that specific case, we set everything in the pktStatus to zeros
		// and reset the packet type accordingly
		memset(pktStatus, 0, sizeof(PacketStatus_t));
		pktStatus->packetType = PACKET_TYPE_NONE;
		break;
	}
}

RadioError_t SX126xDriver::GetDeviceErrors(SX126Handler *sxHandler)
{
    std::cout << "SX126xGetDeviceErrors" << std::endl;
	RadioError_t error;

    sxHandler->ReadCommand(RADIO_GET_ERROR, (uint8_t *)&error, 2);
	return error;
}

void SX126xDriver::ClearDeviceErrors(SX126Handler *sxHandler)
{
	uint8_t buf[2] = {0x00, 0x00};
    sxHandler->WriteCommand(RADIO_CLR_ERROR, buf, 2);
}

void SX126xDriver::ClearIrqStatus(uint16_t irq, SX126Handler *sxHandler)
{
	uint8_t buf[2];

	buf[0] = (uint8_t)(((uint16_t)irq >> 8) & 0x00FF);
	buf[1] = (uint8_t)((uint16_t)irq & 0x00FF);
    sxHandler->WriteCommand(RADIO_CLR_IRQSTATUS, buf, 2);
}
