#include "unistd.h"
#include "chrono"
#include "LoRaMacHelper.h"
#include "region/RegionCommon.h"
#include "system/utilities.h"
#include "datahelper.h"
#include "region/RegionEU433.h"
#include "region/RegionEU868.h"
//#include "itools.h"

#include "mac/LoRaMacTest.h"
//using namespace itop::ITools;

bool lmh_mac_is_busy = false;

#if defined(REGION_EU868)

#include "LoRaMacTest.h"
#define USE_SEMTECH_DEFAULT_CHANNEL_LINEUP 0
#if (USE_SEMTECH_DEFAULT_CHANNEL_LINEUP == 1)
#define LC4                                     \
	{                                           \
		867100000, 0, {((DR_5 << 4) | DR_0)}, 0 \
	}
#define LC5                                     \
	{                                           \
		867300000, 0, {((DR_5 << 4) | DR_0)}, 0 \
	}
#define LC6                                     \
	{                                           \
		867500000, 0, {((DR_5 << 4) | DR_0)}, 0 \
	}
#define LC7                                     \
	{                                           \
		867700000, 0, {((DR_5 << 4) | DR_0)}, 0 \
	}
#define LC8                                     \
	{                                           \
		867900000, 0, {((DR_5 << 4) | DR_0)}, 0 \
	}
#define LC9                                     \
	{                                           \
		868800000, 0, {((DR_7 << 4) | DR_7)}, 2 \
	}
#define LC10                                    \
	{                                           \
		868300000, 0, {((DR_6 << 4) | DR_6)}, 1 \
	}
#endif

#endif

LoraMacHelper::LoraMacHelper(SPIBase *spi)
{

    radio = new RadioHandler();
    radio->sxHandler = new SX126Handler();
    radio->sxHandler->SetSpiLora(spi);
    radio->sxHandler->sxDriver = new SX126xDriver();

#ifdef REGION_EU868
    radioRegion = new RegionEU868(radio);
#elif REGION_EU433
    radioRegion = new RegionEU433(radio);
#endif

    loraMac = new LoraMac(this, radio, radioRegion);

}

void LoraMacHelper::SetSpiLora(SPIBase *spi)
{
      radio->sxHandler->SetSpiLora(spi);
}

lmh_error_status LoraMacHelper::lmh_init(lmh_callback_t *callbacks, lmh_param_t lora_param, bool otaa,
                          eDeviceClass nodeClass, LoRaMacRegion_t user_region)
{
    region = (LoRaMacRegion_t)user_region;
    char strlog1[64];
    char strlog2[64];
    char strlog3[64];

    LoRaMacStatus_t error_status;
    m_param = lora_param;
    m_callbacks = callbacks;

    _otaa = otaa;

    _dutyCycleEnabled = m_param.duty_cycle;

#if (STATIC_DEVICE_EUI != 1)
    m_callbacks->BoardGetUniqueId(DevEui);
#endif

    if (_otaa)
    {
        sprintf(strlog2, "AppEui=%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X", AppEui[0], AppEui[1], AppEui[2], AppEui[3], AppEui[4], AppEui[5], AppEui[6], AppEui[7]);
        sprintf(strlog1, "DevEui=%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X", DevEui[0], DevEui[1], DevEui[2], DevEui[3], DevEui[4], DevEui[5], DevEui[6], DevEui[7]);
        sprintf(strlog3, "AppKey=%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X",
                AppKey[0], AppKey[1], AppKey[2], AppKey[3], AppKey[4], AppKey[5], AppKey[6], AppKey[7],
                AppKey[8], AppKey[9], AppKey[10], AppKey[11], AppKey[12], AppKey[13], AppKey[14], AppKey[15]);
        LOG_LIB("LMH", "OTAA \n%s\nDevAdd=%08X\n%s\n%s", strlog1, DevAddr, strlog2, strlog3);
    }
    else
    {
#if (STATIC_DEVICE_ADDRESS != 1)
        // Random seed initialization
        srand1(m_callbacks->BoardGetRandomSeed());
        // Choose a random device address
        DevAddr = randr(0, 0x01FFFFFF);
#endif
        sprintf(strlog1, "DevEui=%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X", DevEui[0], DevEui[1], DevEui[2], DevEui[3], DevEui[4], DevEui[5], DevEui[6], DevEui[7]);
        sprintf(strlog2, "NwkSKey=%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X",
                NwkSKey[0], NwkSKey[1], NwkSKey[2], NwkSKey[3], NwkSKey[4], NwkSKey[5], NwkSKey[6], NwkSKey[7],
                NwkSKey[8], NwkSKey[9], NwkSKey[10], NwkSKey[11], NwkSKey[12], NwkSKey[13], NwkSKey[14], NwkSKey[15]);
        sprintf(strlog3, "AppSKey=%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X",
                AppSKey[0], AppSKey[1], AppSKey[2], AppSKey[3], AppSKey[4], AppSKey[5], AppSKey[6], AppSKey[7],
                AppSKey[8], AppSKey[9], AppSKey[10], AppSKey[11], AppSKey[12], AppSKey[13], AppSKey[14], AppSKey[15]);
        LOG_LIB("LMH", "ABP \n%s\nDevAdd=%08X\n%s\n%s", strlog1, DevAddr, strlog2, strlog3);
    }

    //loraMac->MacMcpsConfirm(std::bind(&LoraMacHelper::McpsConfirm, this, std::placeholders::_1));
    //loraMac->MacMcpsIndication(std::bind(&LoraMacHelper::McpsIndication, this, std::placeholders::_1));
    //loraMac->MacMlmeConfirm(std::bind(&LoraMacHelper::MlmeConfirm, this, std::placeholders::_1));

    //LoRaMacPrimitives.MacMcpsConfirm = std::bind(&LoraMacHelper::McpsConfirm, this, std::placeholders::_1);
    //LoRaMacPrimitives.MacMcpsIndication = std::bind(&LoraMacHelper::McpsIndication, this, std::placeholders::_1);
    //LoRaMacPrimitives.MacMlmeConfirm = std::bind(&LoraMacHelper::MlmeConfirm, this, std::placeholders::_1);
    //LoRaMacCallbacks.GetBatteryLevel = m_callbacks->BoardGetBatteryLevel;

    error_status = loraMac->LoRaMacInitialization(&LoRaMacPrimitives, &LoRaMacCallbacks, region, nodeClass);
    std::cout << "result - LoraMacInitialization" << error_status << std::endl;
    if (error_status != LORAMAC_STATUS_OK)
    {
        return LMH_ERROR;
    }
    //std::cout << "HOLA";
    mibReq.Type = MIB_ADR;
    mibReq.Param.AdrEnable = lora_param.adr_enable;
    loraMac->LoRaMacMibSetRequestConfirm(&mibReq);

    mibReq.Type = MIB_CHANNELS_DEFAULT_DATARATE;
    mibReq.Param.ChannelsDefaultDatarate = lora_param.tx_data_rate;
    loraMac->LoRaMacMibSetRequestConfirm(&mibReq);

    mibReq.Type = MIB_CHANNELS_DATARATE;
    mibReq.Param.ChannelsDatarate = lora_param.tx_data_rate;
    loraMac->LoRaMacMibSetRequestConfirm(&mibReq);

    mibReq.Type = MIB_CHANNELS_TX_POWER;
    mibReq.Param.ChannelsTxPower = lora_param.tx_power;
    loraMac->LoRaMacMibSetRequestConfirm(&mibReq);

    mibReq.Type = MIB_PUBLIC_NETWORK;
    mibReq.Param.EnablePublicNetwork = lora_param.enable_public_network;
    loraMac->LoRaMacMibSetRequestConfirm(&mibReq);

    mibReq.Type = MIB_DEVICE_CLASS;
    mibReq.Param.Class = nodeClass;
    loraMac->LoRaMacMibSetRequestConfirm(&mibReq);

    loraMac->LoRaMacTestSetDutyCycleOn(_dutyCycleEnabled);
    if (region == LORAMAC_REGION_EU868)
    {
#if (USE_SEMTECH_DEFAULT_CHANNEL_LINEUP == 1)
        LoRaMacChannelAdd(3, (ChannelParams_t)LC4);
        LoRaMacChannelAdd(4, (ChannelParams_t)LC5);
        LoRaMacChannelAdd(5, (ChannelParams_t)LC6);
        LoRaMacChannelAdd(6, (ChannelParams_t)LC7);
        LoRaMacChannelAdd(7, (ChannelParams_t)LC8);
        LoRaMacChannelAdd(8, (ChannelParams_t)LC9);
        LoRaMacChannelAdd(9, (ChannelParams_t)LC10);

        mibReq.Type = MIB_RX2_DEFAULT_CHANNEL;
        mibReq.Param.Rx2DefaultChannel = (Rx2ChannelParams_t){869525000, DR_3};
        LoRaMacMibSetRequestConfirm(&mibReq);

        mibReq.Type = MIB_RX2_CHANNEL;
        mibReq.Param.Rx2Channel = (Rx2ChannelParams_t){869525000, DR_3};
        LoRaMacMibSetRequestConfirm(&mibReq);
#endif
    }

    // Set default SubBandChannels matching with RAK definitions
    switch (region)
    {
    case LORAMAC_REGION_AS923:
    case LORAMAC_REGION_AS923_2:
    case LORAMAC_REGION_AS923_3:
    case LORAMAC_REGION_AS923_4:
        lmh_setSubBandChannels(1);
        break;
    case LORAMAC_REGION_AU915:
        lmh_setSubBandChannels(2);
        break;
    case LORAMAC_REGION_CN470:
        lmh_setSubBandChannels(11);
        break;
    case LORAMAC_REGION_CN779:
        lmh_setSubBandChannels(1);
        break;
    case LORAMAC_REGION_EU433:
        lmh_setSubBandChannels(1);
        break;
    case LORAMAC_REGION_IN865:
        lmh_setSubBandChannels(1);
        break;
    case LORAMAC_REGION_EU868:
    default:
        lmh_setSubBandChannels(1);
        break;
    case LORAMAC_REGION_KR920:
        lmh_setSubBandChannels(1);
        break;
    case LORAMAC_REGION_US915:
        lmh_setSubBandChannels(2);
        break;
    case LORAMAC_REGION_RU864:
        lmh_setSubBandChannels(1);
        break;
    }

    // Make channel shifts for AS923-2, AS923-3 and AS923-4
    switch (region)
    {
    case LORAMAC_REGION_AS923:
        //lmh_setAS923Version(1);
        LOG_LIB("LMH", "Using AS923-1");
        break;
    case LORAMAC_REGION_AS923_2:
        //lmh_setAS923Version(2);
        LOG_LIB("LMH", "Using AS923-2");
        break;
    case LORAMAC_REGION_AS923_3:
        //lmh_setAS923Version(3);
        LOG_LIB("LMH", "Using AS923-3");
        break;
    case LORAMAC_REGION_AS923_4:
        //lmh_setAS923Version(4);
        LOG_LIB("LMH", "Using AS923-4");
        break;
    default:
        break;
    }
    return LMH_SUCCESS;
}

lmh_error_status LoraMacHelper::lmh_send(lmh_app_data_t *app_data, lmh_confirm is_tx_confirmed)
{
    McpsReq_t mcpsReq;
    LoRaMacTxInfo_t txInfo;

    /*if certification test are on going, application data is not sent*/
    if (m_compliance_test.running == true)
    {
        return LMH_ERROR;
    }

    if (lmh_mac_is_busy)
    {
        return LMH_BUSY;
    }

    if (loraMac->LoRaMacQueryTxPossible(app_data->buffsize, &txInfo) != LORAMAC_STATUS_OK)
    {
        LOG_LIB("LMH", "lmh_send -> LoRaMacQueryTxPossible failed");

        // Send empty frame in order to flush MAC commands
        mcpsReq.Type = MCPS_UNCONFIRMED;
        mcpsReq.Req.Unconfirmed.fBuffer = NULL;
        mcpsReq.Req.Unconfirmed.fBufferSize = 0;
        mcpsReq.Req.Unconfirmed.Datarate = m_param.tx_data_rate;
        // cannot send
        return LMH_ERROR;
    }
    else
    {
        if (is_tx_confirmed == LMH_UNCONFIRMED_MSG)
        {
            mcpsReq.Type = MCPS_UNCONFIRMED;
            mcpsReq.Req.Unconfirmed.fPort = app_data->port;
            mcpsReq.Req.Unconfirmed.fBufferSize = app_data->buffsize;
            mcpsReq.Req.Unconfirmed.fBuffer = app_data->buffer;
            mcpsReq.Req.Unconfirmed.Datarate = m_param.tx_data_rate;
        }
        else
        {
            mcpsReq.Type = MCPS_CONFIRMED;
            mcpsReq.Req.Confirmed.fPort = app_data->port;
            mcpsReq.Req.Confirmed.fBufferSize = app_data->buffsize;
            mcpsReq.Req.Confirmed.fBuffer = app_data->buffer;
            /// \todo make nbTrials configurable
            // if ((region == LORAMAC_REGION_AS923) ||
            // 	(region == LORAMAC_REGION_AS923_2) ||
            // 	(region == LORAMAC_REGION_AS923_3) ||
            // 	(region == LORAMAC_REGION_AS923_4))
            if (region == LORAMAC_REGION_AS923)
            {
                mcpsReq.Req.Confirmed.NbTrials = 1; //8;
            }
            else
            {
                mcpsReq.Req.Confirmed.NbTrials = 8;
            }
            mcpsReq.Req.Confirmed.Datarate = m_param.tx_data_rate;
        }

        if (loraMac->LoRaMacMcpsRequest(&mcpsReq) == LORAMAC_STATUS_OK)
        {
            lmh_mac_is_busy = true;
            return LMH_SUCCESS;
        }
        printf("LMH lmh_send -> LoRaMacMcpsRequest failed");
    }

    return LMH_ERROR;
}

lmh_error_status LoraMacHelper::lmh_send_blocking(lmh_app_data_t *app_data, lmh_confirm is_tx_confirmed, uint32_t time_out)
{
    if (lmh_send(app_data, is_tx_confirmed) == LMH_SUCCESS)
    {
        //uint32_t time_start = millis();
        uint64_t time_start = Timestamp();
        while (lmh_mac_is_busy)
        {
            if ((Timestamp() - time_start) > time_out)
            {
                //LOG_LIB("LMH", "timeout at %ld", (millis() - time_start));
                lmh_mac_is_busy = false;
                return LMH_ERROR;
            }
            //delay(250);
            sleep_microseconds(250);
        }
        return LMH_SUCCESS;
    }
    LOG_LIB("LMH", "lmh_send returned LMH_ERROR");
    return LMH_ERROR;
}

void LoraMacHelper::lmh_join(void)
{
    MlmeReq_t mlmeReq;

    mlmeReq.Type = MLME_JOIN;
    mlmeReq.Req.Join.DevEui = DevEui;
    mlmeReq.Req.Join.AppEui = AppEui;
    mlmeReq.Req.Join.AppKey = AppKey;
    mlmeReq.Req.Join.NbTrials = m_param.nb_trials;

    JoinParameters = mlmeReq.Req.Join;

    if (_otaa)
    {
        loraMac->LoRaMacMlmeRequest(&mlmeReq);
    }
    else
    {
        mibReq.Type = MIB_NET_ID;
        mibReq.Param.NetID = LORAWAN_NETWORK_ID;
        loraMac->LoRaMacMibSetRequestConfirm(&mibReq);

        mibReq.Type = MIB_DEV_ADDR;
        mibReq.Param.DevAddr = DevAddr;
        loraMac->LoRaMacMibSetRequestConfirm(&mibReq);

        mibReq.Type = MIB_NWK_SKEY;
        mibReq.Param.NwkSKey = NwkSKey;
        loraMac->LoRaMacMibSetRequestConfirm(&mibReq);

        mibReq.Type = MIB_APP_SKEY;
        mibReq.Param.AppSKey = AppSKey;
        loraMac->LoRaMacMibSetRequestConfirm(&mibReq);

        mibReq.Type = MIB_NETWORK_JOINED;
        mibReq.Param.IsNetworkJoined = JOIN_OK;
        loraMac->LoRaMacMibSetRequestConfirm(&mibReq);

        m_callbacks->lmh_has_joined();
    }
}

lmh_join_status LoraMacHelper::lmh_join_status_get(void)
{
    MibRequestConfirm_t mibReq;
    mibReq.Type = MIB_NETWORK_JOINED;
    loraMac->LoRaMacMibGetRequestConfirm(&mibReq);

    // if (mibReq.Param.IsNetworkJoined == true)
    // {
    // 	return LMH_SET;
    // }
    // else
    // {
    // 	return LMH_RESET;
    // }

    return (lmh_join_status)mibReq.Param.IsNetworkJoined;
}

uint32_t LoraMacHelper::lmh_getDevAddr(void)
{
    return loraMac->LoRaMacGetOTAADevId();
}

lmh_error_status LoraMacHelper::lmh_class_request(DeviceClass_t newClass)
{
    lmh_error_status Errorstatus = LMH_SUCCESS;
    MibRequestConfirm_t mibReq;
    DeviceClass_t currentClass;

    mibReq.Type = MIB_DEVICE_CLASS;
    loraMac->LoRaMacMibGetRequestConfirm(&mibReq);
    currentClass = mibReq.Param.Class;

    // attempt to swicth only if class update
    if (currentClass != newClass)
    {
        switch (newClass)
        {
        case CLASS_A:
        {
            if (currentClass != CLASS_A)
            {
                mibReq.Param.Class = CLASS_A;
                if (loraMac->LoRaMacMibSetRequestConfirm(&mibReq) == LORAMAC_STATUS_OK)
                {
                    // switch is instantanuous
                    m_callbacks->lmh_ConfirmClass(CLASS_A);
                }
                else
                {
                    Errorstatus = LMH_ERROR;
                }
            }
            break;
        }

        case CLASS_B:
        {
            if (currentClass != CLASS_B)
            {
                mibReq.Param.Class = CLASS_B;
                if (loraMac->LoRaMacMibSetRequestConfirm(&mibReq) == LORAMAC_STATUS_OK)
                {
                    // switch is instantanuous
                    m_callbacks->lmh_ConfirmClass(CLASS_B);
                }
                else
                {
                    Errorstatus = LMH_ERROR;
                }
            }
            break;
        }

        case CLASS_C:
        {
            if (currentClass != CLASS_A)
            {
                Errorstatus = LMH_ERROR;
            }
            // switch is instantanuous
            mibReq.Param.Class = CLASS_C;
            if (loraMac->LoRaMacMibSetRequestConfirm(&mibReq) == LORAMAC_STATUS_OK)
            {
                m_callbacks->lmh_ConfirmClass(CLASS_C);
            }
            else
            {
                Errorstatus = LMH_ERROR;
            }
            break;
        }

        default:
            Errorstatus = LMH_ERROR;
            break;
        }
    }

    return Errorstatus;
}

void LoraMacHelper::lmh_class_get(DeviceClass_t *currentClass)
{
    MibRequestConfirm_t mibReq;

    mibReq.Type = MIB_DEVICE_CLASS;
    loraMac->LoRaMacMibGetRequestConfirm(&mibReq);

    *currentClass = mibReq.Param.Class;
}

void LoraMacHelper::lmh_datarate_set(uint8_t data_rate, bool enable_adr)
{
    m_param.adr_enable = enable_adr;
    m_param.tx_data_rate = data_rate;

    mibReq.Type = MIB_ADR;
    mibReq.Param.AdrEnable = enable_adr;
    loraMac->LoRaMacMibSetRequestConfirm(&mibReq);

    mibReq.Type = MIB_CHANNELS_DEFAULT_DATARATE;
    mibReq.Param.ChannelsDefaultDatarate = data_rate;
    loraMac->LoRaMacMibSetRequestConfirm(&mibReq);

    mibReq.Type = MIB_CHANNELS_DATARATE;
    mibReq.Param.ChannelsDefaultDatarate = data_rate;
    loraMac->LoRaMacMibSetRequestConfirm(&mibReq);
}

void LoraMacHelper::lmh_tx_power_set(uint8_t tx_power)
{
    mibReq.Type = MIB_CHANNELS_TX_POWER;
    mibReq.Param.ChannelsTxPower = tx_power;
    loraMac->LoRaMacMibSetRequestConfirm(&mibReq);
}

void LoraMacHelper::lmh_setDevEui(uint8_t userDevEui[])
{
	memcpy(DevEui, userDevEui, 8);
}

void LoraMacHelper::lmh_setAppEui(uint8_t *userAppEui)
{
	memcpy(AppEui, userAppEui, 8);
}

void LoraMacHelper::lmh_setAppKey(uint8_t *userAppKey)
{
	memcpy(AppKey, userAppKey, 16);
}

void LoraMacHelper::lmh_setNwkSKey(uint8_t *userNwkSKey)
{
	memcpy(NwkSKey, userNwkSKey, 16);
}

void LoraMacHelper::lmh_setAppSKey(uint8_t *userAppSKey)
{
	memcpy(AppSKey, userAppSKey, 16);
}

void LoraMacHelper::lmh_setDevAddr(uint32_t userDevAddr)
{
	DevAddr = userDevAddr;
}

bool LoraMacHelper::lmh_setSubBandChannels(uint8_t subBand)
{
    uint16_t subBandChannelMask[6] = {0, 0, 0, 0, 0, 0};
    uint8_t maxBand = 1;
    switch (region)
    {
    case LORAMAC_REGION_AS923:
    case LORAMAC_REGION_AS923_2:
    case LORAMAC_REGION_AS923_3:
    case LORAMAC_REGION_AS923_4:
        maxBand = 1;
        break;
    case LORAMAC_REGION_AU915:
        maxBand = 9;
        break;
    case LORAMAC_REGION_CN470:

        maxBand = 12;
        break;
    case LORAMAC_REGION_CN779:
        maxBand = 2;
        break;
    case LORAMAC_REGION_EU433:
        maxBand = 2;
        break;
    case LORAMAC_REGION_IN865:
        maxBand = 2;
        break;
    case LORAMAC_REGION_EU868:
        maxBand = 2;
        break;
    case LORAMAC_REGION_KR920:
        maxBand = 2;
        break;
    case LORAMAC_REGION_US915:
        maxBand = 9;
        break;
    case LORAMAC_REGION_RU864:
        maxBand = 1;
        break;
    }
    uint16_t upperMask = 0xFF00;
    uint16_t lowerMask = 0x00FF;

    // Check for valid sub band
    if ((subBand == 0) || (subBand > maxBand))
    {
        LOG_LIB("LMH", "Invalid subband");

        // Invalid sub band requested
        return false;
    }

    switch (subBand)
    {
    case 1:
        subBandChannelMask[0] = lowerMask;
        break;
    case 2:
        if (maxBand >= 2)
        {
            subBandChannelMask[0] = upperMask;
        }
        break;
    case 3:
        if (maxBand >= 3)
        {
            subBandChannelMask[1] = lowerMask;
        }
        break;
    case 4:
        if (maxBand >= 4)
        {
            subBandChannelMask[1] = upperMask;
        }
        break;
    case 5:
        if (maxBand >= 5)
        {
            subBandChannelMask[2] = lowerMask;
        }
        break;
    case 6:
        if (maxBand >= 6)
        {
            subBandChannelMask[2] = upperMask;
        }
        break;
    case 7:
        if (maxBand >= 7)
        {
            subBandChannelMask[3] = lowerMask;
        }
        break;
    case 8:
        if (maxBand >= 8)
        {
            subBandChannelMask[3] = upperMask;
        }
        break;
    case 9:
        if (maxBand >= 9)
        {
            subBandChannelMask[4] = lowerMask;
        }
        break;
    case 10:
        if (maxBand >= 10)
        {
            subBandChannelMask[4] = upperMask;
        }
        break;
    case 11:
        if (maxBand >= 11)
        {
            subBandChannelMask[5] = lowerMask;
        }
        break;
    case 12:
        if (maxBand >= 12)
        {
            subBandChannelMask[5] = upperMask;
        }
        break;
    default:
        LOG_LIB("LMH", "Invalid subband");

        return false;
    }
    if (maxBand > 2)
    {
        RegionCommonChanMaskCopy(ChannelsDefaultMask, subBandChannelMask, 6);
        RegionCommonChanMaskCopy(ChannelsMask, subBandChannelMask, 6);
        RegionCommonChanMaskCopy(ChannelsMaskRemaining, subBandChannelMask, 6);

        RegionCommonChanMaskCopy(radioRegion->ChannelsMask, ChannelsMask, 6);
        RegionCommonChanMaskCopy(radioRegion->ChannelsDefaultMask, ChannelsMask, 6);
        RegionCommonChanMaskCopy(radioRegion->ChannelsMaskRemaining, ChannelsMask, 6);
    }
    else
    {
        RegionCommonChanMaskCopy(ChannelsDefaultMask, subBandChannelMask, 1);
        RegionCommonChanMaskCopy(ChannelsMask, subBandChannelMask, 1);
        RegionCommonChanMaskCopy(ChannelsMaskRemaining, subBandChannelMask, 1);

        RegionCommonChanMaskCopy(radioRegion->ChannelsDefaultMask, ChannelsMask, 1);
        RegionCommonChanMaskCopy(radioRegion->ChannelsMask, ChannelsMask, 1);
        RegionCommonChanMaskCopy(radioRegion->ChannelsMask, ChannelsMask, 1);
    }

    LOG_LIB("LMH", "Selected subband %d", subBand);
    return true;
}

void LoraMacHelper::lmh_setSingleChannelGateway(uint8_t userSingleChannel, int8_t userDatarate)
{
	singleChannelGateway = true;
	singleChannelSelected = userSingleChannel;
	singleChannelDatarate = userDatarate;

    loraMac->singleChannelGateway = singleChannelGateway;
    loraMac->singleChannelSelected = singleChannelSelected;
    loraMac->singleChannelDatarate = singleChannelDatarate;
}

bool LoraMacHelper::lmh_setConfRetries(uint8_t retries)
{
    if ((retries > 0) && (retries < 8))
    {
        max_ack_retries = retries;
        return true;
    }
    return false;
}

//static bool compliance_test_tx(void)
bool LoraMacHelper::compliance_test_tx(void)
{
	McpsReq_t mcpsReq;
	LoRaMacTxInfo_t txInfo;

	if (m_compliance_test.link_check == true)
	{
		m_compliance_test.link_check = false;
		m_compliance_test.data_buffer_size = 3;
		m_compliance_test.data_buffer[0] = 5;
		m_compliance_test.data_buffer[1] = m_compliance_test.demod_margin;
		m_compliance_test.data_buffer[2] = m_compliance_test.nb_gateways;
		m_compliance_test.state = 1;
	}
	else
	{
		switch (m_compliance_test.state)
		{
		case 4:
			m_compliance_test.state = 1;
			break;

		case 1:
			m_compliance_test.data_buffer_size = 2;
			m_compliance_test.data_buffer[0] = m_compliance_test.downlink_counter >> 8;
			m_compliance_test.data_buffer[1] = m_compliance_test.downlink_counter;
			break;
		}
	}

    if (loraMac->LoRaMacQueryTxPossible(m_compliance_test.data_buffer_size, &txInfo) != LORAMAC_STATUS_OK)
	{
		// Send empty frame in order to flush MAC commands
		mcpsReq.Type = MCPS_UNCONFIRMED;
		mcpsReq.Req.Unconfirmed.fBuffer = NULL;
		mcpsReq.Req.Unconfirmed.fBufferSize = 0;
		mcpsReq.Req.Unconfirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
	}
	else
	{
		if (m_compliance_test.is_tx_confirmed == LMH_UNCONFIRMED_MSG)
		{
			mcpsReq.Type = MCPS_UNCONFIRMED;
			mcpsReq.Req.Unconfirmed.fPort = LORAWAN_CERTIF_PORT;
			mcpsReq.Req.Unconfirmed.fBufferSize = m_compliance_test.data_buffer_size;
			mcpsReq.Req.Unconfirmed.fBuffer = &(m_compliance_test.data_buffer);
			mcpsReq.Req.Unconfirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
		}
		else
		{
			mcpsReq.Type = MCPS_CONFIRMED;
			mcpsReq.Req.Confirmed.fPort = LORAWAN_CERTIF_PORT;
			mcpsReq.Req.Confirmed.fBufferSize = m_compliance_test.data_buffer_size;
			mcpsReq.Req.Confirmed.fBuffer = &(m_compliance_test.data_buffer);
			/// \todo make nbTrials configurable
			mcpsReq.Req.Confirmed.NbTrials = 8;
			mcpsReq.Req.Confirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
		}
	}

	// certification test on-going
	TimerStart(&ComplianceTestTxNextPacketTimer);

    if (loraMac->LoRaMacMcpsRequest(&mcpsReq) == LORAMAC_STATUS_OK)
	{
		return false;
	}

	return true;
}

/**@brief Function executed on TxNextPacket Timeout event
 */
void LoraMacHelper::OnComplianceTestTxNextPacketTimerEvent(void)
{
	compliance_test_tx();
}

#define LORAMAC_TX_RUNNING 0x00000001
/**@brief MCPS-Confirm event function
 *
 * @param mcpsConfirm Pointer to the confirm structure, containing confirm attributes.
 */
void LoraMacHelper::McpsConfirm(McpsConfirm_t *mcpsConfirm)
{
	if (mcpsConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK)
	{
		LoRaMacState &= ~LORAMAC_TX_RUNNING;
		switch (mcpsConfirm->McpsRequest)
		{
		case MCPS_UNCONFIRMED:
		{
			// Check Datarate
			// Check TxPower
			// Report unconfirmed TX finished
			if (m_callbacks->lmh_unconf_finished != 0)
			{
				m_callbacks->lmh_unconf_finished();
			}
			break;
		}

		case MCPS_CONFIRMED:
		{
			// Check Datarate
			// Check TxPower
			// Check NbTrials

			// Report confirmed TX finished with result
			if (m_callbacks->lmh_conf_result != 0)
			{
				m_callbacks->lmh_conf_result(mcpsConfirm->AckReceived);
			}
			break;
		}

		case MCPS_PROPRIETARY:
		{
			break;
		}

		default:
			break;
		}
	}
	if ((mcpsConfirm->Status == LORAMAC_EVENT_INFO_STATUS_RX1_TIMEOUT) || (mcpsConfirm->Status == LORAMAC_EVENT_INFO_STATUS_RX2_TIMEOUT))
	{
		LoRaMacState &= ~LORAMAC_TX_RUNNING;
		switch (mcpsConfirm->McpsRequest)
		{
		case MCPS_UNCONFIRMED:
		{
			// Check Datarate
			// Check TxPower
			// Report unconfirmed TX finished
			LOG_LIB("LMH", "Timeout TX + RX finished");
			if (m_callbacks->lmh_unconf_finished != 0)
			{
				m_callbacks->lmh_unconf_finished();
			}
			break;
		}

		case MCPS_CONFIRMED:
		{
			// Check Datarate
			// Check TxPower
			// Check NbTrials

			// Report confirmed TX finished with result
			LOG_LIB("LMH", "Timeout Conf TX finished %s", mcpsConfirm->AckReceived ? "SUCC" : "FAIL");
			if (m_callbacks->lmh_conf_result != 0)
			{
				m_callbacks->lmh_conf_result(mcpsConfirm->AckReceived);
			}
			break;
		}

		default:
			break;
		}
	}
}

/**@brief MCPS-Indication event function
 *
 * @param mcpsIndication	Pointer to the indication structure, containing indication attributes.
 */
void LoraMacHelper::McpsIndication(McpsIndication_t *mcpsIndication)
{
	lmh_app_data_t app_data;

	if (mcpsIndication->Status != LORAMAC_EVENT_INFO_STATUS_OK)
	{
		return;
	}

	switch (mcpsIndication->McpsIndication)
	{
	case MCPS_UNCONFIRMED:
	{
		break;
	}

	case MCPS_CONFIRMED:
	{
		break;
	}

	case MCPS_PROPRIETARY:
	{
		break;
	}

	case MCPS_MULTICAST:
	{
		break;
	}

	default:
		break;
	}

	// Check Multicast
	// Check Port
	// Check Datarate
	// Check FramePending
	// Check Buffer
	// Check BufferSize
	// Check Rssi
	// Check Snr
	// Check RxSlot
	if (m_compliance_test.running == true)
	{
		m_compliance_test.downlink_counter++;
	}

	if (mcpsIndication->RxData == true)
	{
		switch (mcpsIndication->Port)
		{
		case LORAWAN_CERTIF_PORT:
			// Compliance not started yet, start it
			if (m_compliance_test.running == false)
			{
				// Check compliance test enable command (i)
				if ((mcpsIndication->BufferSize == 4) &&
					(mcpsIndication->Buffer[0] == 0x01) &&
					(mcpsIndication->Buffer[1] == 0x01) &&
					(mcpsIndication->Buffer[2] == 0x01) &&
					(mcpsIndication->Buffer[3] == 0x01))
				{
					m_compliance_test.is_tx_confirmed = LMH_UNCONFIRMED_MSG;
					m_compliance_test.data_buffer_size = 2;
					m_compliance_test.downlink_counter = 0;
					m_compliance_test.link_check = false;
					m_compliance_test.demod_margin = 0;
					m_compliance_test.nb_gateways = 0;
					m_compliance_test.running = true;
					m_compliance_test.state = 1;

					MibRequestConfirm_t mibReq;
					mibReq.Type = MIB_ADR;
                    loraMac->LoRaMacMibGetRequestConfirm(&mibReq);
					m_adr_enable_init = mibReq.Param.AdrEnable;

					mibReq.Type = MIB_ADR;
					mibReq.Param.AdrEnable = true;
                    loraMac->LoRaMacMibSetRequestConfirm(&mibReq);

                    loraMac->LoRaMacTestSetDutyCycleOn(false);

                    TimerInit(&ComplianceTestTxNextPacketTimer, std::bind(&LoraMacHelper::OnComplianceTestTxNextPacketTimerEvent, this));
					TimerSetValue(&ComplianceTestTxNextPacketTimer, 5000);

					// confirm test mode activation
					compliance_test_tx();
				}
			}
			// Compliance is started, check which stage we are at and take action
			else
			{
				m_compliance_test.state = mcpsIndication->Buffer[0];
				switch (m_compliance_test.state)
				{
				case 0: // Check compliance test disable command (ii)
				{
					m_compliance_test.downlink_counter = 0;
					m_compliance_test.running = false;

					MibRequestConfirm_t mibReq;
					mibReq.Type = MIB_ADR;
					mibReq.Param.AdrEnable = m_adr_enable_init;
                    loraMac->LoRaMacMibSetRequestConfirm(&mibReq);

                    loraMac->LoRaMacTestSetDutyCycleOn(_dutyCycleEnabled);

					break;
				}

				case 1: // (iii, iv)
				{
					m_compliance_test.data_buffer_size = 2;
					break;
				}

				case 2: // Enable confirmed messages (v)
				{
					m_compliance_test.is_tx_confirmed = LMH_CONFIRMED_MSG;
					m_compliance_test.state = 1;
					break;
				}

				case 3: // Disable confirmed messages (vi)
				{
					m_compliance_test.is_tx_confirmed = LMH_UNCONFIRMED_MSG;
					m_compliance_test.state = 1;
					break;
				}

				case 4: // (vii)
				{
					m_compliance_test.data_buffer_size = mcpsIndication->BufferSize;
					m_compliance_test.data_buffer[0] = 4;
					for (uint8_t i = 1; i < T_MIN(m_compliance_test.data_buffer_size, LORAWAN_APP_DATA_MAX_SIZE); i++)
					{
						m_compliance_test.data_buffer[i] = mcpsIndication->Buffer[i] + 1;
					}
					break;
				}

				case 5: // (viii)
				{
					MlmeReq_t mlmeReq;
					mlmeReq.Type = MLME_LINK_CHECK;
                    loraMac->LoRaMacMlmeRequest(&mlmeReq);
					break;
				}

				case 6: // (ix)
				{
					MlmeReq_t mlmeReq;

					// Disable TestMode and revert back to normal operation
					m_compliance_test.is_tx_confirmed = LORAWAN_CONFIRMED_MSG_ON;
					m_compliance_test.downlink_counter = 0;
					m_compliance_test.running = false;

					MibRequestConfirm_t mibReq;
					mibReq.Type = MIB_ADR;
					mibReq.Param.AdrEnable = m_adr_enable_init;
                    loraMac->LoRaMacMibSetRequestConfirm(&mibReq);

                    loraMac->LoRaMacTestSetDutyCycleOn(_dutyCycleEnabled);

					mlmeReq.Type = MLME_JOIN;
					mlmeReq.Req.Join = JoinParameters;
                    loraMac->LoRaMacMlmeRequest(&mlmeReq);
					break;
				}

				case 7: // (x)
				{
					if (mcpsIndication->BufferSize == 3)
					{
						MlmeReq_t mlmeReq;
						mlmeReq.Type = MLME_TXCW;
						mlmeReq.Req.TxCw.Timeout = (uint16_t)((mcpsIndication->Buffer[1] << 8) | mcpsIndication->Buffer[2]);
                        loraMac->LoRaMacMlmeRequest(&mlmeReq);
					}
					else if (mcpsIndication->BufferSize == 7)
					{
						MlmeReq_t mlmeReq;
						mlmeReq.Type = MLME_TXCW_1;
						mlmeReq.Req.TxCw.Timeout = (uint16_t)((mcpsIndication->Buffer[1] << 8) | mcpsIndication->Buffer[2]);
						mlmeReq.Req.TxCw.Frequency = (uint32_t)((mcpsIndication->Buffer[3] << 16) | (mcpsIndication->Buffer[4] << 8) | mcpsIndication->Buffer[5]) * 100;
						mlmeReq.Req.TxCw.Power = mcpsIndication->Buffer[6];
                        loraMac->LoRaMacMlmeRequest(&mlmeReq);
					}
					m_compliance_test.state = 1;
					break;
				}

				default:
					break;
				}
			}

			if (m_compliance_test.running == false)
			{
				// cerification test stops
				TimerStop(&ComplianceTestTxNextPacketTimer);
			}
			break;

		default:
			app_data.port = mcpsIndication->Port;
			app_data.buffsize = mcpsIndication->BufferSize;
			mcpsIndication->Buffer[app_data.buffsize] = 0;
			app_data.buffer = mcpsIndication->Buffer;
			app_data.rssi = mcpsIndication->Rssi;
			app_data.snr = mcpsIndication->Snr;
			m_callbacks->lmh_RxData(&app_data);
			break;
		}
	}
}

/**@brief MLME-Confirm event function
 *
 * @param mlmeConfirm	Pointer to the confirm structure, containing confirm attributes.
 */
void LoraMacHelper::MlmeConfirm(MlmeConfirm_t *mlmeConfirm)
{
	switch (mlmeConfirm->MlmeRequest)
	{
	case MLME_JOIN:
	{
		if (mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK)
		{
			// Status is OK, node has joined the network
			if (m_callbacks->lmh_has_joined != NULL)
			{
				m_callbacks->lmh_has_joined();
			}
		}
		else
		{
			// call joined failed callback here
			if (m_callbacks->lmh_has_joined_failed != NULL)
			{
				m_callbacks->lmh_has_joined_failed();
			}

			// Join was not successful. Try to join again
			// lmh_join(); // people can call it in callback funtion of joined failed.
		}
		break;
	}

	case MLME_LINK_CHECK:
	{
		if (mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK)
		{
			// Check DemodMargin
			// Check NbGateways
			if (m_compliance_test.running == true)
			{
				m_compliance_test.link_check = true;
				m_compliance_test.demod_margin = mlmeConfirm->DemodMargin;
				m_compliance_test.nb_gateways = mlmeConfirm->NbGateways;
			}
		}
		break;
	}

	default:
		break;
	}
}

/*bool lmh_setAS923Version(uint8_t version)
{
	return RegionAS923SetVersion(version);
}*/


