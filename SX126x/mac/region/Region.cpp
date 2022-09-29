/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech
 ___ _____ _   ___ _  _____ ___  ___  ___ ___
/ __|_   _/_\ / __| |/ / __/ _ \| _ \/ __| __|
\__ \ | |/ _ \ (__| ' <| _| (_) |   / (__| _|
|___/ |_/_/ \_\___|_|\_\_| \___/|_|_\\___|___|
embedded.connectivity.solutions===============

Description: LoRa MAC region implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis ( Semtech ), Gregory Cristian ( Semtech ) and Daniel Jaeckle ( STACKFORCE )
*/
#include <stdbool.h>
#include <string.h>
#include <stdint.h>
#include "utilities.h"

#include "boards/mcu/timer.h"
#include "mac/LoRaMac.h"

// Regional includes
#include "Region.h"

RegionBase::RegionBase()
{

}

int8_t RegionBase::GetNextLowerTxDr(int8_t dr, int8_t minDr)
{
    uint8_t nextLowerDr = 0;

    if (dr == minDr)
    {
        nextLowerDr = minDr;
    }
    else
    {
        nextLowerDr = dr - 1;
    }
    return nextLowerDr;
}

int8_t RegionBase::LimitTxPower(int8_t txPower, int8_t maxBandTxPower, int8_t datarate, uint16_t *channelsMask)
{
    int8_t txPowerResult = txPower;

    // Limit tx power to the band max
    txPowerResult = T_MAX(txPower, maxBandTxPower);

    return txPowerResult;
}
