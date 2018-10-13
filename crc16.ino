/*
CRC16 code copied from: http://www.nongnu.org/avr-libc/user-manual/group__util__crc.html
*/
#include "OmniDanaCommon.h"
#include "crc16.h"

static uint16_t crc16_update(uint16_t crc, uint8_t a)
{
    int i;

    crc ^= a;
    for (i = 0; i < 8; ++i)
    {
        if (crc & 1)
            crc = (crc >> 1) ^ 0xA001;
        else
            crc = (crc >> 1);
    }
    return crc;
}

uint16_t crc16_calculate(uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0;

    while(len--)
    {
        crc = crc16_update(crc, *buf++);
    }

    return crc;
}

