#ifndef __CRC16_H__
#define __CRC16_H__

uint16_t crc16_update(uint16_t crc, uint8_t a);
uint16_t crc16_calculate(uint8_t *buf, uint16_t len);

#endif//__CRC16_H__

