#include "common.h"
#include "msgBuf.h"

uint8_t msgLen(uint8_t *start, uint8_t *currentPosition)
{
  size_t a = (size_t)start;
  size_t b = (size_t)currentPosition;

  uint8_t len = (a < b) ? (uint8_t)(b - a) : 0; 
  return (uint8_t)(b - a);
}

void msgPutU16(uint8_t **buf, uint16_t val)
{
  if(buf && *buf)
  {
    uint8_t *b = *buf;

    *(b++) = (val & 0xFF);
    *(b++) = ((val >> 8) & 0xFF);

    *buf = b;
  }
}

void msgPutU8(uint8_t **buf, uint8_t val)
{
  if(buf && *buf)
  {
    uint8_t *b = *buf;

    *(b++) = (val & 0xFF);

    *buf = b;
  }
}

void msgPutU32(uint8_t **buf, uint32_t val)
{
  if(buf && *buf)
  {
    uint8_t *b = *buf;

    *(b++) = (val & 0xFF);
    *(b++) = ((val >> 8) & 0xFF);
    *(b++) = ((val >> 16) & 0xFF);
    *(b++) = ((val >> 24) & 0xFF);

    *buf = b;
  }
}

uint16_t msgGetU16(uint8_t **buf)
{
  uint16_t ret = 0;
  if(buf && *buf)
  {
    uint8_t *b = *buf;

    ret |= *(b++);
    ret |= (*(b++)) << 8;

    *buf = b;
  }
  return ret;
}

uint8_t msgGetU8(uint8_t **buf)
{
  uint8_t ret = 0;
  if(buf && *buf)
  {
    uint8_t *b = *buf;

    ret |= *(b++);

    *buf = b;
  }
  return ret;
}

uint32_t msgGetU32(uint8_t **buf)
{
  uint32_t ret = 0;
  if(buf && *buf)
  {
    uint8_t *b = *buf;

    ret |= *(b++);
    ret |= (*(b++)) << 8;
    ret |= (*(b++)) << 16;
    ret |= (*(b++)) << 24;

    *buf = b;
  }
  return ret;
}
