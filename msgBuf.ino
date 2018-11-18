#include "common.h"
#include "msgBuf.h"

uint8_t msgLen(uint8_t *start, uint8_t *currentPosition)
{
  size_t a = (size_t)start;
  size_t b = (size_t)currentPosition;

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

void msgPutU16InvertedOrder(uint8_t **buf, uint16_t val)
{
  if(buf && *buf)
  {
    uint8_t *b = *buf;

    *(b++) = ((val >> 8) & 0xFF);
    *(b++) = (val & 0xFF);

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

void msgPutTimeDate(uint8_t **buf, time_t t)
{
  tmElements_t tm;
  breakTime(t, tm);

  msgPutU8(buf, tmYearToY2k(tm.Year));
  msgPutU8(buf, tm.Month);
  msgPutU8(buf, tm.Day);
  msgPutU8(buf, tm.Hour);
  msgPutU8(buf, tm.Minute);
  msgPutU8(buf, tm.Second);
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

uint16_t msgGetU16InvertedOrder(uint8_t **buf)
{
  uint16_t ret = 0;
  if(buf && *buf)
  {
    uint8_t *b = *buf;

    ret |= (*(b++)) << 8;
    ret |= *(b++);

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

time_t msgGetTimeDate(uint8_t **buf)
{
  tmElements_t tm;

  tm.Year = y2kYearToTm(msgGetU8(buf));
  tm.Month = msgGetU8(buf);
  tm.Day = msgGetU8(buf);
  tm.Hour = msgGetU8(buf);
  tm.Minute = msgGetU8(buf);
  tm.Second = msgGetU8(buf);

  return makeTime(tm);
}

