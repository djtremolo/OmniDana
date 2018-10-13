#ifndef __OMNIDANACOMMON_H__
#define __OMNIDANACOMMON_H__

#include <Arduino_FreeRTOS.h>
#include "message_buffer.h"


typedef unsigned int size_t;

typedef unsigned int uint16_t;
typedef unsigned char uint8_t;

void DumpMessage(uint8_t* prefix, uint8_t* postfix, uint8_t* buf, size_t len);

#endif//__OMNIDANACOMMON_H__
