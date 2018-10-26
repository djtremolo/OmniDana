#ifndef __OMNIDANACOMMON_H__
#define __OMNIDANACOMMON_H__

#include <Arduino_FreeRTOS.h>
#include "message_buffer.h"
#include "ioInterface.h"

/*MACROS*/
#define REC_TO_COMM_BUFFER_SIZE   31    /*incoming messages*/
#define COMM_TO_CTRL_BUFFER_SIZE  21    /*treatment requests*/
#define COMM_TO_REC_BUFFER_SIZE   21    /*response messages*/
#define FB_TO_CTRL_BUFFER_SIZE    21    /*feedback events*/

/*TASK PRIORITIES*/
#define UART_TASK_PRIORITY         7
#define COMM_TASK_PRIORITY        8
#define CTRL_TASK_PRIORITY        9
#define FB_TASK_PRIORITY          10


typedef unsigned int size_t;
typedef unsigned int uint16_t;
typedef unsigned char uint8_t;

typedef struct
{
  MessageBufferHandle_t commToRecBuffer;
  MessageBufferHandle_t recToCommBuffer;
  MessageBufferHandle_t commToCtrlBuffer;
  MessageBufferHandle_t fbToCtrlBuffer;
  boolean fbPdmIsBusy;
  buttonKey_t ctrlActiveButton;
} OmniDanaContext_t;


#endif//__OMNIDANACOMMON_H__
