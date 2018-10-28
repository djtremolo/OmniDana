#ifndef __OMNIDANACOMMON_H__
#define __OMNIDANACOMMON_H__

#include <Arduino_FreeRTOS.h>
#include "message_buffer.h"
#include "ioInterface.h"


/*TASK PRIORITIES*/
#define UART_TASK_PRIORITY            2
#define CTRL_TASK_PRIORITY            4
#define FB_TASK_PRIORITY              5

/*DANARS MESSAGE SIZES*/
#define DANA_MAX_PAYLOAD_LENGTH       20
#define DANA_HEADER_LEN               5
#define DANA_FOOTER_LEN               4
#define DANA_HEADER_BYTES_AFTER_LEN   2
#define DANA_MAX_BUF_LEN              (DANA_HEADER_LEN + DANA_MAX_PAYLOAD_LENGTH + DANA_FOOTER_LEN)
#define DANA_MAX_LEN_FIELD            (DANA_HEADER_BYTES_AFTER_LEN + DANA_MAX_PAYLOAD_LENGTH)
#define DANA_RAW_MSG_LEN(_plLen)      (DANA_HEADER_LEN + (_plLen) + DANA_FOOTER_LEN)


/*QUEUE LENGTHS - HOW MANY FULL MESSAGE FRAMES WILL FIT INTO MESSAGE QUEUE*/
#define CTRL_TASK_QUEUE_LENGTH        4
#define FB_TASK_QUEUE_LENGTH          10

typedef unsigned int size_t;
typedef unsigned int uint16_t;
typedef unsigned char uint8_t;

typedef struct
{
  uint8_t buf[DANA_MAX_LEN_FIELD];
  uint8_t length;
} DanaMessage_t;

typedef enum
{
  TREATMENT_BOLUS,
  TREATMENT_EXTENDED_BOLUS,
  TREATMENT_TEMPORARY_BASAL_RATE
} TreatmentType_t;

typedef struct
{
  TreatmentType_t treatment;
  uint16_t param1;
  uint16_t param2;  
} TreatmentMessage_t;

typedef enum
{
  FEEDBACK_OK,                    //double beeps
  FEEDBACK_FAILURE,               //single beep
  FEEDBACK_SCREAM_OF_DEATH,       //single continuous beep
  FEEDBACK_USER_KEY_PRESS         //user pressed a button
} FeedbackEvent_t;

typedef struct
{
  uint8_t error;
  uint8_t status;
/*
        pump.pumpSuspended = (status & 0x01) == 0x01;
        pump.isTempBasalInProgress = (status & 0x10) == 0x10;
        pump.isExtendedInProgress = (status & 0x04) == 0x04;
        pump.isDualBolusInProgress = (status & 0x08) == 0x08;
*/




  boolean isExtendedInProgress;
  uint16_t extendedBolusMinutes;
  float extendedBolusAbsoluteRate;
  uint16_t extendedBolusSoFarInMinutes;
  float extendedBolusDeliveredSoFar;

  float dailyTotalUnits;
  float maxDailyTotalUnits;
  float reservoirRemainingUnits;
  float currentBasal;
  uint8_t tempBasalPercent;
  uint8_t batteryRemaining;
  float iob;
} DanaRSPump_t;


typedef struct
{
  MessageBufferHandle_t commToCtrlBuffer;
  MessageBufferHandle_t fbToCtrlBuffer;
  boolean fbPdmIsBusy;
  buttonKey_t ctrlActiveButton;

  DanaRSPump_t pump;
} OmniDanaContext_t;



/*MESSAGE BUFFER SIZES*/
#define COMM_TO_CTRL_BUFFER_SIZE      ((sizeof(TreatmentMessage_t) + sizeof(size_t)) * CTRL_TASK_QUEUE_LENGTH )   /*treatment requests*/
#define FB_TO_CTRL_BUFFER_SIZE        ((sizeof(FeedbackEvent_t) + sizeof(size_t)) * FB_TASK_QUEUE_LENGTH)         /*feedback events*/



void BlinkLed(int times);

#endif//__OMNIDANACOMMON_H__
