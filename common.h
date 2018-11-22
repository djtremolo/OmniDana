#ifndef __OMNIDANACOMMON_H__
#define __OMNIDANACOMMON_H__

#include <Arduino_FreeRTOS.h>
#include "message_buffer.h"
#include "timers.h"
#include "semphr.h"
#include "ioInterface.h"
#include <Time.h>


/*TASK PRIORITIES*/
#define UART_TASK_PRIORITY            1
#define CTRL_TASK_PRIORITY            2

/*DANARS MESSAGE SIZES*/
#define DANA_MAX_PAYLOAD_LENGTH       60
#define DANA_HEADER_LEN               5
#define DANA_FOOTER_LEN               4
#define DANA_HEADER_BYTES_AFTER_LEN   2
#define DANA_MAX_BUF_LEN              (DANA_HEADER_LEN + DANA_MAX_PAYLOAD_LENGTH + DANA_FOOTER_LEN)
#define DANA_MAX_LEN_FIELD            (DANA_HEADER_BYTES_AFTER_LEN + DANA_MAX_PAYLOAD_LENGTH)
#define DANA_RAW_MSG_LEN(_plLen)      (DANA_HEADER_LEN + (_plLen) + DANA_FOOTER_LEN)


/*QUEUE LENGTHS - HOW MANY FULL MESSAGE FRAMES WILL FIT INTO MESSAGE QUEUE*/
#define CTRL_TASK_QUEUE_LENGTH        2

#define UNITS_MMOL                    1


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
  TREATMENT_BOLUS_START,
  TREATMENT_BOLUS_STOP,
  TREATMENT_EXTENDED_BOLUS_START,
  TREATMENT_EXTENDED_BOLUS_STOP,
  TREATMENT_TEMPORARY_BASAL_RATE_START,
  TREATMENT_TEMPORARY_BASAL_RATE_STOP

} TreatmentType_t;

typedef struct
{
  TreatmentType_t treatment;
  uint16_t param1;
  uint16_t param2;  
  uint16_t param3;  
} TreatmentMessage_t;

typedef struct
{
  buttonKey_t key;
  bool active;
} KeyEvent_t;

typedef enum
{
  FB_NONE,
  FB_POSITIVE_ACK,
  FB_NEGATIVE_ACK,
  FB_SCREAM_OF_DEATH
} FbEvent_t;

#define PUMP_STATUS_SUSPENDED                    0x01
#define PUMP_STATUS_TEMP_BASAL_IN_PROGRESS       0x10
#define PUMP_STATUS_EXTENDED_BOLUS_IN_PROGRESS   0x04
#define PUMP_STATUS_DUAL_BOLUS_IN_PROGRESS       0x08

typedef struct
{
  boolean pairingRequested;

  uint16_t pass;
  uint8_t error;
  uint8_t status;

  time_t extendedBolusStartTime;
  time_t extendedBolusStartLastReported;
  time_t extendedBolusStopTime;
  time_t extendedBolusStopLastReported;
  boolean isExtendedInProgress;
  uint16_t extendedBolusMinutes;
  float extendedBolusAbsoluteRate;
  float extendedBolusAmount;
  uint16_t extendedBolusSoFarInMinutes;
  float extendedBolusDeliveredSoFar;

  float dailyTotalUnits;
  float maxDailyTotalUnits;
  float reservoirRemainingUnits;
  float currentBasal;
  uint8_t batteryRemaining;
  //float iob;

  uint8_t bolusType;
  float initialBolusAmount;
  uint8_t lastBolusTimeHour;
  uint8_t lastBolusTimeMinute;
  float lastBolusAmount;
  float maxBolus;
  float bolusStep;

/*  time_t tempBasalStartTime;
  time_t tempBasalStartLastReported;
  time_t tempBasalStopTime;
  time_t tempBasalStopLastReported;
*/
  uint8_t isTempBasalInProgress;
  uint8_t tempBasalPercent;
  uint8_t tempBasalDurationHour;    /*150==15min, 160==30min, otherwise hour*3600*/
  uint16_t tempBasalRunningMin;

//  uint8_t serialNumber[10];
//  uint8_t shippingDate[3];    /* byte0=year minus 1900, byte1=month between 0-11, byte2= day of the month between 1-31.*/
//  uint8_t shippingCountry[3];

  uint8_t model;
  uint8_t protocol;
  uint8_t productCode;

  uint8_t activeProfile;


  uint8_t isExtendedBolusEnabled;
  uint8_t bolusCalculationOption;
  uint8_t missedBolusConfig;

  float maxBasal;
  float basalStep;  /*float as u8*/

  //uint16_t profileBasal[24];

 // uint16_t currentTarget;
 // uint16_t currentCIR;
 // uint16_t currentCF;
  uint8_t units;

  uint8_t language;
/*
  uint16_t morningCIR;
  uint16_t afternoonCIR;
  uint16_t eveningCIR;
  uint16_t nightCIR;

  float morningCF;
  float afternoonCF;
  float eveningCF;
  float nightCF;
*/


} DanaRSPump_t;




typedef struct
{
  MessageBufferHandle_t commToCtrlBuffer;
  boolean fbPdmIsBusy;
  buttonKey_t ctrlActiveButton;

  DanaRSPump_t pump;
} OmniDanaContext_t;



/*MESSAGE BUFFER SIZES*/
#define COMM_TO_CTRL_BUFFER_SIZE      ((sizeof(TreatmentMessage_t) + sizeof(size_t)) * CTRL_TASK_QUEUE_LENGTH )   /*treatment requests*/



void BlinkLed(int times);


#endif//__OMNIDANACOMMON_H__
