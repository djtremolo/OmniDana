#include "task.h"
#include "common.h"
#include "ctrlTask.h"
#include "ioInterface.h"
#include "avrtimer.h"

#define MAX_KEY_EVENTS                4
#define MAX_FB_EVENTS                 2

#define SYSTEM_STATE_PULSE_NONE               0x00
#define SYSTEM_STATE_PULSE_SHORT              0x01
#define SYSTEM_STATE_PULSE_HALF               0x0F
#define SYSTEM_STATE_PULSE_DOUBLE_SHORT       0x05

#define SYSTEM_STATE_CLEARING_MASK_NIBBLE     0x0F
#define SYSTEM_STATE_CLEARING_MASK_BYTE       0xFF

#define SYSTEM_STATE_SHIFT_GLOBAL_STATE       (0*4)   
#define SYSTEM_STATE_SHIFT_CONFIGURATION_MODE (1*4)   
#define SYSTEM_STATE_SHIFT_NORMAL_BOLUS       (1*8)
#define SYSTEM_STATE_SHIFT_EXTENDED_BOLUS     (2*8)
#define SYSTEM_STATE_SHIFT_TEMP_BASAL         (3*8)


typedef enum
{
  SYSTEM_STATE_PUMP_ACTIVE,      //not suspended
  SYSTEM_STATE_BOLUS,
  SYSTEM_STATE_TEMP_BASAL,
  SYSTEM_STATE_EXTENDED_BOLUS,
  SYSTEM_STATE_CONFIGURATION_MODE,
} ledSystemState_t;


#define KEY_PRESS_LENGTH_SHORT_IN_MS    130
#define KEY_PRESS_LENGTH_LONG_IN_MS    1300

typedef enum
{
  PRESS_SHORT,
  PRESS_LONG
} buttonPress_t;

#define BUTTON_STATE_ACTIVE_MASK    0x01
#define BUTTON_STATE_SENT_MASK      0x02



#define MAX_PULSE_MEASUREMENTS            4
#define PREV_IDX(idx)                     ((idx) > 0 ? (idx)-1 : (MAX_PULSE_MEASUREMENTS - 1)) 
#define NEXT_IDX(idx)                     (((idx)+1) % MAX_PULSE_MEASUREMENTS)

#define CHECK_PULSE_LENGTH(pTime, minTime, maxTime)     ((((pTime)>=(minTime)) && ((pTime)<=(maxTime))) ? true : false) 

#define FB_DEBOUNCING_TIME_MS         10

static volatile uint32_t systemStateLedSecondValue;
static volatile bool beeperActivityDetected = false;
static bool userActive = false;
static bool controlIsActive = false;
static buttonKey_t controlDriveButton = KEY_NONE;
static uint8_t buttonState[KEY_MAX];

static QueueHandle_t keyQueue;
static QueueHandle_t fbQueue;

TimerHandle_t tickTimer;
SemaphoreHandle_t tickSemaphore;

static bool handleTreatment(OmniDanaContext_t *ctx, TreatmentMessage_t *tr);
static void ctrlTask(void *pvParameters);
static void ctrlTaskTimerTick(TimerHandle_t xTimer);
static bool keyPress(buttonKey_t key, buttonPress_t type);
static void checkUserKeypad();
static bool treatmentBolusStart(OmniDanaContext_t *ctx, uint16_t p1, uint16_t p2, uint16_t p3);
static bool treatmentBolusStop(OmniDanaContext_t *ctx, uint16_t p1, uint16_t p2, uint16_t p3);
static bool treatmentExtendedBolusStart(OmniDanaContext_t *ctx, uint16_t p1, uint16_t p2, uint16_t p3);
static bool treatmentExtendedBolusStop(OmniDanaContext_t *ctx, uint16_t p1, uint16_t p2, uint16_t p3);
static bool treatmentTemporaryBasalStart(OmniDanaContext_t *ctx, uint16_t p1, uint16_t p2, uint16_t p3);
static bool treatmentTemporaryBasalStop(OmniDanaContext_t *ctx, uint16_t p1, uint16_t p2, uint16_t p3);
static void systemStateSet(ledSystemState_t state, bool active);
static void systemStateUpdateLed();
static void systemStateInitialize();


static void fbISR(void);

void ctrlTaskInitialize(OmniDanaContext_t *ctx);

ISR(TIMER2_COMPA_vect)
{
  fbISR();
}

void ctrlTaskInitialize(OmniDanaContext_t *ctx)
{
  Serial.println(F("ctrlTaskInitialize"));
  Serial.print(F("portTICK_PERIOD_MS="));
  Serial.print(portTICK_PERIOD_MS, DEC);
  Serial.println(".");

  systemStateInitialize();

  timer2SetUp();

  keyQueue = xQueueCreate(MAX_KEY_EVENTS, sizeof(KeyEvent_t));
  fbQueue = xQueueCreate(MAX_FB_EVENTS, sizeof(FbEvent_t));

  tickTimer = xTimerCreate
                 ( (const portCHAR *)"ctrlTaskTimer",
                   100 / portTICK_PERIOD_MS, 
                   pdTRUE,
                   (void*)0,
                   ctrlTaskTimerTick );

  if(tickTimer == NULL)
  {
    Serial.println(F("xTimerCreate failed"));
  }

  tickSemaphore = xSemaphoreCreateBinary();
  if(tickSemaphore == NULL)
  {
    Serial.println(F("xSemaphoreCreateBinary failed"));
  }

  if(xTaskCreate(
      ctrlTask, 
      (const portCHAR *)"ctrlTask",
      250,
      (void *)ctx, 
      CTRL_TASK_PRIORITY,
      NULL) != pdPASS)
  {
    Serial.println(F("xTaskCreate failed"));
  }
}

static void systemStateInitialize()
{
  systemStateSet(SYSTEM_STATE_PUMP_ACTIVE, true);
  systemStateSet(SYSTEM_STATE_BOLUS, false);
  systemStateSet(SYSTEM_STATE_TEMP_BASAL, false);
  systemStateSet(SYSTEM_STATE_EXTENDED_BOLUS, false);
  systemStateSet(SYSTEM_STATE_CONFIGURATION_MODE, false);
}


static void systemStateSet(ledSystemState_t state, bool active)
{
  uint32_t tmp;
  uint32_t out = systemStateLedSecondValue;

  /*
    Output: 
      64 bits, 100ms each, 6.4 seconds per repetition 
        first 32 bits are always zero
        the second 32 bits are divided into four segments as defined below


    00  01  02  03  04  05  06  07  08  09  10  11  12  13  14  15  16  17  18  19  20  21  22  23  24  25  26  27  28  29  30  31 
    ----------- byte 0 -------------____________ byte 1 ____________------------ byte 2 ------------___________ byte 3 _____________  
    ----------------________________----------------________________----------------________________----------------________________

    ****                                                                                                                                  Pump Active (not suspended)
    ****    ****                                                                                                                          Pump Not Active (suspended)

                    ****    ****                                                                                                          Configuration Mode Active

                                    
                                    ****                                                                                                  Bolus Not Active
                                    ****************                                                                                      Bolus Active
                                    
                                                                    ****                                                                  Temp Basal Not Active
                                                                    ****************                                                      Temp Basal Active
                                                                                                    
                                                                                                    ****                                  Ext Bolus Not Active
                                                                                                    ****************                      Ext Bolus Active
  */


  switch(state)
  {
    case SYSTEM_STATE_PUMP_ACTIVE:
      tmp = (uint32_t)((active ? SYSTEM_STATE_PULSE_SHORT : SYSTEM_STATE_PULSE_DOUBLE_SHORT)) << SYSTEM_STATE_SHIFT_GLOBAL_STATE;
      out &= ~(((uint32_t)SYSTEM_STATE_CLEARING_MASK_NIBBLE << SYSTEM_STATE_SHIFT_GLOBAL_STATE));
      out |= tmp;
      break;
    case SYSTEM_STATE_BOLUS:
      tmp = (uint32_t)((active ? SYSTEM_STATE_PULSE_HALF : SYSTEM_STATE_PULSE_SHORT)) << SYSTEM_STATE_SHIFT_NORMAL_BOLUS;
      out &= ~(((uint32_t)SYSTEM_STATE_CLEARING_MASK_BYTE << SYSTEM_STATE_SHIFT_NORMAL_BOLUS));
      out |= tmp;
      break;
    case SYSTEM_STATE_TEMP_BASAL:
      tmp = (uint32_t)((active ? SYSTEM_STATE_PULSE_HALF : SYSTEM_STATE_PULSE_SHORT)) << SYSTEM_STATE_SHIFT_TEMP_BASAL;
      out &= ~(((uint32_t)SYSTEM_STATE_CLEARING_MASK_BYTE << SYSTEM_STATE_SHIFT_TEMP_BASAL));
      out |= tmp;
      break;
    case SYSTEM_STATE_EXTENDED_BOLUS:
      tmp = (uint32_t)((active ? SYSTEM_STATE_PULSE_HALF : SYSTEM_STATE_PULSE_SHORT)) << SYSTEM_STATE_SHIFT_EXTENDED_BOLUS;
      out &= ~(((uint32_t)SYSTEM_STATE_CLEARING_MASK_BYTE << SYSTEM_STATE_SHIFT_EXTENDED_BOLUS));
      out |= tmp;
      break;
    case SYSTEM_STATE_CONFIGURATION_MODE:
      tmp = (uint32_t)((active ? SYSTEM_STATE_PULSE_DOUBLE_SHORT : SYSTEM_STATE_PULSE_NONE)) << SYSTEM_STATE_SHIFT_CONFIGURATION_MODE;
      out &= ~(((uint32_t)SYSTEM_STATE_CLEARING_MASK_NIBBLE << SYSTEM_STATE_SHIFT_CONFIGURATION_MODE));
      out |= tmp;
      break;
    default:
      break;
  }

  Serial.print(F("SystemState = "));
  Serial.print(out, HEX);
  Serial.println(F("."));

  systemStateLedSecondValue = out;  /*latch to LED*/
}

static void systemStateUpdateLed()
{
  static uint8_t index = 0;
  bool outBit = false;

  /*first 32 bits are always zero, the second word is get from systemStateLedSecondValue*/
  if(index > 31)
  {
    uint8_t shiftBits = index-32; /*gets 0...31*/
    outBit = (systemStateLedSecondValue >> shiftBits) & 0x1; /*extract the desired bit*/
  }

  /*write value to led*/
  digitalWrite(LED_BUILTIN, (outBit ? HIGH : LOW));

  /*advance to next*/
  index = (index + 1) % 64;

} 


static void checkUserKeypad()
{
  bool keyActive = false;

  /*period: ̃100ms*/

  //vTaskSuspendAll();
  for(int idx = 0; idx < KEY_MAX; idx++)
  {  
    buttonKey_t b = (buttonKey_t)idx;

    /*skip the buttons that are actively driven by the control pins*/
    if(b != controlDriveButton)
    {
      /*check if the button is active now*/
      if(IoInterfaceReadButtonPin(b))
      {
        keyActive = true;

        buttonState[b] |= BUTTON_STATE_ACTIVE_MASK;

        /*button is active, check if we need to send an event about it*/
        if((buttonState[b] & BUTTON_STATE_SENT_MASK) == 0)
        {        
          buttonState[b] |= BUTTON_STATE_SENT_MASK;

          /*short press detected, send event*/
          KeyEvent_t e;
          e.key = b;
          e.active = true;

          xQueueSendToBack(keyQueue, &e, 0);

          //Serial.println(F("SENT!"));
          //digitalWrite(LED_BUILTIN, HIGH);

        }
      }
      else
      {
        uint8_t requiredBits = (BUTTON_STATE_ACTIVE_MASK | BUTTON_STATE_SENT_MASK);

        /*the button was released. Check if we have sent an activation event. If yes, then send a deactivation event.*/
        if((buttonState[b] & requiredBits) == requiredBits)
        {
          KeyEvent_t e;
          e.key = b;
          e.active = false;
          xQueueSendToBack(keyQueue, &e, 0);
          buttonState[b] &= (~requiredBits);  /*drop active and sent*/

          //digitalWrite(LED_BUILTIN, LOW);

        }
      }
    }
  }
  //vTaskResumeAll();

  if(keyActive)
  {
    userActive = true; /*inform system of user keypad activity. Latched indication, must be cleared by the system.*/
  }
}




#define TASK_TICK_RELOAD_VALUE    0 //10
static void ctrlTaskTimerTick(TimerHandle_t xTimer)
{
  static uint16_t tickCounter = TASK_TICK_RELOAD_VALUE;
  (void)xTimer;

  /*interval ~100ms*/

  /*per each tick, let's see if the user keypad is busy*/
  checkUserKeypad();

  /*update LED with the state information*/
  systemStateUpdateLed();

  if(tickCounter-- == 0)
  {
    tickCounter = TASK_TICK_RELOAD_VALUE;
    xSemaphoreGive(tickSemaphore);
  }
}
#if 0
static void fbISR(void)
{
  static bool debouncingLastState = true;
  static uint16_t consecutiveStates = 0;
  static bool lastActionState = false;

  bool fbVal = IoInterfaceReadFeedbackPin(FB_BEEP);
  unsigned long currentMillis = millis();
  unsigned long timeFromLastChange;
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;


  /*debounce until a steady signal is found*/
  if(fbVal == debouncingLastState)
  {
    if(consecutiveStates < FB_DEBOUNCING_TIME_MS)
    {
      consecutiveStates++;
      return; /*not yet*/
    }
  }
  else
  {
    debouncingLastState = fbVal;
    consecutiveStates = 0;
    return;
  }

  //digitalWrite(LED_BUILTIN, fbVal); //HIGH

  /*act only when the state has been changed*/
  if(fbVal != lastActionState)
  {
    static unsigned long pulseTimes[MAX_PULSE_MEASUREMENTS];
    static uint8_t pulseIdx;
    static unsigned long previousMillis[2] = {};
    static uint8_t newStateChanges = 0;
  
    /*store this state for change detection logic*/
    lastActionState = fbVal;

    /*measure pulse lengths*/
    if(fbVal)
    {
      /*rising edge*/
      previousMillis[1] = currentMillis;
      timeFromLastChange = previousMillis[1] - previousMillis[0];   /*time diff between last falling edge and this rising edge*/
    }
    else
    {
      /*falling edge*/
      previousMillis[0] = currentMillis;
      timeFromLastChange = previousMillis[0] - previousMillis[1];   /*time diff between last rising edge and this falling edge*/ 
    }

    /*count state changes but saturate at 4 as more would not make any difference*/
    if(newStateChanges < MAX_PULSE_MEASUREMENTS)
    {
      newStateChanges++;
    }

    /*store time delta of previous pulse*/
    pulseTimes[pulseIdx] = timeFromLastChange;

  #if 1

    /*detect positive ack: two active pulses of 135ms, 115ms in between*/
    if((!fbVal) && (newStateChanges >= 4))
    {
      uint8_t secondHighPulseIdx = pulseIdx;
      uint8_t middleLowPulseIdx = PREV_IDX(secondHighPulseIdx);
      uint8_t firstHighPulseIdx = PREV_IDX(middleLowPulseIdx);
      /*at the low level now, measure last positive pulse*/
      
      if(CHECK_PULSE_LENGTH(pulseTimes[firstHighPulseIdx], 100, 200))
      {
        if(CHECK_PULSE_LENGTH(pulseTimes[middleLowPulseIdx], 100, 200))
        {
          if(CHECK_PULSE_LENGTH(pulseTimes[secondHighPulseIdx], 100, 200))
          {
            FbEvent_t fb = FB_POSITIVE_ACK;
            xQueueSendToBackFromISR(fbQueue, &fb, &xHigherPriorityTaskWoken);
          }
        }
      }
    }
    /*detect negative ack: two active pulses of 135ms, 115ms in between*/
    if((!fbVal) && (newStateChanges >= 2))
    {
      uint8_t highPulseIdx = pulseIdx;
      /*at the low level now, measure last positive pulse*/
      
      if(CHECK_PULSE_LENGTH(pulseTimes[highPulseIdx], 700, 900))
      {
        FbEvent_t fb = FB_NEGATIVE_ACK;
        xQueueSendToBackFromISR(fbQueue, &fb, &xHigherPriorityTaskWoken);
      }
    }
    /*scream of death detection: we will see it once the user has cleared it: one long pulse, > 2000ms*/
    if((!fbVal) && (newStateChanges >= 2))
    {
      uint8_t highPulseIdx = pulseIdx;
      /*at the low level now, measure last positive pulse*/
      
      if(CHECK_PULSE_LENGTH(pulseTimes[highPulseIdx], 2000, (unsigned long)24*60*60*1000))  /*don't detect screaming of more than 24 hours. Should we?*/
      {
        FbEvent_t fb = FB_SCREAM_OF_DEATH;
        xQueueSendToBackFromISR(fbQueue, &fb, &xHigherPriorityTaskWoken);
      }
    }

  #endif

    /*advance to next pulse index*/
    pulseIdx = NEXT_IDX(pulseIdx);

    if(xHigherPriorityTaskWoken)
    {
      /* Actual macro used here is port specific. */
    //  taskYIELD_FROM_ISR();
    }    
   // digitalWrite(LED_BUILTIN, LOW); //HIGH
  }
}

#else

#define FB_MAX_PULSE_LENGTH         10000
static void fbISR(void)
{
  static uint16_t onPulseInMs = 0;
  static uint16_t offPulseInMs = 0;
  static bool positiveAckFirstPulseFound = false;
  static bool positiveAckSecondPulseFound = false;
  static bool prevActValue = true;
  bool currentActValue = beeperActivityDetected;
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  /*the beeper gives audible pulses, but is driven by a 4kHz square wave. The interrupt fires 
  at each square wave pulse and triggers the beeperActivityDetected flag.
  That flag is checked here at each 1ms tick and the feedback signals are generated from here.*/

  /*first, let's clear the flag risen by the ISR*/
  beeperActivityDetected = false; /*clear flag*/

  /*check for activity. If something has happened, we are in "beep" state*/
  if(currentActValue)
  {
    /*cumulate time during "on" state*/
    if(onPulseInMs < FB_MAX_PULSE_LENGTH)
      onPulseInMs++;
  }
  else
  {
    /*cumulate time during "off" state*/
    if(offPulseInMs < FB_MAX_PULSE_LENGTH)
      offPulseInMs++;
  }

  /*check if the state has changed, i.e. if this is an edge*/
  if(prevActValue != currentActValue)
  {
    /*this is an edge, so let's see if it was rising or falling one*/
    if(currentActValue)
    {
      /*logical transition: off->on*/

      /*check if start of a positive ack: 135+115+135 double pulse*/
      if(positiveAckFirstPulseFound && CHECK_PULSE_LENGTH(offPulseInMs, 80, 160))
      {
        positiveAckSecondPulseFound = true;
      }

      /*clear counter*/
      offPulseInMs = 0;
    }
    else
    {
      /*logical transition: on->off*/

      /*check if this is the final pulse of a positive ack*/
      if(positiveAckSecondPulseFound && CHECK_PULSE_LENGTH(onPulseInMs, 100, 180))
      {
        FbEvent_t fb = FB_POSITIVE_ACK;
        xQueueSendToBackFromISR(fbQueue, &fb, &xHigherPriorityTaskWoken);
      }
      /*check if first pulse of a positive ack: ~135*/
      else if(CHECK_PULSE_LENGTH(onPulseInMs, 100, 180))
      {
        positiveAckFirstPulseFound = true;
      }
      /*check if negative ack: ~800ms single pulse*/
      else if(CHECK_PULSE_LENGTH(onPulseInMs, 700, 900))
      {
        FbEvent_t fb = FB_NEGATIVE_ACK;
        xQueueSendToBackFromISR(fbQueue, &fb, &xHigherPriorityTaskWoken);
      }
      else
      {
        /*positive ack pulse failed*/
        positiveAckFirstPulseFound = false;
        positiveAckSecondPulseFound = false;
      }

      /*clear counter*/
      onPulseInMs = 0;
    }

    /*store for next round*/
    prevActValue = currentActValue;
  }

  /*check if scream of death: >2000ms */
  if(onPulseInMs == 5000)   /*report scream once at 5 seconds, the counter will saturate at 10k*/
  {
    FbEvent_t fb = FB_SCREAM_OF_DEATH;
    xQueueSendToBackFromISR(fbQueue, &fb, &xHigherPriorityTaskWoken);
  }
}

#endif


typedef enum
{
  CONF_MODE_NONE = 0,
  CONF_MODE_INSULIN_RESERVOIR,
} confMode_t;


#define CONF_PUMP_MAX_UNITS   200
#define CONF_PUMP_MIN_UNITS   0
#define CONF_PUMP_STEP        10

#define CONF_TIMEOUT_MS       5000


static bool processKeyEvent(OmniDanaContext_t *ctx, KeyEvent_t *ev)
{
  static uint32_t lastActivityTimestamp = 0;
  static confMode_t activeConfigurationMode = CONF_MODE_NONE;
  uint16_t tmp;
  uint32_t nowMs = millis();

  switch(ev->key)
  {
    case KEY_F1:
      activeConfigurationMode = (ev->active ? CONF_MODE_INSULIN_RESERVOIR : CONF_MODE_NONE);
      break;
    case KEY_F2:
      break;
    case KEY_F3:
      break;
    case KEY_HOME:
      break;
    case KEY_UP:
      if(ev->active)
      {
        switch(activeConfigurationMode)
        {
          case CONF_MODE_INSULIN_RESERVOIR:
            tmp = ctx->pump.reservoirRemainingUnits;
            ctx->pump.reservoirRemainingUnits = (tmp > (CONF_PUMP_MAX_UNITS - CONF_PUMP_STEP) ? CONF_PUMP_MAX_UNITS : tmp + CONF_PUMP_STEP);
            break;
          default:
            break;
        }
      }

      break;
    case KEY_DOWN:
      if(ev->active)
      {
        switch(activeConfigurationMode)
        {
          case CONF_MODE_INSULIN_RESERVOIR:
            tmp = ctx->pump.reservoirRemainingUnits;
            ctx->pump.reservoirRemainingUnits = (tmp < (CONF_PUMP_MIN_UNITS + CONF_PUMP_STEP) ? CONF_PUMP_MIN_UNITS : tmp - CONF_PUMP_STEP);
            break;
          default:
            break;
        }
      }
      break;
    case KEY_QUESTIONMARK:
      break;

    default:
      break;
  }

  if((nowMs - lastActivityTimestamp) > CONF_TIMEOUT_MS)
  {
    activeConfigurationMode = CONF_MODE_NONE;
  }
  
  /*store this timestamp*/
  lastActivityTimestamp = nowMs;

  bool inConfigMode = (activeConfigurationMode != CONF_MODE_NONE);

  systemStateSet(SYSTEM_STATE_CONFIGURATION_MODE, inConfigMode);

  return inConfigMode;
}



void beeperFollowerISR(void)
{
  /*a change is detected, just report it*/
  beeperActivityDetected = true;    /*set flag*/
}


static void ctrlTask(void *pvParameters)
{
  OmniDanaContext_t *ctx = (OmniDanaContext_t *)pvParameters;

#if DEBUG_PRINT
  Serial.print(F("ctrlTask: starting with ctx = "));
  Serial.print((uint16_t)ctx, HEX);
  Serial.print(F(", ctx->commToCtrlBuffer = "));
  Serial.print((uint16_t)(ctx->commToCtrlBuffer), HEX);
  Serial.println();
#endif

  IoInterfaceSetupPins(beeperFollowerISR);

  xTimerStart(tickTimer, 0);
  timer2Start();  


  while (1)
  {
    TreatmentMessage_t treatment;
    KeyEvent_t keyEvent;
    FbEvent_t fbEvent;

    /*wait for a tick*/
    xSemaphoreTake(tickSemaphore, portMAX_DELAY);

  //  Serial.println("TICK");

    if(pdTRUE == xQueueReceive(keyQueue, &keyEvent, 0))
    {
      Serial.print("Event received: key=");
      Serial.print(keyEvent.key, DEC);
      Serial.print(", act=");
      Serial.print(keyEvent.active, DEC);
      Serial.println(".");


      processKeyEvent(ctx, &keyEvent);
    }

    if(pdTRUE == xQueueReceive(fbQueue, &fbEvent, 0))
    {
      Serial.print("Feedback received: fb=");
      switch(fbEvent)
      {
        case FB_NONE:
          Serial.print(F("FB_NONE"));
          break;
        case FB_POSITIVE_ACK:
          Serial.print(F("FB_POSITIVE_ACK"));
          break;
        case FB_NEGATIVE_ACK:
          Serial.print(F("FB_NEGATIVE_ACK"));
          break;
        case FB_SCREAM_OF_DEATH:
          Serial.print(F("FB_SCREAM_OF_DEATH"));
          break;
      }
      Serial.println(".");
    }


    int recBytes = xMessageBufferReceive(ctx->commToCtrlBuffer, (void *)&treatment, sizeof(TreatmentMessage_t), 0);

    if (recBytes == sizeof(TreatmentMessage_t))
    {
#if DEBUG_PRINT
      Serial.flush();
      Serial.println(F("ctrlTask: treatment received!"));
#endif
      handleTreatment(ctx, &treatment);
    }


    /*and continue waiting for next...*/
  }
}

static bool handleTreatment(OmniDanaContext_t *ctx, TreatmentMessage_t *tr)
{
  bool ret = true;
  (void)ctx;
  (void)tr;
#if DEBUG_PRINT
  Serial.println(F("handleTreatment:"));
#endif

  /*inform keypad busy detector that we are running a control*/
  controlIsActive = true;
  switch(tr->treatment)
  {

    case TREATMENT_BOLUS_START:
      ret = treatmentBolusStart(ctx, tr->param1, tr->param2, tr->param3);
      break;
    case TREATMENT_BOLUS_STOP:
      ret = treatmentBolusStop(ctx, tr->param1, tr->param2, tr->param3);
      break;
    case TREATMENT_EXTENDED_BOLUS_START:
      ret = treatmentExtendedBolusStart(ctx, tr->param1, tr->param2, tr->param3);
      break;
    case TREATMENT_EXTENDED_BOLUS_STOP:
      ret = treatmentExtendedBolusStop(ctx, tr->param1, tr->param2, tr->param3);
      break;
    case TREATMENT_TEMPORARY_BASAL_RATE_START:
      ret = treatmentTemporaryBasalStart(ctx, tr->param1, tr->param2, tr->param3);
      break;
    case TREATMENT_TEMPORARY_BASAL_RATE_STOP:
      ret = treatmentTemporaryBasalStop(ctx, tr->param1, tr->param2, tr->param3);
      break;

    default:
      break;
  }
  /*inform keypad busy detector that we are done*/
  controlIsActive = false;

  return ret;
}

static FbEvent_t waitForFeedback(uint16_t maxWaitMs)
{
  FbEvent_t fbEvent;
  if(pdTRUE == xQueueReceive(fbQueue, &fbEvent, maxWaitMs / portTICK_PERIOD_MS))
  {
    /*consume acks but feed back the scream of death so the control task can handle it*/
    if(fbEvent == FB_SCREAM_OF_DEATH)
    {
      xQueueSendToBack(fbQueue, &fbEvent, 0);
    }
    return fbEvent;
  }

  return FB_NONE;
}


/*
  FB_NONE,
  FB_POSITIVE_ACK,
  FB_NEGATIVE_ACK,
  FB_SCREAM_OF_DEATH
} FbEvent_t;
*/



#define GO_TO_MENU_WITH_BUSY_CHECK() { if(treatmentGoToMenu() == false) return false; }

#define KEYPRESS_WITH_BUSY_CHECK(bKey, bPress) { if(keyPress((bKey), (bPress)) == false) return false; }

static bool treatmentGoToMenu()
{
  /*activate screen*/
  KEYPRESS_WITH_BUSY_CHECK(KEY_HOME, PRESS_LONG);

  /*get rid of the randomized login screen*/
  KEYPRESS_WITH_BUSY_CHECK(KEY_F1, PRESS_SHORT);
  KEYPRESS_WITH_BUSY_CHECK(KEY_F2, PRESS_SHORT);

  /*if communication error, we need to press "skip" button. It's harmless at this state.*/
  KEYPRESS_WITH_BUSY_CHECK(KEY_F2, PRESS_LONG);

  /*if the PDM does not have pod communication, the F1 won't bring the menu so we need to press HOME instead.
  Going to menu by HOME button works in both cases*/
  KEYPRESS_WITH_BUSY_CHECK(KEY_HOME, PRESS_SHORT);

  return true;
}

static bool treatmentBolusStart(OmniDanaContext_t *ctx, uint16_t p1, uint16_t p2, uint16_t p3)
{
  bool ret = false;
  (void)ctx;

  /*Returns false in case of failure (i.e. user keypad activity or if feedback was negative).*/

  /*
  param1: amount (units multiplied with 100)
  param2: not used
  */
  uint16_t stepsForBolusAmount = p1 / 10;   /*each step is 0.10u. Requested=12.3u -> p1==1230 -> steps = 123*/

  (void)p2;
  (void)p3;


  #if DEBUG_PRINT
  Serial.println(F("treatmentBolusStart"));
  #endif

  GO_TO_MENU_WITH_BUSY_CHECK();

  /*choose Bolus*/
  KEYPRESS_WITH_BUSY_CHECK(KEY_F3, PRESS_SHORT);

  /*get rid of BG input prompts*/
  KEYPRESS_WITH_BUSY_CHECK(KEY_F2, PRESS_SHORT);
  KEYPRESS_WITH_BUSY_CHECK(KEY_F2, PRESS_SHORT);

  /*input bolus amount*/
  for(uint16_t step = 0; step < stepsForBolusAmount; step++)
  {
    KEYPRESS_WITH_BUSY_CHECK(KEY_UP, PRESS_SHORT);
  }

  /*accept*/
  KEYPRESS_WITH_BUSY_CHECK(KEY_F3, PRESS_SHORT);

  /*confirm*/
  KEYPRESS_WITH_BUSY_CHECK(KEY_F2, PRESS_SHORT);


  #if DEBUG_PRINT
  Serial.println(F("waiting for feedback"));
  #endif

  /*we should get positive feedback*/  
  FbEvent_t fb = waitForFeedback(10000);

  switch(fb)
  {
    case FB_POSITIVE_ACK:

      #if DEBUG_PRINT
      Serial.println(F("FB_POSITIVE_ACK"));
      #endif

      ret = true;
      break;

    case FB_NEGATIVE_ACK:
      #if DEBUG_PRINT
      Serial.println(F("FB_NEGATIVE_ACK"));
      #endif
      break;

    default:
      #if DEBUG_PRINT
      Serial.println(F("failure"));
      #endif
      break;
  }




  return ret;
}

static bool treatmentBolusStop(OmniDanaContext_t *ctx, uint16_t p1, uint16_t p2, uint16_t p3)
{
  /*Returns false in case of failure (i.e. user keypad activity or if feedback was negative).*/
  (void)ctx;
  (void)p1;
  (void)p2;
  (void)p3;

  return true;
}

static bool treatmentExtendedBolusStart(OmniDanaContext_t *ctx, uint16_t p1, uint16_t p2, uint16_t p3)
{
  bool ret = false;
  uint16_t stepsForBolusAmount = p1 / 10;   /*each step is 0.10u. Requested=12.3u -> p1==1230 -> steps = 123*/
  uint16_t stepsForBolusNow = p2 / 10;   /*each step is 0.10u. Requested=12.3u -> p1==1230 -> steps = 123*/
  uint16_t halfHours = p3;

  (void)ctx;

  /*Returns false in case of failure (i.e. user keypad activity or if feedback was negative).*/

  /*
  param1: total amount (units multiplied with 100)
  param2: amount now (units multiplied with 100)
  param3: time in half hours
  */

  //#if DEBUG_PRINT
  Serial.println(F("treatmentExtendedBolusStart"));
  //#endif
  
  systemStateSet(SYSTEM_STATE_EXTENDED_BOLUS, true);


  GO_TO_MENU_WITH_BUSY_CHECK();

  /*choose Bolus*/
  KEYPRESS_WITH_BUSY_CHECK(KEY_F3, PRESS_SHORT);

  /*get rid of BG input prompts*/
  KEYPRESS_WITH_BUSY_CHECK(KEY_F2, PRESS_SHORT);
  KEYPRESS_WITH_BUSY_CHECK(KEY_F2, PRESS_SHORT);

  /*input total amount*/
  for(uint16_t step = 0; step < stepsForBolusAmount; step++)
  {
    KEYPRESS_WITH_BUSY_CHECK(KEY_UP, PRESS_SHORT);
  }

  /*extend*/
  KEYPRESS_WITH_BUSY_CHECK(KEY_F2, PRESS_SHORT);

  /*input now part*/
  for(uint16_t step = 0; step < stepsForBolusNow; step++)
  {
    KEYPRESS_WITH_BUSY_CHECK(KEY_UP, PRESS_SHORT);
  }

  /*accept*/
  KEYPRESS_WITH_BUSY_CHECK(KEY_F3, PRESS_SHORT);

  /*make sure we start from 0.5h time*/
  for(uint16_t step = 0; step < 10; step++)    /*usually, we need only one step down but in some cases there might be more needed*/
  {
    KEYPRESS_WITH_BUSY_CHECK(KEY_DOWN, PRESS_SHORT);
  }

  /*input extension time*/
  for(uint16_t step = 1; step < halfHours; step++)    /*start from 1 because the minimum is 0.5h already*/
  {
    KEYPRESS_WITH_BUSY_CHECK(KEY_UP, PRESS_SHORT);
  }

  /*confirm*/
  KEYPRESS_WITH_BUSY_CHECK(KEY_F2, PRESS_SHORT);



  /*we should get positive feedback*/  
  FbEvent_t fb = waitForFeedback(10000);

  switch(fb)
  {
    case FB_POSITIVE_ACK:

    //  #if DEBUG_PRINT
      Serial.println(F("FB_POSITIVE_ACK"));
   //   #endif

      ret = true;
      break;

    case FB_NEGATIVE_ACK:
    //  #if DEBUG_PRINT
      Serial.println(F("FB_NEGATIVE_ACK"));
     // #endif
      break;

    default:
    //  #if DEBUG_PRINT
      Serial.println(F("failure"));
   //   #endif
      break;
  }


  return ret;
}

static bool treatmentExtendedBolusStop(OmniDanaContext_t *ctx, uint16_t p1, uint16_t p2, uint16_t p3)
{
  /*Returns false in case of failure (i.e. user keypad activity or if feedback was negative).*/
  (void)ctx;
  (void)p1;
  (void)p2;
  (void)p3;


  systemStateSet(SYSTEM_STATE_EXTENDED_BOLUS, false);


  return true;
}


static bool keyPress(buttonKey_t key, buttonPress_t type)
{
  if(userActive)
  {
    userActive = false; /*clear and report the cancellation*/
    return false;
  }

  /*let key input detector we are about to control this button*/
  controlDriveButton = key;

  /*control keypad: Activate key*/
  IoInterfaceSetButtonPin(key, true);

  /*keep button pressed for certain time*/
  int delayMs = (type == PRESS_SHORT ? KEY_PRESS_LENGTH_SHORT_IN_MS : KEY_PRESS_LENGTH_LONG_IN_MS); 
  vTaskDelay(delayMs / portTICK_PERIOD_MS);

  /*deactivate key*/
  IoInterfaceSetButtonPin(key, false);

  /*wait fixed inter-button time*/
  vTaskDelay(KEY_PRESS_LENGTH_SHORT_IN_MS / portTICK_PERIOD_MS);

  /*inform key detector that we are done with this button*/
  controlDriveButton = KEY_NONE;

  return true;
}


static bool treatmentTemporaryBasalStart(OmniDanaContext_t *ctx, uint16_t p1, uint16_t p2, uint16_t p3)
{
  bool ret = false;
  (void)ctx;

  /*Returns false in case of failure (i.e. user keypad activity or if feedback was negative).*/

  uint16_t newBasalRate = p1;   /**/
  uint16_t timeInMinutes = p2;   /**/

  bool needsToCancelTemporaryBasal = false;
  (void)p3;


  //#if DEBUG_PRINT
  Serial.print(F("treatmentTemporaryBasalStart("));
  Serial.print(p1, DEC);
  Serial.print(F(","));
  Serial.print(p2, DEC);
  Serial.println(F(")."));

  systemStateSet(SYSTEM_STATE_TEMP_BASAL, true);

  //#endif
  if(needsToCancelTemporaryBasal)
  {
    if(!treatmentTemporaryBasalStop(ctx, 0, 0, 0))
    {
      return false;
    }

    /*activate menu again with home button*/
    KEYPRESS_WITH_BUSY_CHECK(KEY_HOME, PRESS_SHORT);
  }
  else
  {
    /*start*/
    GO_TO_MENU_WITH_BUSY_CHECK();
  }

  /*go 2 menu items down: set temporary basal*/
  KEYPRESS_WITH_BUSY_CHECK(KEY_DOWN, PRESS_SHORT);
  KEYPRESS_WITH_BUSY_CHECK(KEY_DOWN, PRESS_SHORT);

  /*clear current temp basal value to zero*/
  for(uint16_t step=0; step < (20 * 5); step++)   /*needs 20 steps for one unit, estimating 5u/hr is our max temp basal*/
  {
    KEYPRESS_WITH_BUSY_CHECK(KEY_DOWN, PRESS_SHORT);
  }

  /*set temporary basal*/
  uint16_t stepsForTempBasal = newBasalRate / 5;
  for(uint16_t step=0; step < stepsForTempBasal; step++)   /*needs 20 steps for one unit, estimating 5u/hr is our max temp basal. TODO*/
  {
    KEYPRESS_WITH_BUSY_CHECK(KEY_UP, PRESS_SHORT);
  }

  /*accept*/
  KEYPRESS_WITH_BUSY_CHECK(KEY_F3, PRESS_SHORT);

  /*clear current temp basal value to zero*/
  for(uint16_t step=0; step < 10; step++)   /*1 step == 30min, this clears from up to 5 hr*/
  {
    KEYPRESS_WITH_BUSY_CHECK(KEY_DOWN, PRESS_SHORT);
  }

  /*set time in half hours*/
  uint16_t halfHours = timeInMinutes / 30;
  for(uint16_t step=1; step < halfHours; step++)   /*starts from 0.5h anyway, so -1*/
  {
    KEYPRESS_WITH_BUSY_CHECK(KEY_UP, PRESS_SHORT);
  }

  /*accept*/
  KEYPRESS_WITH_BUSY_CHECK(KEY_F3, PRESS_SHORT);

  /*confirm*/
  KEYPRESS_WITH_BUSY_CHECK(KEY_F2, PRESS_SHORT);

  


  //#if DEBUG_PRINT
  Serial.println(F("waiting for feedback"));
  //#endif

  /*we should get positive feedback*/  
  FbEvent_t fb = waitForFeedback(10000);

  switch(fb)
  {
    case FB_POSITIVE_ACK:

      #if DEBUG_PRINT
      Serial.println(F("FB_POSITIVE_ACK"));
      #endif

      ret = true;
      break;

    case FB_NEGATIVE_ACK:
      #if DEBUG_PRINT
      Serial.println(F("FB_NEGATIVE_ACK"));
      #endif
      break;

    default:
      #if DEBUG_PRINT
      Serial.println(F("failure"));
      #endif
      break;
  }

  return ret;
}


static bool treatmentTemporaryBasalStop(OmniDanaContext_t *ctx, uint16_t p1, uint16_t p2, uint16_t p3)
{
  bool ret = false;
  (void)ctx;

  /*Returns false in case of failure (i.e. user keypad activity or if feedback was negative).*/
  (void)p1;
  (void)p2;
  (void)p3;
  //#if DEBUG_PRINT
  Serial.println(F("treatmentTemporaryBasalStop()"));

  systemStateSet(SYSTEM_STATE_TEMP_BASAL, false);

  /*Returns false in case of failure (i.e. user keypad activity or if feedback was negative).*/
  GO_TO_MENU_WITH_BUSY_CHECK();

  /*go 5 menu items down: Cancel*/
  KEYPRESS_WITH_BUSY_CHECK(KEY_DOWN, PRESS_SHORT);
  KEYPRESS_WITH_BUSY_CHECK(KEY_DOWN, PRESS_SHORT);
  KEYPRESS_WITH_BUSY_CHECK(KEY_DOWN, PRESS_SHORT);
  KEYPRESS_WITH_BUSY_CHECK(KEY_DOWN, PRESS_SHORT);
  KEYPRESS_WITH_BUSY_CHECK(KEY_DOWN, PRESS_SHORT);

  /*accept*/
  KEYPRESS_WITH_BUSY_CHECK(KEY_F3, PRESS_SHORT);

  /*confirm*/
  KEYPRESS_WITH_BUSY_CHECK(KEY_F2, PRESS_SHORT);



  //#if DEBUG_PRINT
  Serial.println(F("waiting for feedback for stopping temp basal"));
  //#endif

  /*we should get positive feedback*/  
  FbEvent_t fb = waitForFeedback(10000);

  switch(fb)
  {
    case FB_POSITIVE_ACK:

      #if DEBUG_PRINT
      Serial.println(F("FB_POSITIVE_ACK"));
      #endif

      ret = true;
      break;

    case FB_NEGATIVE_ACK:
      #if DEBUG_PRINT
      Serial.println(F("FB_NEGATIVE_ACK"));
      #endif
      break;

    default:
      #if DEBUG_PRINT
      Serial.println(F("failure"));
      #endif
      break;
  }

  return ret;
}


#if 0
static bool checkFeedback()
{
  /*positive ack pulse:
  off
  135ms on
  115ms off
  135ms on
  off

  _________|----135ms-----|__115ms_____|----135ms-----|___________


  negative ack pulse:
  off
  760ms on
  off

  ________|--------------------760ms---------------------------------------------------------------|_____________


  */
  int totalWaitTimeMs = 1000 + waitingTimeMs; /*add 1000 ms for the beeps*/
  uint16_t consecutiveTime = 0;
  bool

  while(totalWaitTimeMs > 0)
  {
    bool beep = IoInterfaceReadFeedbackPin(FB_BEEP);


    vTaskDelay(delayMs / portTICK_PERIOD_MS);




  }




}
#endif