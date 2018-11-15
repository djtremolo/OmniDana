#include "task.h"
#include "common.h"
#include "ctrlTask.h"
#include "ioInterface.h"
#include "avrtimer.h"

#define MAX_KEY_EVENTS                4
#define MAX_FB_EVENTS                 2

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
static bool treatmentBolusStart(uint16_t p1, uint16_t p2, uint16_t p3);
static bool treatmentBolusStop(uint16_t p1, uint16_t p2, uint16_t p3);
static bool treatmentExtendedBolusStart(uint16_t p1, uint16_t p2, uint16_t p3);
static bool treatmentExtendedBolusStop(uint16_t p1, uint16_t p2, uint16_t p3);
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




#define TASK_TICK_RELOAD_VALUE    10
static void ctrlTaskTimerTick(TimerHandle_t xTimer)
{
  static uint16_t tickCounter = TASK_TICK_RELOAD_VALUE;
  (void)xTimer;

  /*interval ~100ms*/

  /*per each tick, let's see if the user keypad is busy*/
  checkUserKeypad();

  if(tickCounter-- == 0)
  {
    tickCounter = TASK_TICK_RELOAD_VALUE;
    xSemaphoreGive(tickSemaphore);
  }
}

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

  digitalWrite(LED_BUILTIN, fbVal); //HIGH

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

  IoInterfaceSetupPins();

  xTimerStart(tickTimer, 0);
  timer2Start();  


  while (1)
  {
    TreatmentMessage_t treatment;
    KeyEvent_t keyEvent;
    FbEvent_t fbEvent;

    /*wait for a tick*/
    xSemaphoreTake(tickSemaphore, portMAX_DELAY);

    Serial.println("TICK");

    if(pdTRUE == xQueueReceive(keyQueue, &keyEvent, 0))
    {
      Serial.print("Event received: key=");
      Serial.print(keyEvent.key, DEC);
      Serial.print(", act=");
      Serial.print(keyEvent.active, DEC);
      Serial.println(".");
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
      ret = treatmentBolusStart(tr->param1, tr->param2, tr->param3);
      break;
    case TREATMENT_BOLUS_STOP:
      ret = treatmentBolusStop(tr->param1, tr->param2, tr->param3);
      break;
    case TREATMENT_EXTENDED_BOLUS_START:
      ret = treatmentExtendedBolusStart(tr->param1, tr->param2, tr->param3);
      break;
    case TREATMENT_EXTENDED_BOLUS_STOP:
      ret = treatmentExtendedBolusStop(tr->param1, tr->param2, tr->param3);
      break;
    case TREATMENT_TEMPORARY_BASAL_RATE_START:
      break;
    case TREATMENT_TEMPORARY_BASAL_RATE_STOP:
      break;

    default:
      break;
  }
  /*inform keypad busy detector that we are done*/
  controlIsActive = false;

  return ret;
}

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

static bool treatmentBolusStart(uint16_t p1, uint16_t p2, uint16_t p3)
{
  /*Returns false in case of failure (i.e. user keypad activity or if feedback was negative).*/

  /*
  param1: amount (units multiplied with 100)
  param2: not used
  */
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
  uint16_t stepsForBolusAmount = p1 / 10;   /*each step is 0.10u. Requested=12.3u -> p1==1230 -> steps = 123*/
  for(uint16_t step = 0; step < stepsForBolusAmount; step++)
  {
    KEYPRESS_WITH_BUSY_CHECK(KEY_UP, PRESS_SHORT);
  }

  /*accept*/
  KEYPRESS_WITH_BUSY_CHECK(KEY_F3, PRESS_SHORT);

  /*confirm*/
  KEYPRESS_WITH_BUSY_CHECK(KEY_F2, PRESS_SHORT);

  /*we should get positive feedback*/  
  //waitForFeedback();

  return true;
}

static bool treatmentBolusStop(uint16_t p1, uint16_t p2, uint16_t p3)
{
  /*Returns false in case of failure (i.e. user keypad activity or if feedback was negative).*/
  (void)p1;
  (void)p2;
  (void)p3;

  return true;
}

static bool treatmentExtendedBolusStart(uint16_t p1, uint16_t p2, uint16_t p3)
{
  /*Returns false in case of failure (i.e. user keypad activity or if feedback was negative).*/

  /*
  param1: total amount (units multiplied with 100)
  param2: amount now (units multiplied with 100)
  param3: time in minutes
  */
 
  (void)p3;

  #if DEBUG_PRINT
  Serial.println(F("treatmentExtendedBolusStart"));
  #endif

  GO_TO_MENU_WITH_BUSY_CHECK();

  /*choose Bolus*/
  KEYPRESS_WITH_BUSY_CHECK(KEY_F3, PRESS_SHORT);

  /*get rid of BG input prompts*/
  KEYPRESS_WITH_BUSY_CHECK(KEY_F2, PRESS_SHORT);
  KEYPRESS_WITH_BUSY_CHECK(KEY_F2, PRESS_SHORT);

  /*input bolus amount*/
  uint16_t stepsForBolusAmount = p1 / 10;   /*each step is 0.10u. Requested=12.3u -> p1==1230 -> steps = 123*/
  for(uint16_t step = 0; step < stepsForBolusAmount; step++)
  {
    KEYPRESS_WITH_BUSY_CHECK(KEY_UP, PRESS_SHORT);
  }

  /*extend*/
  KEYPRESS_WITH_BUSY_CHECK(KEY_F2, PRESS_SHORT);

  /*input bolus amount*/
  uint16_t stepsForBolusNow = p2 / 10;   /*each step is 0.10u. Requested=12.3u -> p1==1230 -> steps = 123*/
  for(uint16_t step = 0; step < stepsForBolusNow; step++)
  {
    KEYPRESS_WITH_BUSY_CHECK(KEY_UP, PRESS_SHORT);
  }

  /*accept*/
  KEYPRESS_WITH_BUSY_CHECK(KEY_F3, PRESS_SHORT);

  /*confirm*/
  KEYPRESS_WITH_BUSY_CHECK(KEY_F2, PRESS_SHORT);
  
  //waitForFeedback();

  return true;
}

static bool treatmentExtendedBolusStop(uint16_t p1, uint16_t p2, uint16_t p3)
{
  /*Returns false in case of failure (i.e. user keypad activity or if feedback was negative).*/
  (void)p1;
  (void)p2;
  (void)p3;

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