#include "ioInterface.h"

#define PIN_D2                  2
#define PIN_D3                  3
#define PIN_D4                  4
#define PIN_D5                  5

#define PIN_A0                  14
#define PIN_A1                  15
#define PIN_A2                  16
#define PIN_A3                  17

#define IOMODE_NORMALLY_UP      1
#define IOMODE_NORMALLY_DOWN    2

#define KEY_IO_MODE             IOMODE_NORMALLY_DOWN
#define FEEDBACK_IO_MODE        IOMODE_NORMALLY_DOWN

#define KEY_HIZ_MODE            true

const uint8_t GPIOKeyMapping[KEY_MAX] = 
{
  PIN_D2,   //KEY_F1
  PIN_D3,   //KEY_F2
  PIN_D4,   //KEY_F3
  PIN_D5,   //KEY_HOME
  PIN_A0,   //KEY_UP
  PIN_A1    //KEY_DOWN
};

const uint8_t GPIOFeedbackMapping[FB_MAX] = 
{
  PIN_A2,   //FB_BEEP
  PIN_A3    //FB_BACKLIGHT
};

static void setupFeedbackPin(feedbackKey_t fb);
static void setupButtonPin(buttonKey_t key);
static void setPinTriState(uint8_t ioPin);
static void setPinHigh(uint8_t ioPin);
static void setPinLow(uint8_t ioPin);

bool IoInterfaceReadFeedbackPin(feedbackKey_t fb);
void IoInterfaceSetButtonPin(buttonKey_t key, bool value);
void IoInterfaceSetupPins();

void IoInterfaceSetupPins()
{
  uint8_t i;

  for(i = 0; i<KEY_MAX; i++)
  {
    setupButtonPin((buttonKey_t)i);
  }

  for(i = 0; i<FB_MAX; i++)
  {
    setupFeedbackPin((feedbackKey_t)i);
  }
}


static void setupFeedbackPin(feedbackKey_t fb)
{
  if(fb >= FB_MAX)
    return;

  uint8_t ioPin = GPIOFeedbackMapping[fb];

  pinMode(ioPin, INPUT);
}

bool IoInterfaceReadFeedbackPin(feedbackKey_t fb)
{
  if(fb >= FB_MAX)
    return;

  uint8_t ioPin = GPIOFeedbackMapping[fb];
  bool state;

  #if FEEDBACK_IO_MODE == IOMODE_NORMALLY_DOWN
    state = (bool)digitalRead(ioPin);
  #else
    state = !((bool)digitalRead(ioPin));
  #endif

  return state;
}

static void setPinHigh(uint8_t ioPin)
{
  vTaskSuspendAll();
  digitalWrite(ioPin, HIGH); /*set state / disable pull-up, depending on the pin mode*/
  pinMode(ioPin, OUTPUT);  /*force low by default*/
  xTaskResumeAll();
}
static void setPinLow(uint8_t ioPin)
{
  vTaskSuspendAll();
  digitalWrite(ioPin, LOW); /*set state / disable pull-up, depending on the pin mode*/
  pinMode(ioPin, OUTPUT);  /*force low by default*/  
  xTaskResumeAll();
}
static void setPinTriState(uint8_t ioPin)
{
  vTaskSuspendAll();
  digitalWrite(ioPin, LOW); /*make sure driving and pull-up are disabled*/
  pinMode(ioPin, INPUT);    /*set to input to enable HI-Z mode*/
  xTaskResumeAll();
}

static void setupButtonPin(buttonKey_t key)
{
  if(key >= KEY_MAX)
    return;

  uint8_t ioPin = GPIOKeyMapping[key];

  #if KEY_HIZ_MODE
    setPinTriState(ioPin);
  #else
    #if KEY_IO_MODE == IOMODE_NORMALLY_DOWN
    setPinLow(ioPin);
    #else
    setPinHigh(ioPin);
    #endif
  #endif

  // The operation is complete.  Restart the RTOS kernel.
  xTaskResumeAll();
}

void IoInterfaceSetButtonPin(buttonKey_t key, bool value)
{
  if(key >= KEY_MAX)
    return;

  uint8_t ioPin = GPIOKeyMapping[key];

  switch(value)
  {
    case false:   /*normal / nonactive state*/
      #if KEY_HIZ_MODE
        setPinTriState(ioPin);
      #else
        #if KEY_IO_MODE == IOMODE_NORMALLY_DOWN
        setPinLow(ioPin);
        #else
        setPinHigh(ioPin);
        #endif
      #endif
      break;

    default:
      #if KEY_HIZ_MODE
        setPinTriState(ioPin);
      #else
        #if KEY_IO_MODE == IOMODE_NORMALLY_DOWN
        setPinHigh(ioPin);
        #else
        setPinLow(ioPin);
        #endif
      #endif
      break;
  }
}





