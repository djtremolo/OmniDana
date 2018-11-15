#include "ioInterface.h"

static void setupFeedbackPin(feedbackKey_t fb);
static void setupButtonPin(buttonKey_t key);
static void setPinState(uint8_t ioPin, pinState_t state);

bool IoInterfaceReadFeedbackPin(feedbackKey_t fb);
void IoInterfaceSetButtonPin(buttonKey_t key, bool value);
void IoInterfaceSetupPins(void (*fbFun)(void));

void IoInterfaceSetupPins()
{
  uint8_t i;

  for (i = 0; i < KEY_MAX; i++)
  {
    setupButtonPin((buttonKey_t)i);
  }

  for (i = 0; i < FB_MAX; i++)
  {
    setupFeedbackPin((feedbackKey_t)i);
  }


}

static void setupFeedbackPin(feedbackKey_t fb)
{
  if (fb >= FB_MAX)
    return;

  uint8_t ioPin = GPIOFeedbackMapping[fb];
  pinMode(ioPin, INPUT_PULLUP);
}

bool IoInterfaceReadFeedbackPin(feedbackKey_t fb)
{
  if (fb >= FB_MAX)
    return false;

  uint8_t ioPin = GPIOFeedbackMapping[fb];
  bool state;

#if FEEDBACK_IO_MODE == IOMODE_NORMALLY_DOWN
  state = (bool)digitalRead(ioPin);
#else
  state = !((bool)digitalRead(ioPin));
#endif

  return state;
}


static bool getPinState(uint8_t ioPin)
{
#if KEY_IO_MODE == IOMODE_NORMALLY_DOWN
  return (digitalRead(ioPin) == 0 ? false : true);
#else
  return (digitalRead(ioPin) == 0 ? true : false);
#endif
}


static void setPinState(uint8_t ioPin, pinState_t state)
{
  vTaskSuspendAll();
  switch (state)
  {
  case PIN_STATE_DRIVE_LOW:
    digitalWrite(ioPin, LOW);
    pinMode(ioPin, OUTPUT);
    break;
  case PIN_STATE_DRIVE_HIGH:
    digitalWrite(ioPin, HIGH);
    pinMode(ioPin, OUTPUT);
    break;
  default:
    digitalWrite(ioPin, LOW); /*drive disabled*/
  #if KEY_IO_PULLUP
    pinMode(ioPin, INPUT_PULLUP);    /*set to input with pull up*/
  #else
    pinMode(ioPin, INPUT);    /*set to input to enable HI-Z mode*/
  #endif
    break;
  }
  xTaskResumeAll();
}

static void setupButtonPin(buttonKey_t key)
{
  if (key >= KEY_MAX)
    return;

  uint8_t ioPin = GPIOKeyMapping[key];

#if KEY_HIZ_MODE
  setPinState(ioPin, PIN_STATE_TRISTATE);
#else
#if KEY_IO_MODE == IOMODE_NORMALLY_DOWN
  setPinState(ioPin, PIN_STATE_DRIVE_LOW);
#else
  setPinState(ioPin, PIN_STATE_DRIVE_HIGH);
#endif
#endif
}

bool IoInterfaceReadButtonPin(buttonKey_t key)
{
  if (key >= KEY_MAX)
    return false;

  return getPinState(GPIOKeyMapping[key]);
}


void IoInterfaceSetButtonPin(buttonKey_t key, bool value)
{
  if (key >= KEY_MAX)
    return;

  uint8_t ioPin = GPIOKeyMapping[key];

  if (value)
  {
/*go to active state: push the button*/
#if KEY_IO_MODE == IOMODE_NORMALLY_DOWN
    setPinState(ioPin, PIN_STATE_DRIVE_HIGH);
#else
    setPinState(ioPin, PIN_STATE_DRIVE_LOW);
#endif
  }
  else
  {
/*go to inactive state: release button*/
#if KEY_HIZ_MODE
    setPinState(ioPin, PIN_STATE_TRISTATE);
#else
#if KEY_IO_MODE == IOMODE_NORMALLY_DOWN
    setPinState(ioPin, PIN_STATE_DRIVE_LOW);
#else
    setPinState(ioPin, PIN_STATE_DRIVE_HIGH);
#endif
#endif
  }
}
