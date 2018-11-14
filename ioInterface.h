#ifndef __IOINTERFACE_H__
#define __IOINTERFACE_H__

typedef enum
{
  KEY_F1 = 0,
  KEY_F2,
  KEY_F3,
  KEY_HOME,
  KEY_UP,
  KEY_DOWN,
  KEY_QUESTIONMARK,
  /****/
  KEY_MAX,
  KEY_NONE /*used for checking if ctrlTask is actively pushing a button or not*/
} buttonKey_t;

typedef enum
{
  FB_BEEP = 0,
  /****/
  FB_MAX
} feedbackKey_t;

#define PIN_D2                  2       //interrupt capable
#define PIN_D3                  3       //interrupt capable
#define PIN_D4                  4
#define PIN_D5                  5

#define IOMODE_NORMALLY_UP      1
#define IOMODE_NORMALLY_DOWN    2

#define KEY_IO_PULLUP           true
#define KEY_IO_MODE             IOMODE_NORMALLY_UP
#define FEEDBACK_IO_MODE        IOMODE_NORMALLY_UP

#define KEY_HIZ_MODE            true

typedef enum
{
  PIN_STATE_DRIVE_LOW = false,
  PIN_STATE_DRIVE_HIGH = true,
  PIN_STATE_TRISTATE = 3
} pinState_t;

const uint8_t GPIOKeyMapping[KEY_MAX] =
{
  PIN_D3, //KEY_F1
  PIN_D4, //KEY_F2
  PIN_D5, //KEY_F3
  PIN_A0, //KEY_HOME
  PIN_A1, //KEY_UP
  PIN_A2, //KEY_DOWN
  PIN_A3  //KEY_QUESTIONMARK
};

const uint8_t GPIOFeedbackMapping[FB_MAX] =
{
  PIN_D2 //FB_BEEP
};

void IoInterfaceSetupPins(void (*fbFun)(void));
bool IoInterfaceReadFeedbackPin(feedbackKey_t fb);
void IoInterfaceSetButtonPin(buttonKey_t key, bool value);

#endif //__IOINTERFACE_H__