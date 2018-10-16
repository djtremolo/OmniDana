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
  /****/
  KEY_MAX
} buttonKey_t;

typedef enum 
{
  FB_BEEP = 0,
  FB_BACKLIGHT,
  /****/
  FB_MAX
} feedbackKey_t;

void IoInterfaceSetupPins();
bool IoInterfaceReadFeedbackPin(feedbackKey_t fb);
void IoInterfaceSetButtonPin(buttonKey_t key, bool value);

#endif  //__IOINTERFACE_H__