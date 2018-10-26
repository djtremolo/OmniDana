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
  KEY_NONE  /*used for checking if ctrlTask is actively pushing a button or not*/
} buttonKey_t;

typedef enum 
{
  FB_BEEP = 0,
  /****/
  FB_MAX
} feedbackKey_t;

void IoInterfaceSetupPins();
bool IoInterfaceReadFeedbackPin(feedbackKey_t fb);
void IoInterfaceSetButtonPin(buttonKey_t key, bool value);

#endif  //__IOINTERFACE_H__