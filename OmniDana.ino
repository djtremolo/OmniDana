#include "common.h"

#include "uartTask.h"
#include "ctrlTask.h"
#include "fbTask.h"

void BlinkLed(int times)
{
  while(times--)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    vTaskDelay( 250 / portTICK_PERIOD_MS );
    digitalWrite(LED_BUILTIN, LOW);
    vTaskDelay( 250 / portTICK_PERIOD_MS );
  }
}

/*INTERNAL DATA*/
//static DanaMessage_t commTaskMsg;
static DanaMessage_t uartTaskMsg;
static uint8_t uartTaskRawMsg[DANA_MAX_BUF_LEN];


static OmniDanaContext_t odContext;

// the setup function runs once when you press reset or power the board
void setup() {

  Serial.begin(115200);

  setTime(0,40,33,1,11,2018);

  pinMode(LED_BUILTIN, OUTPUT);

  memset(&odContext, 0, sizeof(OmniDanaContext_t));

  odContext.commToCtrlBuffer = xMessageBufferCreate( COMM_TO_CTRL_BUFFER_SIZE );
  odContext.fbToCtrlBuffer = xMessageBufferCreate( FB_TO_CTRL_BUFFER_SIZE );

  ctrlTaskInitialize(&odContext);
  fbTaskInitialize(&odContext);
  uartTaskInitialize(&odContext);

}

void loop()
{
  // Empty. Things are done in Tasks.
}
