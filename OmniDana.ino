#include "common.h"

#include "uartTask.h"
#include "ctrlTask.h"

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



static OmniDanaContext_t odContext;

// the setup function runs once when you press reset or power the board
void setup() {






  Serial.begin(115200);

  setTime(0,40,33,1,11,2018);

  pinMode(LED_BUILTIN, OUTPUT);

  memset(&odContext, 0, sizeof(OmniDanaContext_t));

  odContext.commToCtrlBuffer = xMessageBufferCreate( COMM_TO_CTRL_BUFFER_SIZE );

  ctrlTaskInitialize(&odContext);
  uartTaskInitialize(&odContext);

}

void loop()
{
  // Empty. Things are done in Tasks.
}
