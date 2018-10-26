#include "common.h"

#include "uartTask.h"
#include "ctrlTask.h"
#include "fbTask.h"
#include "commTask.h"



/*INTERNAL DATA*/
OmniDanaContext_t odContext;

// the setup function runs once when you press reset or power the board
void setup() {

  Serial.begin(115200);

  pinMode(LED_BUILTIN, OUTPUT);

  memset(&odContext, 0, sizeof(OmniDanaContext_t));

  odContext.commToRecBuffer = xMessageBufferCreate( COMM_TO_REC_BUFFER_SIZE );
  odContext.recToCommBuffer = xMessageBufferCreate( REC_TO_COMM_BUFFER_SIZE );
  odContext.commToCtrlBuffer = xMessageBufferCreate( COMM_TO_CTRL_BUFFER_SIZE );
  odContext.fbToCtrlBuffer = xMessageBufferCreate( FB_TO_CTRL_BUFFER_SIZE );

 // commTaskInitialize(&odContext);
 // ctrlTaskInitialize(&odContext);
 // fbTaskInitialize(&odContext);
  uartTaskInitialize(&odContext);
}

void loop()
{
  // Empty. Things are done in Tasks.
}
