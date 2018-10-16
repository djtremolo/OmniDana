#include "OmniDanaCommon.h"

#include "recTask.h"
#include "ctrlTask.h"
#include "fbTask.h"
#include "crc16.h"

/*MACROS*/
#define BT_TO_CONTROL_BUFFER_SIZE 101


/*INTERNAL DATA*/
MessageBufferHandle_t BtToControlBuffer;

// the setup function runs once when you press reset or power the board
void setup() {

  Serial.begin(115200);

  BtToControlBuffer = xMessageBufferCreate( BT_TO_CONTROL_BUFFER_SIZE );

 // recTaskInitialize(BtToControlBuffer, 2);
  ctrlTaskInitialize(BtToControlBuffer, 3);
}

void loop()
{
  // Empty. Things are done in Tasks.
}
