#include <Arduino_FreeRTOS.h>
#include "message_buffer.h"
#include "OmniDanaCommon.h"

#include "BTReceiverTask.h"
#include "ControlTask.h"
#include "FeedbackTask.h"
#include "crc16.h"

/*MACROS*/
#define BT_TO_CONTROL_BUFFER_SIZE 101


/*INTERNAL DATA*/
MessageBufferHandle_t BtToControlBuffer;

// the setup function runs once when you press reset or power the board
void setup() {

  Serial.begin(115200);

  BtToControlBuffer = xMessageBufferCreate( BT_TO_CONTROL_BUFFER_SIZE );

  Serial.print("setup: BtToControlBuffer= ");
  Serial.print((unsigned int)BtToControlBuffer, HEX);
  Serial.println(".");


  // Now set up two tasks to run independently.


  BTReceiverInitialize(BtToControlBuffer);
  ControlInitialize(BtToControlBuffer);
}

void loop()
{
  // Empty. Things are done in Tasks.
}


void TaskReceiver(void *pvParameters)  // This is a task.
{
  MessageBufferHandle_t mb = (MessageBufferHandle_t)pvParameters;
  uint8_t ucRxData[ 20 ];
  size_t xReceivedBytes;
  const TickType_t xBlockTime = pdMS_TO_TICKS( 100 );
  int i;


  Serial.print("TaskB: xMessageBuffer= ");
  Serial.print((unsigned int)mb, HEX);
  Serial.println(".");


  while (1)
  {
    /* Receive the next message from the message buffer.  Wait in the Blocked
      state (so not using any CPU processing time) for a maximum of 100ms for
      a message to become available. */

    //  Serial.println("TaskReceiver:waiting");
    xReceivedBytes = xMessageBufferReceive( mb,
                                            ( void * ) ucRxData,
                                            sizeof( ucRxData ),
                                            portMAX_DELAY  );

    //    Serial.println("TaskReceiver:timeout exceeded");
    if ( xReceivedBytes > 0 )
    {
      Serial.print("Received ");
      Serial.print(xReceivedBytes, DEC);
      Serial.print("bytes: ");

      for(i=0; i<xReceivedBytes; i++)
      {
        Serial.print(ucRxData[i], HEX);
      }

      Serial.println(".");

      /* A ucRxData contains a message that is xReceivedBytes long.  Process
        the message here.... */
    }

//     vTaskDelay( 100000 / portTICK_PERIOD_MS ); // wait for one second


  }
}
