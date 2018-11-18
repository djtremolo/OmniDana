#include "common.h"

#include "uartTask.h"
#include "ctrlTask.h"

static OmniDanaContext_t odContext;
static void initializePumpData(DanaRSPump_t *pump);


// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin(115200);

  setTime(0,0,0,1,1,2018);

  pinMode(LED_BUILTIN, OUTPUT);

  memset(&odContext, 0, sizeof(OmniDanaContext_t));

  initializePumpData(&(odContext.pump));

  odContext.commToCtrlBuffer = xMessageBufferCreate( COMM_TO_CTRL_BUFFER_SIZE );

  ctrlTaskInitialize(&odContext);
  uartTaskInitialize(&odContext);

}

void loop()
{
  // Empty. Things are done in Tasks.
}


static void initializePumpData(DanaRSPump_t *pump)
{
  pump->pairingRequested = false;
  pump->pass = 7493;
  pump->error = 0;
  pump->status = 0;
  pump->isExtendedInProgress = 0;
  pump->extendedBolusMinutes = 0;
  pump->extendedBolusAbsoluteRate = 0;
  pump->extendedBolusSoFarInMinutes = 0;
  pump->extendedBolusDeliveredSoFar = 0;
  pump->dailyTotalUnits = 26;
  pump->maxDailyTotalUnits = 90;
  pump->reservoirRemainingUnits = 70;
  pump->currentBasal = 2.9;
  pump->batteryRemaining = 89;
  pump->iob = 2.5;
  pump->bolusType = 0;
  pump->initialBolusAmount = 0;
  pump->lastBolusTimeHour = 0;
  pump->lastBolusTimeMinute = 0;
  pump->lastBolusAmount = 0;
  pump->maxBolus = 24;
  pump->bolusStep = 0.05;
  pump->isTempBasalInProgress = 0;
  pump->tempBasalPercent = 0;
  pump->tempBasalDurationHour = 0;    /*150==15min, 160==30min, otherwise hour*3600 */
  pump->tempBasalRunningMin = 0;
  pump->model = 1;
  pump->protocol = 2;
  pump->productCode = 3;
  pump->activeProfile = 0;
  pump->isExtendedBolusEnabled = 0;
  pump->bolusCalculationOption = 0;
  pump->missedBolusConfig = 0;
  pump->maxBasal = 4.0;
  pump->basalStep = 0.1;  /*float as u8*/
  pump->currentTarget = 5;
  pump->currentCIR = 20;
  pump->currentCF = 6;
  pump->units = UNITS_MMOL;
  pump->language = 0;
  
  /*
  pump->morningCIR = 20;
  pump->afternoonCIR = 20;
  pump->eveningCIR = 20;
  pump->nightCIR = 20;
  pump->morningCF = 6;
  pump->afternoonCF = 6;
  pump->eveningCF = 6;
  pump->nightCF = 6;
  */

  for (int i = 0; i < 10; i++)
  {
    uint8_t s[] = "1234567890";
    pump->serialNumber[i] = s[i];
  }

  for (int i = 0; i < 3; i++)
  {
    uint8_t s[] = {2018 - 1900, 11 - 1, 1};
    pump->shippingDate[i] = s[i];
  }
  for (int i = 0; i < 3; i++)
  {
    uint8_t s[] = "FIN";
    pump->shippingCountry[i] = s[i];
  }


}

