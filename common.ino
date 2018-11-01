#include "common.h"


time_t mySoftRTC;

void clockUpdate()
{
    mySoftRTC = now();
}