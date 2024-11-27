#pragma once
#include "hal.h"


bool startSdLog(systime_t timeout);
void stopSdLog();
bool sdLogInitialized();


msg_t startSensorLog();
void stopSensorLog();
bool isLoggingSensors();


