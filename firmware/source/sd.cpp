#include "sd.h"
#include "stdutil.h"
#include "sdLog.h"
#include "sdio.h"
#include "sensors.h"

thread_t* log_th_handle;
bool log_status = false;

static THD_WORKING_AREA(waLog, 10000);
void logThd(void*) {
  chRegSetThreadName("SdLogger");

  while (!isCardInserted())
  {
    // wait for a card to be inserted
    chThdSleepMilliseconds(200);
    if(chThdShouldTerminateX()) {
      return;
    }
  }

  if(sdLogInit(NULL) != SDLOG_OK) {
    DebugTrace("SD fail to connect!");
    return;
  }

  if(sdLogOpenLog(0, "test", "toto", 1, false, 0, false) != SDLOG_OK) {
    DebugTrace("SD fail to open logfile");
    return;
  }

  sdLogWriteLog(0,"tunnel_temp,temp,diff_p,pressure\n");
  log_status = true;

  while(!chThdShouldTerminateX()) {

    float tunnel_temp = getTunnelTemp();
    float temp = getTemp();
    float pressure = getAbsolutePressure();
    float diff_p = getDiffPressure();

    if(sdLogWriteLog(0, "%f,%f,%f,%f\n", tunnel_temp, temp, diff_p, pressure) != SDLOG_OK) {
      sdLogCloseLog(0);   // try to close log, but will probably fail
      log_status = false;
      return;
    }

    chThdSleepMilliseconds(500);    
  }

  sdLogCloseLog(0);
  log_status = false;

}



void startLog() {
  log_th_handle = chThdCreateStatic(waLog, sizeof(waLog), NORMALPRIO + 1, logThd, NULL);
}

void stopLog() {
  chThdTerminate(log_th_handle);
}

bool isLogging() {
  return log_status;
}
