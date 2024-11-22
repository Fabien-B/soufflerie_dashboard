#include "sd.h"
#include "stdutil.h"
#include "sdLog.h"
#include "sdio.h"
#include "sensors.h"

thread_t* log_th_handle;
bool log_status = false;

FileDes log_data_fd;

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

  if(sdLogOpenLog(&log_data_fd, "test", "toto", 1, false, 0, false) != SDLOG_OK) {
    DebugTrace("SD fail to open logfile");
    return;
  }

  sdLogWriteLog(log_data_fd,"tunnel_temp,temp,diff_p,pressure\n");
  log_status = true;

  while(!chThdShouldTerminateX()) {

    float tunnel_temp = getTunnelTemp();
    float temp = getTemp();
    float pressure = getAbsolutePressure();
    float diff_p = getDiffPressure();

    if(sdLogWriteLog(log_data_fd, "%f,%f,%f,%f\n", tunnel_temp, temp, diff_p, pressure) != SDLOG_OK) {
      sdLogCloseLog(log_data_fd);   // try to close log, but will probably fail
      log_status = false;
      return;
    }

    chThdSleepMilliseconds(500);    
  }

  sdLogCloseLog(log_data_fd);
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
