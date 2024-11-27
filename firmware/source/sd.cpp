#include "sd.h"
#include "stdutil.h"
#include "sdLog.h"
#include "sdio.h"
#include "sensors.h"

bool sdlog_initialized = false;
thread_t* sdlog_watcher_thd = NULL;

thread_t* sensor_log_th_handle;
bool sensor_log_status = false;

FileDes log_data_fd;

static THD_WORKING_AREA(waSdLogWatcher, 2048);
void sdWatcherThd(void*) {
  chRegSetThreadName("SdWatcher");
  while(true) {
    if(chThdShouldTerminateX()) {
      chThdExit(SDLOG_OK);
    }
    msg_t ret = sdLogInit(0);
    if(ret == SDLOG_NOCARD) {
      // No SD card? Just wait a little and try again.
      chThdSleepMilliseconds(200);
      continue;
    }
    else if(ret == SDLOG_WAS_LAUNCHED) {
      // already launched? Close everything and try again.
      sdLogCloseAllLogs(false);
      continue;
    }
    else if(ret == SDLOG_FATFS_ERROR) {
      // FATFS error, probably unrecoverable.
      DebugTrace("sdLogInit: FATFS_ERROR");
      chThdExit(SDLOG_FATFS_ERROR);
    }
    else if(ret == SDLOG_OK) {
      // SDLog initialized!
      break;
    }
    
  }

  sdlog_initialized = true;
  DebugTrace("sdLogInit OK!");

  while (true)
  {
    if(chThdShouldTerminateX()) {
      sdlog_initialized = false;
      sdLogCloseAllLogs(true);
      chThdExit(SDLOG_OK);
    }
    chThdSleepMilliseconds(200);
  }
}

bool startSdLog(systime_t timeout) {
  if(sdlog_watcher_thd == NULL) {
    sdlog_watcher_thd = chThdCreateStatic(waSdLogWatcher, sizeof(waSdLogWatcher), NORMALPRIO + 1, sdWatcherThd, NULL);
  }
  
  systime_t start = chVTGetSystemTime();
  while(chVTTimeElapsedSinceX(start) < timeout && !sdlog_initialized){
    chThdSleepMilliseconds(5);
  }

  if(sdlog_initialized) {
    return true;
  }
  return false;
}

void stopSdLog() {
  if(sdlog_watcher_thd != NULL) {
    chThdTerminate(sdlog_watcher_thd);
    chThdWait(sdlog_watcher_thd);
    sdlog_watcher_thd = NULL;
  }
}

bool sdLogInitialized() {
  return sdlog_initialized;
}



static THD_WORKING_AREA(waSensorLog, 4096);
void sensorLogThd(void*) {
  chRegSetThreadName("SdLogger");

  while(!chThdShouldTerminateX()) {

    float tunnel_temp = getTunnelTemp();
    float temp = getTemp();
    float pressure = getAbsolutePressure();
    float diff_p = getDiffPressure();

    if(sdLogWriteLog(log_data_fd, "%f,%f,%f,%f\n", tunnel_temp, temp, diff_p, pressure) != SDLOG_OK) {
      sdLogCloseLog(log_data_fd);   // try to close log, but will probably fail
      sensor_log_status = false;
      return;
    }

    chThdSleepMilliseconds(500);    
  }

  sdLogCloseLog(log_data_fd);
  sensor_log_status = false;
}



msg_t startSensorLog() {
  if(sensor_log_status) {
    // already started
    return MSG_OK;
  }

  if(!startSdLog(chTimeMS2I(500))) {
    return MSG_TIMEOUT;
  }

  if(sdLogOpenLog(&log_data_fd, "test", "toto", 1, false, 0, false) != SDLOG_OK) {
    DebugTrace("SD fail to open logfile");
    return MSG_RESET;
  }

  sdLogWriteLog(log_data_fd,"tunnel_temp,temp,diff_p,pressure\n");
  sensor_log_status = true;

  sensor_log_th_handle = chThdCreateStatic(waSensorLog, sizeof(waSensorLog), NORMALPRIO + 1, sensorLogThd, NULL);
  return MSG_OK;
}

void stopSensorLog() {
  // if(sensor_log_th_handle != NULL) {
  //   chThdTerminate(sensor_log_th_handle);
  //   sensor_log_status = false;
  //   chThdWait(sensor_log_th_handle);
  // }
  if(sensor_log_status) {
    sensor_log_status = false;
    chThdTerminate(sensor_log_th_handle);
    chThdWait(sensor_log_th_handle);
    sensor_log_th_handle = NULL;
  }
}

bool isLoggingSensors() {
  return sensor_log_status;
}
