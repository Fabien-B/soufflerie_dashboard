#include "uss_handler.h"
#include "USS.h"
#include "hal.h"
#include "sdLog.h"
#include "stdutil++.hpp"
#include "sd.h"



void uss_msg_cb(USSDriver *ussp);

FileDes log_uss_fd;

USSConfig ussconf = {
    .uartp = &UARTD1,
    .gpt = &GPTD5,
    .speed = 115200,
    .node_nb = 0,       // TODO
    .standard_cb = NULL,
    .broadcast_cb = NULL,
    .special_cb = NULL,
    .any_cb = uss_msg_cb,
    .user_data = NULL,
};

static IN_DMA_SECTION(USSDriver ussd);


bool uss_log_opened = false;

void uss_msg_cb(USSDriver *ussp) {
    if(uss_log_opened) {
        sdLogWriteRaw(log_uss_fd, (uint8_t*)&ussp->rxTelegram, ussp->rxTelegram.lge+2);
        palToggleLine(LINE_LED1);
    }
}


msg_t startUSSLog() {
    if(uss_log_opened) {
        //already started
        return MSG_OK;
    }
    if(!startSdLog(chTimeMS2I(500))) {
        return MSG_RESET;
    }
    
    auto qsd = sdLogOpenLog(&log_uss_fd, "USS", "std", 1, false, 0, false);

    if(qsd != SDLOG_OK) {
        DebugTrace("SD fail to open USS logfile");
        return MSG_RESET;
    }

    uint8_t buf[] = {0x50, 0x4c, 0x4f, 0x50};
    sdLogWriteRaw(log_uss_fd, buf, 4);

    uss_log_opened = true;
    return MSG_OK;
}

void stopUSSLog() {
    uss_log_opened = false;
    sdLogCloseLog(log_uss_fd);
}

void startUSSListener() {
    ussStart(&ussd, &ussconf);
}

bool isLoggingUSS() {
  return uss_log_opened;
}
