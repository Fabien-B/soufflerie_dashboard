#include "uss_logger.h"
#include "USS.h"
#include "hal.h"
#include "sdLog.h"
#include "stdutil++.hpp"
#include "sdio.h"



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


void uss_msg_cb(USSDriver *ussp) {
    sdLogWriteRaw(log_uss_fd, (uint8_t*)&ussp->rxTelegram, ussp->rxTelegram.lge+2);
    palToggleLine(LINE_LED1);
}


msg_t startUSSLog() {
    if(!isCardInserted()) {
        return MSG_RESET;
    }

    if(sdLogOpenLog(&log_uss_fd, "USS", "std", 1, false, 0, false) != SDLOG_OK) {
        DebugTrace("SD fail to open USS logfile");
        return MSG_RESET;
    }
    
    ussStart(&ussd, &ussconf);
    return MSG_OK;
}

void stopUSSLog() {
    sdLogCloseLog(log_uss_fd);
}