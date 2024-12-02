#include "uss_handler.h"
#include "USS.h"
#include "hal.h"
#include "sdLog.h"
#include "stdutil++.hpp"
#include "sd.h"
#include "ch.h"
#include "string.h"

#define TLGM_NB 10
Telegram_t tlgm_buffer[TLGM_NB];
msg_t free_tlgm_queue[TLGM_NB];
MAILBOX_DECL(mb_free_tlgms, free_tlgm_queue, TLGM_NB);
msg_t filled_tlgm_queue[TLGM_NB];
MAILBOX_DECL(mb_filled_tlgms, filled_tlgm_queue, TLGM_NB);

static void init_queue() {
    chMBReset(&mb_filled_tlgms);
    chMBResumeX(&mb_filled_tlgms);
    chMBReset(&mb_free_tlgms);
    chMBResumeX(&mb_free_tlgms);
    // Pre-filling the free buffers pool with the available buffers, the post
    // will not stop because the mailbox is large enough.
    for(int i=0; i<TLGM_NB; i++) {
        chMBPostTimeout(&mb_free_tlgms, (msg_t)&tlgm_buffer[i], 0);
    }
}


void uss_msg_cb(USSDriver *ussp);

FileDes log_uss_fd;

USSConfig ussconf = {
    .uartp = &UARTD1,
    .rs485_en_line = LINE_TX485EN,
    .gpt = &GPTD5,
    .speed = 115200,
    .node_nb = 0,       // TODO
    .standard_cb = NULL,
    .broadcast_cb = NULL,
    .special_cb = NULL,
    .any_cb = uss_msg_cb,
    .silent = false,
    .user_data = NULL,
};

static IN_DMA_SECTION(USSDriver ussd);


bool uss_log_opened = false;

void uss_msg_cb(USSDriver *ussp) {
    Telegram_t* tlgm;
    // get a free telegram
    chSysLockFromISR();
    msg_t ret = chMBFetchI(&mb_free_tlgms, (msg_t*)&tlgm);
    if(ret == MSG_OK) {
        // copy data
        memcpy(&tlgm, &ussp->rxTelegram, ussp->rxTelegram.lge+2);
        // post it to the filled queue
        chMBPostI(&mb_filled_tlgms, (msg_t)&tlgm);
    }
    chSysUnlockFromISR();
}


THD_WORKING_AREA(waUSSLogger, 1024);
void uss_log(void*) {
    chRegSetThreadName("USS logger");
    while(!chThdShouldTerminateX()) {
        Telegram_t* tlgm;
        // get a filled telegram
        msg_t ret = chMBFetchTimeout(&mb_filled_tlgms, (msg_t*)&tlgm, chTimeMS2I(100));
        if(ret == MSG_OK && uss_log_opened) {
            sdLogWriteRaw(log_uss_fd, (uint8_t*)tlgm, tlgm->lge+2);
            // post the buffer back to free telegrams
            chMBPostTimeout(&mb_free_tlgms, (msg_t)tlgm, TIME_IMMEDIATE);
        }
    }
}

thread_t* uss_log_thd = NULL;

msg_t startUSSLog() {
    init_queue();
    if(uss_log_opened) {
        //already started
        return MSG_OK;
    }
    if(!startSdLog(chTimeMS2I(500))) {
        return MSG_RESET;
    }

    if(sdLogOpenLog(&log_uss_fd, "USS", "std", 1, false, 0, false) != SDLOG_OK) {
        DebugTrace("SD fail to open USS logfile");
        return MSG_RESET;
    }
    uss_log_opened = true;

    uss_log_thd = chThdCreateStatic(waUSSLogger, sizeof(waUSSLogger), NORMALPRIO-1, uss_log, NULL);

    return MSG_OK;
}

void stopUSSLog() {
    uss_log_opened = false;
    sdLogCloseLog(log_uss_fd);
    if(uss_log_thd) {
        chThdTerminate(uss_log_thd);
        chThdWait(uss_log_thd);
        uss_log_thd = NULL;
    }
}

void startUSSListener() {
    ussStart(&ussd, &ussconf);
}

bool isLoggingUSS() {
  return uss_log_opened;
}
