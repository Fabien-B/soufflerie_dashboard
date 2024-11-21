#include "USS.h"
#include "hal.h"
#include "string.h"

// 8 data bits, even parity, 1 stop bit, LSB first (standard)
// time between 2 characters: less than 2x character time (22bits)
// STX: 0x02
// start interval: at least 2 character run times (2*11/baudrate)
// Response delay time: in the [start interval; 20ms] range

// Telegram structure
// STX | LGE | ADR | 1..n | BCC
// LGE: 1byte, telegram length, TX and LGE excluded.  LGE = n + 2

// maximum telegram residual time = 1.5*(n+3)*character_time


static void char_received(UARTDriver *uartp, uint16_t c);
static void telegram_received(UARTDriver *uartp);
static void error_cb(UARTDriver *uartp, uartflags_t e);
static void telegram_sent(UARTDriver *uartp);

static void residual_timeout_cb(GPTDriver *gptp);

UARTConfig rs485_conf = {
  .txend1_cb = NULL,                // End of transmission buffer callback.
  .txend2_cb = telegram_sent,       //Physical end of transmission callback.
  .rxend_cb = telegram_received,    //Receive buffer filled callback.
  .rxchar_cb = char_received,       //Character received while out if the @p UART_RECEIVE state.
  .rxerr_cb = error_cb,             //Receive error callback.

  /* End of the mandatory fields.*/

  .timeout_cb=NULL, //Receiver timeout callback.
  /**
   * @brief   Receiver timeout value in terms of number of bit duration.
   * @details Set it to 0 when you want to handle idle interrupt instead of
   *          hardware timeout.
   */
  .timeout = 0,

  .speed = 115200,
  .cr1 = USART_CR1_PCE,
  .cr2 = USART_CR2_STOP1_BITS | USART_CR2_LINEN,
  .cr3 = 0,
};

GPTConfig gptconf = {
    .frequency=115200,
    .callback = residual_timeout_cb,
    .cr2 = 0,
    .dier = 0,
};


/**
 *  wait response delay time before sending the telegram in the txBuffer
 */
static void txThd(void* arg) {
    chRegSetThreadName("USS tx thread");
    USSDriver* ussp = (USSDriver*)arg;
    // 2 character time, rounded up.
    uint32_t response_delay = 2*11*1000000/ussp->config->speed + 1;
    while (!chThdShouldTerminateX())
    {
        chBSemWait(&ussp->tx_sem);
        chThdSleepMicroseconds(response_delay);
        uint8_t lge = ussp->txBuffer[1];
        uartStartSend(ussp->config->uartp, lge+2, ussp->txBuffer);
        ussp->txState = USS_TX_SENDING;
    }
}

/**
 * Schedule a transmission of the telegram currently in the txBuffer.
 * Do not overwrite txBuffer before end of transmission
 */
static void sendTelegram(USSDriver* ussp) {
    ussp->txState = USS_TX_RSP_WAIT;
    chSysLockFromISR();
    chBSemSignalI(&ussp->tx_sem);
    chSysUnlockFromISR();
}


static void error_cb(UARTDriver *uartp, uartflags_t e) {
    (void)e;
    USSDriver* ussp = (USSDriver*)uartp->ussp;
    // stop receiving current telegram, if any
    ussp->rxState = USS_RX_STX;
    uartStopReceive(uartp);
}


static void char_received(UARTDriver *uartp, uint16_t c) {
    USSDriver* ussp = (USSDriver*)uartp->ussp;

    switch (ussp->rxState)
    {
    case USS_RX_STX:
        if(c == USS_STX) {   // start of telegram !
            ussp->rxState = USS_RX_LGE;
        }
    break;
    case USS_RX_LGE:
    {
        ussp->lge = c;
        // setup DMA to receive telegram in ussp->buffer
        chSysLockFromISR();
        uartStartReceiveI(uartp, ussp->lge, &ussp->rxBuffer);
        uint16_t residual_time = 1.5*(ussp->lge)*11;
        gptStartOneShotI(ussp->gpt, residual_time);
        chSysUnlockFromISR();
        ussp->rxState = USS_RX_RESIDUAL;
    }
    break;
    case USS_RX_RESIDUAL:
        // should no happens (I suppose)
        //chSysHalt("USS received byte while in residual");
    break;
    
    default:
        break;
    }
}

uint8_t getBCC(USSDriver* ussp) {
    return ussp->rxBuffer[ussp->lge-1];
}

uint8_t computeBCC(USSDriver* ussp) {
    uint8_t bcc = 0;
    bcc ^= USS_STX;
    bcc ^= ussp->lge;
    for(uint8_t i=0; i<ussp->lge-1; i++) {
        bcc ^= ussp->rxBuffer[i];
    }
    return bcc;
}

static void telegram_received(UARTDriver *uartp) {
    USSDriver* ussp = (USSDriver*)uartp->ussp;
    // cancel the residual time timeout
    chSysLockFromISR();
    gptStopTimerI(ussp->gpt);
    chSysUnlockFromISR();
    

    if(getBCC(ussp) != computeBCC(ussp)) {
        ussp->status = USS_BCC_MISMATCH;
        return;
    }
    ussp->adr = ussp->rxBuffer[0];

    // special telegram
    if(ussp->adr & 0x80) {
        if(ussp->config->special_cb) {
            ussp->config->special_cb(ussp);
        }
        return;
    }

    // mirror telegram: The node number is evaluated and the
    // addressed slave returns the telegram, unchanged, to the master
    if((ussp->adr & 0x40) && ((ussp->adr & 0x1F) == ussp->config->node_nb)) {
        ussp->txBuffer[0] = USS_STX;
        ussp->txBuffer[1] = ussp->lge;
        memcpy(ussp->txBuffer+2, ussp->rxBuffer, ussp->lge);
        sendTelegram(ussp);
        return;
    }

    // broadcast telegram. Node number not evaluated
    if(ussp->adr & 0x20) {
        if(ussp->config->broadcast_cb) {
            ussp->config->broadcast_cb(ussp);
        }
        return;
    }

    // standard data transfer. Node number is evaluated
    if((ussp->adr & 0xE0) == 0 && ((ussp->adr & 0x1F) == ussp->config->node_nb)) {
        if(ussp->config->standard_cb) {
            ussp->config->standard_cb(ussp);
        }
        return;
    }
}

/**
 * telegram residual time expired
 */
static void residual_timeout_cb(GPTDriver *gptp) {
    USSDriver* ussp = (USSDriver*)gptp->ussp;
    ussp->rxState = USS_RX_STX;
}


static void telegram_sent(UARTDriver *uartp) {
    USSDriver* ussp = (USSDriver*)uartp->ussp;
    ussp->txState = USS_TX_IDLE;
}

void ussStart(USSDriver *ussp, USSConfig *usscfg)
{
    ussp->config = usscfg;
    usscfg->uartp->ussp = ussp;
    ussp->rxState = USS_RX_STX;
    ussp->txState = USS_TX_IDLE;
    ussp->status = USS_OK;
    chBSemObjectInit(&ussp->tx_sem, true);
    rs485_conf.speed = usscfg->speed;
    gptconf.frequency = usscfg->speed;
    chThdCreateStatic(ussp->waTxThread, sizeof(ussp->waTxThread), NORMALPRIO + 1, txThd, ussp);
    uartStart(ussp->config->uartp, &rs485_conf);
    gptStart(ussp->gpt, &gptconf);

}
