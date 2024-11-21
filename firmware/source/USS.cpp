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

// A task telegram is the transfer of a complete net data block from the master to the slave.
// A response telegram is the transfer of the complete net data block from the slave to the master


static void char_received(UARTDriver *uartp, uint16_t c);
static void telegram_received(UARTDriver *uartp);
static void error_cb(UARTDriver *uartp, uartflags_t e);
static void telegram_sent(UARTDriver *uartp);
static void residual_timeout_cb(GPTDriver *gptp);


/***
 *      _____    _                                  ____  __  __
 *     |_   _|__| | ___  __ _ _ __ __ _ _ __ ___   |  _ \ \ \/ /
 *       | |/ _ \ |/ _ \/ _` | '__/ _` | '_ ` _ \  | |_) | \  / 
 *       | |  __/ |  __/ (_| | | | (_| | | | | | | |  _ <  /  \ 
 *       |_|\___|_|\___|\__, |_|  \__,_|_| |_| |_| |_| \_\/_/\_\
 *                      |___/                                   
 */


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
        uartStartSend(ussp->config->uartp, ussp->txTelegram.lge+2, (uint8_t*)&ussp->txTelegram);
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
        if(c == USS_STX) {   // start of telegram
            ussp->rxTelegram.stx = c;
            ussp->rxState = USS_RX_LGE;
        }
    break;
    case USS_RX_LGE:
    {
        ussp->rxTelegram.lge = c;
        // setup DMA to receive telegram in ussp->buffer
        chSysLockFromISR();
        uartStartReceiveI(uartp, ussp->rxTelegram.lge, (uint8_t*)&ussp->rxTelegram.adr);
        uint16_t residual_time = 1.5*ussp->rxTelegram.lge*11;
        gptStartOneShotI(ussp->config->gpt, residual_time);
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
    return ussp->rxTelegram.data[ussp->rxTelegram.lge-2];
}

uint8_t computeBCC(USSDriver* ussp) {
    uint8_t bcc = 0;
    for(uint8_t i=0; i<ussp->rxTelegram.lge+2; i++) {
        bcc ^= ((uint8_t*)&ussp->rxTelegram)[i];
    }
    return bcc;
}

static void telegram_received(UARTDriver *uartp) {
    USSDriver* ussp = (USSDriver*)uartp->ussp;
    // cancel the residual time timeout
    chSysLockFromISR();
    gptStopTimerI(ussp->config->gpt);
    chSysUnlockFromISR();
    

    if(getBCC(ussp) != computeBCC(ussp)) {
        ussp->status = USS_BCC_MISMATCH;
        return;
    }

    // special telegram
    if(ussp->rxTelegram.adr & 0x80) {
        if(ussp->config->special_cb) {
            ussp->config->special_cb(ussp);
        }
        return;
    }

    // mirror telegram: The node number is evaluated and the
    // addressed slave returns the telegram, unchanged, to the master
    if((ussp->rxTelegram.adr & 0x40) && ((ussp->rxTelegram.adr & 0x1F) == ussp->config->node_nb)) {
        memcpy(&ussp->txTelegram, &ussp->rxTelegram, ussp->rxTelegram.lge+2);
        sendTelegram(ussp);
        return;
    }

    // broadcast telegram. Node number not evaluated
    if(ussp->rxTelegram.adr & 0x20) {
        if(ussp->config->broadcast_cb) {
            ussp->config->broadcast_cb(ussp);
        }
        return;
    }

    // standard data transfer. Node number is evaluated
    if((ussp->rxTelegram.adr & 0xE0) == 0 && ((ussp->rxTelegram.adr & 0x1F) == ussp->config->node_nb)) {
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


/***
 *          _       _                                            
 *       __| | __ _| |_ __ _   _ __  _ __ ___   ___ ___  ___ ___ 
 *      / _` |/ _` | __/ _` | | '_ \| '__/ _ \ / __/ _ \/ __/ __|
 *     | (_| | (_| | || (_| | | |_) | | | (_) | (_|  __/\__ \__ \
 *      \__,_|\__,_|\__\__,_| | .__/|_|  \___/ \___\___||___/___/
 *                            |_|                                
 */

void data_process(USSDriver *ussp) {
    // ussp->rxBuffer
    // ussp->lge


}




/***
 *      ____  _             _   
 *     / ___|| |_ __ _ _ __| |_ 
 *     \___ \| __/ _` | '__| __|
 *      ___) | || (_| | |  | |_ 
 *     |____/ \__\__,_|_|   \__|
 *                              
 */


void ussStart(USSDriver *ussp, const USSConfig *usscfg)
{
    // UART driver config
    ussp->uartConfig.txend1_cb = NULL;                // End of transmission buffer callback.
    ussp->uartConfig.txend2_cb = telegram_sent;       //Physical end of transmission callback.
    ussp->uartConfig.rxend_cb = telegram_received;    //Receive buffer filled callback.
    ussp->uartConfig.rxchar_cb = char_received;       //Character received while out if the @p UART_RECEIVE state.
    ussp->uartConfig.rxerr_cb = error_cb;             //Receive error callback.
    ussp->uartConfig.timeout_cb = NULL;
    ussp->uartConfig.timeout = 0;
    ussp->uartConfig.speed = usscfg->speed;
    ussp->uartConfig.cr1 = USART_CR1_PCE;
    ussp->uartConfig.cr2 = USART_CR2_STOP1_BITS | USART_CR2_LINEN;
    ussp->uartConfig.cr3 = 0;

    // GPT driver config
    ussp->gptConfig.frequency = usscfg->speed;
    ussp->gptConfig.callback = residual_timeout_cb;
    ussp->gptConfig.cr2 = 0;
    ussp->gptConfig.dier = 0;

    ussp->config = usscfg;
    usscfg->uartp->ussp = ussp;
    ussp->rxState = USS_RX_STX;
    ussp->txState = USS_TX_IDLE;
    ussp->status = USS_OK;
    chBSemObjectInit(&ussp->tx_sem, true);

    chThdCreateStatic(ussp->waTxThread, sizeof(ussp->waTxThread), NORMALPRIO + 1, txThd, ussp);
    uartStart(ussp->config->uartp, &ussp->uartConfig);
    gptStart(ussp->config->gpt, &ussp->gptConfig);
}
