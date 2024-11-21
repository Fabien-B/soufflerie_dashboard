#pragma once
#include "hal.h"
#include "ch.h"

#define USS_STX 0x02

#define USS_TELEGRAM_LEN 50


typedef struct USSDriver_private USSDriver;
typedef void (*usscb_t)(USSDriver *ussp);


typedef enum {
    USS_RX_STX,     // wait for STX
    USS_RX_LGE,     // wait for LGE
    USS_RX_RESIDUAL // wait for the rest of the telegram (LGE bytes)
} USSRxState;

typedef enum {
    USS_TX_IDLE,        // not sending, nothing to send
    USS_TX_RSP_WAIT,    // wait for response delay to elapse before sending response
    USS_TX_SENDING,     // transmission pending
} USSTxState;

typedef enum {
    USS_OK,
    USS_BCC_MISMATCH,
} USSError;


typedef struct {
    UARTDriver* uartp;      // UART driver
    GPTDriver* gpt;
    uint32_t speed;         // UART baudrate
    uint8_t node_nb;        // node id
    usscb_t standard_cb;    // standard telegram received callback
    usscb_t broadcast_cb;   // broadcast telegram received callback
    usscb_t special_cb;     // special telegram received callback (probably NULL)
    void* user_data;
} USSConfig;


typedef struct {
    uint8_t stx;
    uint8_t lge;
    uint8_t adr;
    uint8_t data[USS_TELEGRAM_LEN];
} __attribute__((packed)) Telegram_t;

struct USSDriver_private{
    const USSConfig* config;
    UARTConfig uartConfig;  // UART config
    GPTConfig  gptConfig;   // GPT driver config
    USSRxState rxState;
    USSTxState txState;
    USSError status;
    
    Telegram_t rxTelegram;
    Telegram_t txTelegram;

    binary_semaphore_t tx_sem;
    thread_t tx_thread;
    THD_WORKING_AREA(waTxThread, 512);

    
};


void ussStart(USSDriver* ussp, USSConfig* usscfg);
