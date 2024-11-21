#pragma once
#include "hal.h"
#include "ch.h"

#define USS_STX 0x02

#define USS_RX_BUF_LEN 50


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
    UARTConfig uartConfig;  // UART config
    GPTConfig  gptConfig;   // GPT driver config
    uint32_t speed;         // UART baudrate
    uint8_t node_nb;        // node id
    usscb_t standard_cb;    // standard telegram received callback
    usscb_t broadcast_cb;   // broadcast telegram received callback
    usscb_t special_cb;     // special telegram received callback (probably NULL)
    void* user_data;
} USSConfig;


struct USSDriver_private{
    USSConfig* config;
    USSRxState rxState;
    USSTxState txState;
    USSError status;
    
    uint8_t lge;
    uint8_t adr;
    uint8_t rxBuffer[USS_RX_BUF_LEN];
    uint8_t txBuffer[USS_RX_BUF_LEN];

    binary_semaphore_t tx_sem;
    thread_t tx_thread;
    THD_WORKING_AREA(waTxThread, 512);

    GPTDriver* gpt;
};


void ussStart(USSDriver* ussp, USSConfig* usscfg);
