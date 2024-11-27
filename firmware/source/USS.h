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
    usscb_t any_cb;         // any telegram received callback
    void* user_data;
} USSConfig;


typedef enum {
    USS_PWE_AK_TASK_NO_TASK,                   // No task
    USS_PWE_AK_TASK_REQUEST_PWE,               // Request PWE
    USS_PWE_AK_TASK_CHANGE_PWE_W,              // Change PWE (word)
    USS_PWE_AK_TASK_CHANGE_PWE_DW,             // Change PWE (double word)
    USS_PWE_AK_TASK_REQUEST_PBE,               // Request PBE element
    USS_PWE_AK_TASK_CHANGE_PBE,                // Change PBE element
    USS_PWE_AK_TASK_REQUEST_PWE_ARRAY,         // Request PWE (array)
    USS_PWE_AK_TASK_CHANGE_PWE_ARRAY_W,        // Change PWE (array word)
    USS_PWE_AK_TASK_CHANGE_PWE_ARRAY_DW,       // Change PWE (array double word) 
    USS_PWE_AK_TASK_REQUEST_NB_ARRAY_ELT,      // Request the number of array elements
    USS_PWE_AK_TASK_RESERVED,                  // Reserve
    USS_PWE_AK_TASK_CHANGE_PWE_ARRAY_DW_STR,   // Change PWE (array double word), and store in the EEPROM 
    USS_PWE_AK_TASK_CHANGE_PWE_ARRAY_W_STR,    // Change PWE (array word), and store in the EEPROM
    USS_PWE_AK_TASK_CHANGE_PWE_DW_STR,         // Change PWE (double word), and store in EEPROM
    USS_PWE_AK_TASK_CHANGE_PWE_W_STR,          // Change PWE (word), and store in the EEPROM
    USS_PWE_AK_TASK_REQUEST_CHANGE_TEXT,       // Request or change text ( only valid for SIMOVERT Master Drives)
} pwe_ak_task_t;

typedef enum {
    USS_PWE_AK_RSP_NO_RESPONSE,                    // No response
    USS_PWE_AK_RSP_TRANSFER_PWE_W,                 // Transfer PWE (word)
    USS_PWE_AK_RSP_TRANSFER_PWE_DW,                // Transfer PWE (double word)
    USS_PWE_AK_RSP_TRANSFER_PBE_ELT,               // Transfer the number of array elements
    USS_PWE_AK_RSP_TRANSFER_PWE_ARRAY_W,           // Transfer PWE (array word)
    USS_PWE_AK_RSP_TRANSFER_PWE_ARRAY_DW,          // Transfer PWE (array double word)
    USS_PWE_AK_RSP_TRANSFER_NB_ARRAY_ELT,          // Transfer number of array elements
    USS_PWE_AK_RSP_TASK_CANNOT_EXEC,               // Task cannot be executed
    USS_PWE_AK_RSP_PKW_READ_ONLY,                  // No PKW parameter change rights
    USS_PWE_AK_RSP_PARAM_CHANGE_REPORT_W,          // Parameter change report (word)
    USS_PWE_AK_RSP_PARAM_CHANGE_REPORT_DW,         // Parameter change report (double word)
    USS_PWE_AK_RSP_PARAM_CHANGE_REPORT_ARRAY_W,    // Parameter change report (array word)
    USS_PWE_AK_RSP_PARAM_CHANGE_REPORT_ARRAY_DW,  // Parameter change report (array double word)
    USS_PWE_AK_RSP_RESERVED_1,                       // Reserve
    USS_PWE_AK_RSP_RESERVED_2,                       // Reserve
    USS_PWE_AK_RSP_TRANSFER_TEXT,                  // Transfer text
} pwe_ak_rps_t;

typedef struct {
    //PKE
    uint16_t pke_ak : 4;
    uint16_t pke_sp : 1;
    uint16_t pke_pnu : 11;
    //IND
    uint16_t ind_dcs:6;
    uint16_t ind_rxte: 2;
    uint16_t ind_low: 8;
} pkw_t;

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
    float gpt_freq;
    USSRxState rxState;
    USSTxState txState;
    USSError status;
    
    Telegram_t rxTelegram;
    Telegram_t txTelegram;

    binary_semaphore_t tx_sem;
    thread_t* tx_thread;
    THD_WORKING_AREA(waTxThread, 512);

    
};

void ussStart(USSDriver* ussp, const USSConfig* usscfg);
void ussStop(USSDriver* ussp);


