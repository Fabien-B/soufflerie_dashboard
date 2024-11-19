#pragma once
#include "hal.h"


/**
 * @name    I2C slave number
 * @brief   SHT4x can be configured to have one of three I²C slave number
 * @{
 */
typedef enum __attribute__ ((__packed__)) {
    SHT4X_ADDRESS1 =0x44,   /**< @brief  SHT4x-A    */
    SHT4X_ADDRESS2,	        /**< @brief  SHT4x-B 	*/	 
    SHT4X_ADDRESS3	        /**< @brief  SHT4x-C	*/  
    } Sht4xAddress;
/** @} */




/**
 * @name    I2C command for regular transation
 * @brief   can be used directly by sdp3xSend function for advanced use
 * @brief   read datasheet for details
 * @{
 */
typedef enum __attribute__ ((__packed__)) {
    SHT4X_NO_COMMAND = 0x0,
    SHT4x_TEMP_RH_HI = 0xFD,    /**< @brief  measure T and RH with high precision   --> 8.3ms   */
    SHT4x_TEMP_RH_MD = 0xF6,    /**< @brief  measure T and RH with medium precision --> 4.5ms  */
    SHT4x_TEMP_RH_LO = 0xE0,    /**< @brief  measure T and RH with low precision    --> 1.6ms */
    SHT4x_READ_IDENT = 0x89,    /**< @brief  read serial number    */
    SHT4x_SOFT_RESET = 0x94,    /**< @brief  soft reset   --> 1ms */
    SHT4x_HEAT_200_1S = 0x39,   /**< @brief  active heater with 200mW for 1s    */
    SHT4x_HEAT_200_01S = 0x32,  /**< @brief  active heater with 200mW for 0.1s    */
    SHT4x_HEAT_110_1S = 0x2F,   /**< @brief  active heater with 110mW for 1s    */
    SHT4x_HEAT_110_01S = 0x24,  /**< @brief  active heater with 110mW for 0.1s    */
    SHT4x_HEAT_20_1S = 0x1E,    /**< @brief  active heater with 20mW for 1s    */
    SHT4x_HEAT_20_01S = 0x15,   /**< @brief  active heater with 20mW for 0.1s    */
    }  Sht4xCommand;
/** @} */


/**
 * @name    structure containing identification data : part number and serial number
 * @brief   read datasheet for details
 * @{
 */
typedef struct {
  uint16_t sn0;
  uint16_t sn1;
} Sht4xIdent;
/** @} */



/**
 * @name    handler of driver
 * @{
 */
typedef struct Sht4xDriver Sht4xDriver; /**< @brief  opaque type */
/** @} */



struct Sht4xDriver {
  I2CDriver    *i2cp;
  Sht4xAddress  slaveAddr;
  float		temp;           /**< @brief  temperature en degrees celcius   */
  float		rh;             /**< @brief  relative humidity in percents   */
  Sht4xIdent ident;
  Sht4xCommand cmd_issued;
};

void sht4xStart(Sht4xDriver *shtp, I2CDriver *i2cp, const Sht4xAddress addr);

msg_t sht4xSend(Sht4xDriver *shtp, const Sht4xCommand cmd);

msg_t  sht4xFetch(Sht4xDriver *shtp);


static inline float sht4xGetTemp(Sht4xDriver *shtp) {
    return shtp->temp;
}

static inline float sht4xGetRH(Sht4xDriver *shtp) {
    return shtp->rh;
}

static inline Sht4xIdent sht4xGetIdent(Sht4xDriver *shtp) {
    return shtp->ident;
}
