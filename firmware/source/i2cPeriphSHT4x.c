#include "i2cPeriphSHT4x.h"
#include "stdutil.h"


#define I2C_TIMOUT_100MS TIME_MS2I(100U)

__attribute__((__unused__)) static void restartI2c(I2CDriver *i2cp)
{
  const I2CConfig *cfg = i2cp->config;
  i2cStop(i2cp);
  chThdSleepMilliseconds(1);
  i2cStart(i2cp, cfg);
  chThdSleepMilliseconds(1);
}

static inline msg_t i2cMasterCacheTransmitTimeout	(I2CDriver *i2cp, i2caddr_t addr,
							 const uint8_t *txbuf, size_t txbytes,
							 uint8_t *rxbuf, size_t	rxbytes,
							 sysinterval_t timeout)
{
  cacheBufferFlush(txbuf, txbytes);
  const msg_t status = i2cMasterTransmitTimeout(i2cp, addr, txbuf, txbytes, rxbuf, rxbytes, timeout);
  if (rxbuf)
    cacheBufferInvalidate(rxbuf, rxbytes);
  return status;
}


static inline msg_t i2cMasterCacheReceiveTimeout 	(I2CDriver *  	i2cp, i2caddr_t  	addr,
							 uint8_t *  	rxbuf, size_t  	rxbytes,
							 sysinterval_t  timeout)
{
  const msg_t status = i2cMasterReceiveTimeout(i2cp, addr, rxbuf, rxbytes, timeout);
  cacheBufferInvalidate(rxbuf, rxbytes);
  return status;
}






typedef struct __attribute__((packed)) {
  uint8_t data[2];
  uint8_t crc;
}  Sht4xDataAtom ;


#define DATA_SIZE 2
typedef struct __attribute__((packed)) {
  Sht4xDataAtom data_atom[DATA_SIZE]; // SN then PN
} Sdp3xRawData ;

static inline uint8_t crc8_poly31_calc (const uint8_t data[], const size_t len);
static bool atomCheck(const Sht4xDataAtom *atom);

void sht4xStart(Sht4xDriver *shtp, I2CDriver *i2cp, const Sht4xAddress addr)
{
  shtp->i2cp = i2cp;
  shtp->slaveAddr = addr;
}


// TODO gerer les timings ???
msg_t  sht4xFetch(Sht4xDriver *shtp)
{
  msg_t status;
  Sdp3xRawData CACHE_ALIGNED(data);

  if(shtp->cmd_issued == SHT4x_SOFT_RESET || shtp->cmd_issued == SHT4X_NO_COMMAND) {
    // nothing to fetch
    return MSG_RESET;
  }
  
#if I2C_USE_MUTUAL_EXCLUSION
  i2cAcquireBus(shtp->i2cp);
#endif
  status = i2cMasterCacheReceiveTimeout(shtp->i2cp, shtp->slaveAddr,
				   (uint8_t *) &data, sizeof(data),
				   I2C_TIMOUT_100MS);
  
  if (status != MSG_OK) {
    restartI2c(shtp->i2cp);
#if I2C_USE_MUTUAL_EXCLUSION
    i2cReleaseBus(shtp->i2cp);
#endif
    return status;
  }
#if I2C_USE_MUTUAL_EXCLUSION
  i2cReleaseBus(shtp->i2cp);
#endif
  
  for (size_t i=0; i<DATA_SIZE; i++) {
    if (!(atomCheck(&data.data_atom[i])))
      return  MSG_RESET;
  }

  switch (shtp->cmd_issued)
  {
    case SHT4x_READ_IDENT:
        shtp->ident.sn0 = __builtin_bswap16(*(uint16_t*)data.data_atom[0].data);
        shtp->ident.sn0 = __builtin_bswap16(*(uint16_t*)data.data_atom[1].data);
    break;

    case SHT4x_SOFT_RESET:
    break;

    default:
    {
        uint16_t st = __builtin_bswap16(*(uint16_t*)data.data_atom[0].data);
        uint16_t srh = __builtin_bswap16(*(uint16_t*)data.data_atom[0].data);

        shtp->temp = -45.0 + 175.0*st/((float)((1<<16) - 1));
        shtp->rh = -6.0 + 125.0*srh/((float)((1<<16) - 1));
    }
    break;
  }

  shtp->cmd_issued = SHT4X_NO_COMMAND;
  
  return MSG_OK;
}



msg_t sht4xSend(Sht4xDriver *shtp, const Sht4xCommand cmd)
{
#if I2C_USE_MUTUAL_EXCLUSION
  i2cAcquireBus(shtp->i2cp);
#endif
#if defined(__DCACHE_PRESENT) && __DCACHE_PRESENT != 0
  const Sht4xCommand CACHE_ALIGNED(alCmd) = cmd;
  msg_t status = i2cMasterCacheTransmitTimeout(shtp->i2cp, shtp->slaveAddr,
					       (uint8_t *) &alCmd, sizeof(alCmd),
					       NULL, 0, I2C_TIMOUT_100MS) ;
#else
  msg_t status = i2cMasterCacheTransmitTimeout(sdpp->i2cp, sdpp->slaveAddr,
					       (uint8_t *) &cmd, sizeof(cmd),
					       NULL, 0, I2C_TIMOUT_100MS) ;
#endif
  if (status != MSG_OK) 
    restartI2c(shtp->i2cp);
  
#if I2C_USE_MUTUAL_EXCLUSION
  i2cReleaseBus(shtp->i2cp);
#endif

  if(status == MSG_OK) {
    shtp->cmd_issued = cmd;
  }
  
  return status;
}





__attribute__ ((pure))
static inline uint8_t crc8_poly31_calc(const uint8_t data[], const size_t len) {
  static const uint8_t crc8_poly31[256] =
    { 0x00, 0x31, 0x62, 0x53, 0xC4, 0xF5, 0xA6, 0x97, 0xB9, 0x88, 0xDB, 0xEA, 0x7D, 0x4C, 0x1F,
      0x2E, 0x43, 0x72, 0x21, 0x10, 0x87, 0xB6, 0xE5, 0xD4, 0xFA, 0xCB, 0x98, 0xA9, 0x3E, 0x0F,
      0x5C, 0x6D, 0x86, 0xB7, 0xE4, 0xD5, 0x42, 0x73, 0x20, 0x11, 0x3F, 0x0E, 0x5D, 0x6C, 0xFB,
      0xCA, 0x99, 0xA8, 0xC5, 0xF4, 0xA7, 0x96, 0x01, 0x30, 0x63, 0x52, 0x7C, 0x4D, 0x1E, 0x2F,
      0xB8, 0x89, 0xDA, 0xEB, 0x3D, 0x0C, 0x5F, 0x6E, 0xF9, 0xC8, 0x9B, 0xAA, 0x84, 0xB5, 0xE6,
      0xD7, 0x40, 0x71, 0x22, 0x13, 0x7E, 0x4F, 0x1C, 0x2D, 0xBA, 0x8B, 0xD8, 0xE9, 0xC7, 0xF6,
      0xA5, 0x94, 0x03, 0x32, 0x61, 0x50, 0xBB, 0x8A, 0xD9, 0xE8, 0x7F, 0x4E, 0x1D, 0x2C, 0x02,
      0x33, 0x60, 0x51, 0xC6, 0xF7, 0xA4, 0x95, 0xF8, 0xC9, 0x9A, 0xAB, 0x3C, 0x0D, 0x5E, 0x6F,
      0x41, 0x70, 0x23, 0x12, 0x85, 0xB4, 0xE7, 0xD6, 0x7A, 0x4B, 0x18, 0x29, 0xBE, 0x8F, 0xDC,
      0xED, 0xC3, 0xF2, 0xA1, 0x90, 0x07, 0x36, 0x65, 0x54, 0x39, 0x08, 0x5B, 0x6A, 0xFD, 0xCC,
      0x9F, 0xAE, 0x80, 0xB1, 0xE2, 0xD3, 0x44, 0x75, 0x26, 0x17, 0xFC, 0xCD, 0x9E, 0xAF, 0x38,
      0x09, 0x5A, 0x6B, 0x45, 0x74, 0x27, 0x16, 0x81, 0xB0, 0xE3, 0xD2, 0xBF, 0x8E, 0xDD, 0xEC,
      0x7B, 0x4A, 0x19, 0x28, 0x06, 0x37, 0x64, 0x55, 0xC2, 0xF3, 0xA0, 0x91, 0x47, 0x76, 0x25,
      0x14, 0x83, 0xB2, 0xE1, 0xD0, 0xFE, 0xCF, 0x9C, 0xAD, 0x3A, 0x0B, 0x58, 0x69, 0x04, 0x35,
      0x66, 0x57, 0xC0, 0xF1, 0xA2, 0x93, 0xBD, 0x8C, 0xDF, 0xEE, 0x79, 0x48, 0x1B, 0x2A, 0xC1,
      0xF0, 0xA3, 0x92, 0x05, 0x34, 0x67, 0x56, 0x78, 0x49, 0x1A, 0x2B, 0xBC, 0x8D, 0xDE, 0xEF,
      0x82, 0xB3, 0xE0, 0xD1, 0x46, 0x77, 0x24, 0x15, 0x3B, 0x0A, 0x59, 0x68, 0xFF, 0xCE, 0x9D,
      0xAC };
  uint8_t crc = 0xff;
  
  for (size_t i=0; i<len; ++i)
    crc = crc8_poly31[crc ^ data[i]];
  
  return crc;
}

static bool atomCheck(const Sht4xDataAtom *atom)
{
  return crc8_poly31_calc(atom->data, sizeof(atom->data)) == atom->crc;
}
