#include "sensors.h"
#include "hal.h"
#include "ch.h"
#include "stdutil++.hpp"
extern "C" {
    #include "i2cPeriphBMP3XX.h"
}

// Digital noise filter: 0 disabled, [0x1 - 0xF] enable up to n t_I2CCLK
#define STM32_CR1_DNF(n)          ((n & 0x0f) << 8)

I2CConfig i2c1_conf = {
    .timingr = 0x0080195B,      // 50ns rise time, 10ns fall time, DNF=2 (au pif)
    .cr1 = STM32_CR1_DNF(2),    // au pif
    .cr2 = 0
};


Bmp3xxDriver bmp3p;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
Bmp3xxConfig bmp3_conf = {
    .i2cp = &I2CD1,
    .slaveAddr = BMP3_ADDR_I2C_PRIM,
    .settings = {
        .op_mode =BMP3_MODE_NORMAL,
        .press_en = BMP3_ENABLE,
        .temp_en = BMP3_ENABLE,
        .odr_filter = {
            .press_os = BMP3_OVERSAMPLING_32X,
            .temp_os = BMP3_OVERSAMPLING_2X,
            .odr = BMP3_ODR_12_5_HZ // BMP3_ODR_6_25_HZ BMP3_ODR_200_HZ
            },
    },
    .settings_sel = BMP3_SEL_PRESS_EN | BMP3_SEL_TEMP_EN | BMP3_SEL_PRESS_OS |
                    BMP3_SEL_TEMP_OS | BMP3_SEL_ODR /* | BMP3_SEL_DRDY_EN*/
};
#pragma GCC diagnostic pop

static THD_WORKING_AREA(waSensors, 1000);

static void sensorsThd(void*) {
    chRegSetThreadName("sensorsThd");

    i2cStart(&I2CD1, &i2c1_conf);
    if(bmp3xxStart(&bmp3p, &bmp3_conf) == MSG_OK) {
        DebugTrace ("bmp init OK");
    } else {
        DebugTrace ("bmp init FAIL");
    }

    while(true) {
        if (bmp3xxFetch(&bmp3p, BMP3_PRESS | BMP3_TEMP) == MSG_OK) {
            DebugTrace("Temp =%.2f, Press=%.2f mB",
            bmp3xxGetTemp(&bmp3p), bmp3xxGetPressure(&bmp3p)/100.0f);
        
        } else {
            DebugTrace ("bmp fetch FAIL");
        }
        chThdSleepMilliseconds(500);
    }

}


float getTemp() {
    return bmp3xxGetTemp(&bmp3p);
}

float getAbsolutePressure() {
    return bmp3xxGetPressure(&bmp3p)/100.0f;
}



void startSensors() {
    chThdCreateStatic(waSensors, sizeof(waSensors), NORMALPRIO + 1, sensorsThd, NULL);
}