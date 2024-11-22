#include "display.h"
#include "display4DS.h"
#include "sensors.h"
#include "sd.h"
#include "stdutil++.hpp"
#include "sdio.h"
#include "uss_logger.h"


#define RED 100, 0, 0
#define GREEN 0, 100, 0
#define WHITE 100, 100, 100
#define BLACK 0, 0, 0


#define YELLOW_16b fds_colorDecTo16b(100, 100, 0)
#define RED_16b fds_colorDecTo16b(100, 0, 0)
#define GRAY_16b fds_colorDecTo16b(20, 20, 20)
#define BLACK_16b fds_colorDecTo16b(0, 0, 0)
#define WHITE_16b fds_colorDecTo16b(100, 100, 100)


#define NOTOUCH 0
#define TOUCH_PRESSED 1
#define TOUCH_RELEASED 2
#define TOUCH_MOVING 3

#define SD_X 230
#define SD_Y 10
PolyPoint GRPH_SD[] = {
    {SD_X-6, SD_Y-7},
    {SD_X+2, SD_Y-7},
    {SD_X+6, SD_Y-2},
    {SD_X+6, SD_Y+7},
    {SD_X-6, SD_Y+7},
    {SD_X-6, SD_Y-7},
};
#define GRPH_SD_LEN (sizeof(GRPH_SD)/sizeof(GRPH_SD[0]))


static void drawLayout(FdsDriver* fds) {
    fdsSetTextSizeMultiplier(fds, 1, 1);
    fdsDrawLine(fds, 0, 120,  240, 120,  2);
    fdsDrawLine(fds, 0, 150, 240, 150, 2);

    fdsDrawLine(fds, 0, 180, 240, 180, 2);
    fdsDrawLine(fds, 0, 210, 240, 210, 2);
    fdsDrawLine(fds, 72, 120, 72, 240, 2);

    //gfx_button(fds, 0, 20, 10, fds_colorDecTo16b(50, 50, 200), fds_colorDecTo16b(100, 100, 100), 0, 1, 1, "test");
    
    gfx_moveTo(fds, 0, 125);
    txt_putStr(fds, "  tunnel", NULL);
    gfx_moveTo(fds, 0, 135);
    txt_putStr(fds, "    temp", NULL);
    gfx_moveTo(fds, 0, 155);
    txt_putStr(fds, "   diff.", NULL);
    gfx_moveTo(fds, 0, 165);
    txt_putStr(fds, "pressure", NULL);
    gfx_moveTo(fds, 0, 185);
    txt_putStr(fds, "   board", NULL);
    gfx_moveTo(fds, 0, 195);
    txt_putStr(fds, "    temp", NULL);
    gfx_moveTo(fds, 0, 215);
    txt_putStr(fds, "absolute", NULL);
    gfx_moveTo(fds, 0, 225);
    txt_putStr(fds, "pressure", NULL);
    
}

static THD_WORKING_AREA(waDisplay, 1000);
void displayThd(void*) {

    char buffer[15];

    FdsDriver fds;
    fdsStart(&fds, &SD2, 300000, LINE_LCD_RESET, FDS_PIXXI);
    fdsEnableTouch(&fds, true);

    fdsSetTextFgColorTable(&fds, 1, RED);
    fdsSetTextFgColorTable(&fds, 2, GREEN);
    fdsSetTextFgColorTable(&fds, 3, WHITE);
    fdsSetTextFgColorTable(&fds, 4, BLACK);

    chThdSleepMilliseconds(20);
    drawLayout(&fds);
    
    fdsSetTextSizeMultiplier(&fds, 2, 2);
    txt_fgColour(&fds,  WHITE_16b, NULL);

    systime_t last_touch_time = 0;

    while(true) {

        if(rtcntDiffNow(last_touch_time) > chTimeMS2I(200)) {
            last_touch_time = chSysGetRealtimeCounterX();

            float tunnel_temp = getTunnelTemp();
            float temp = getTemp();
            float pressure = getAbsolutePressure();
            float diff_p = getDiffPressure();
            float airspeed = getAirspeed();

            fdsSetTextSizeMultiplier(&fds, 3, 3);
            txt_fgColour(&fds,  YELLOW_16b, NULL);
            chsnprintf(buffer, 11, "%6.2f m/s", airspeed);
            gfx_moveTo(&fds, 0, 60);
            txt_putStr(&fds, buffer, NULL);


            fdsSetTextSizeMultiplier(&fds, 2, 2);
            txt_fgColour(&fds,  WHITE_16b, NULL);

            chsnprintf(buffer, 15, "  %6.2f C", tunnel_temp);
            gfx_moveTo(&fds, 80, 125);
            txt_putStr(&fds, buffer, NULL);

            chsnprintf(buffer, 15, "%7.2f Pa", diff_p);
            gfx_moveTo(&fds, 80, 155);
            txt_putStr(&fds, buffer, NULL);
            
            chsnprintf(buffer, 15, "  %6.2f C", temp);
            gfx_moveTo(&fds, 80, 185);
            txt_putStr(&fds, buffer, NULL);
            
            chsnprintf(buffer, 15, "%6.1f hPa", pressure);
            gfx_moveTo(&fds, 80, 215);
            txt_putStr(&fds, buffer, NULL);

            // SD card status
            gfx_rectangleFilled(&fds, SD_X-4, SD_Y-4, SD_X+4, SD_Y+4, BLACK_16b);
            fdsDrawPolyLine(&fds, GRPH_SD_LEN, GRPH_SD, 3);
            if(isLogging()) {
                gfx_circleFilled(&fds, SD_X, SD_Y, 4, RED_16b);
            } else {
                if(!isCardInserted()) {
                    gfx_line(&fds,SD_X-4, SD_Y-4, SD_X+4, SD_Y+4, WHITE_16b);
                    gfx_line(&fds,SD_X-4, SD_Y+4, SD_X+4, SD_Y-4, WHITE_16b);
                }
            }

        }


        switch (fdsTouchGetStatus(&fds))
        {
        case NOTOUCH:
            /* code */
            break;
        case TOUCH_PRESSED:
            gfx_circleFilled(&fds, 120, 10, 5, RED_16b);
            break;
        case TOUCH_RELEASED:
            gfx_circleFilled(&fds, 120, 10, 5, GRAY_16b);
            break;
        case TOUCH_MOVING:
            /* code */
            break;
        }
        

        chThdSleepMilliseconds(5);
    }
}


static THD_WORKING_AREA(waEncoderThd, 1000);
void encoderThd(void*) {
    chRegSetThreadName("encoderThd");
    palEnableLineEvent(LINE_ENC_PUSH, PAL_EVENT_MODE_FALLING_EDGE);

    bool log_state = false;

    while(true) {
        palWaitLineTimeout(LINE_ENC_PUSH, TIME_INFINITE);
        if(log_state) {
            stopLog();
            stopUSSLog();
        } else {
            startLog();
            startUSSLog();
        }
    }
}

void startUI() {
    chThdCreateStatic(waDisplay, sizeof(waDisplay), NORMALPRIO + 1, displayThd, NULL);
    chThdCreateStatic(waEncoderThd, sizeof(waEncoderThd), NORMALPRIO + 1, encoderThd, NULL);
}