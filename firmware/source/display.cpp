#include "display.h"
#include "display4DS.h"
#include "sensors.h"


#define RED 100, 0, 0
#define GREEN 0, 100, 0
#define WHITE 100, 100, 100

#define PLOP fds_colorDecTo16b(100, 100, 0)

void displayThd(void*) {
    FdsDriver fds;
    fdsStart(&fds, &SD2, 9600, LINE_LCD_RESET, FDS_PIXXI);

    fdsSetTextFgColorTable(&fds, 1, RED);
    fdsSetTextFgColorTable(&fds, 2, GREEN);

    chThdSleepMilliseconds(20);
    fdsDrawLine(&fds, 0, 110, 240, 110, 2);
    fdsDrawLine(&fds, 0, 155, 240, 155, 2);
    fdsDrawLine(&fds, 0, 200, 240, 200, 2);
    fdsDrawLine(&fds, 72, 110, 72, 240, 2);

    fdsGotoXY(&fds, 0, 10);
    fdsPrintBuffer(&fds, "  diff_p");
    fdsGotoXY(&fds, 0, 14);
    fdsPrintBuffer(&fds, "    temp");
    fdsGotoXY(&fds, 0, 18);
    fdsPrintBuffer(&fds, "pressure");
    
    
    //gfx_circleFilled(&fds, 50, 50, 40, PLOP);
    
    fdsSetTextSizeMultiplier(&fds, 2, 2);
    fdsSetTextFgColor(&fds, WHITE);

    while(true) {
        float temp = getTemp();
        float pressure = getAbsolutePressure();
        float diff_p = getDiffPressure();

        fdsGotoXY(&fds, 5, 5);
        fdsPrintFmt(&fds, "%7.2f Pa", diff_p);
        fdsGotoXY(&fds, 5, 7);
        fdsPrintFmt(&fds, "  %6.2f C", temp);
        fdsGotoXY(&fds, 5, 9);
        fdsPrintFmt(&fds, "%6.1f hPa", pressure);

        chThdSleepMilliseconds(200);
    }
}
