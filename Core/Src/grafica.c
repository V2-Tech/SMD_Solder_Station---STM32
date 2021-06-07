/*
 *  Created on: 01/06/2021
 *      Author: V2Tech
 */

#include "grafica.h"
#include "u8g2/u8g2.h"
u8g2_t u8g2;

void TestFPS() {
    u8g2_DrawBox(&u8g2, 0, 0, 128, 64);
    u8g2_SendBuffer(&u8g2);
    u8g2_ClearBuffer(&u8g2);
    uint32_t start = HAL_GetTick();
    uint32_t end = start;
    int fps = 0;
    char message[] = "ABCDEFGHIJK";
    u8g2_SetFont(&u8g2, u8g2_font_ncenB10_tr);
    u8g2_DrawStr(&u8g2, 2, 30, message);
    u8g2_SendBuffer(&u8g2);

    do {
    	u8g2_ClearBuffer(&u8g2);
    	u8g2_DrawStr(&u8g2, 2, 18, message);
        u8g2_SendBuffer(&u8g2);

        char ch = message[0];
        memmove(message, message+1, sizeof(message)-2);
        message[sizeof(message)-2] = ch;

        fps++;
        end = HAL_GetTick();
    } while((end - start) < 5000);


    char buff[64];
    fps = (float)fps / ((end - start) / 1000.0);
    snprintf(buff, sizeof(buff), "~%d FPS", fps);

    u8g2_ClearBuffer(&u8g2);
    u8g2_DrawStr(&u8g2, 2, 18, buff);
    u8g2_SendBuffer(&u8g2);

    osDelay(2000);
}
