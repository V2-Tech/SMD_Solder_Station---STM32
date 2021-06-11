/*
 *  Created on: 01/06/2021
 *      Author: V2Tech
 */

#include "grafica.h"
#include "u8g2/u8g2.h"
u8g2_t u8g2;
_Bool BlinkVar = 0;

/*
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
*/

void EncoderRead(VisualInterface* Interface, TIM_HandleTypeDef* EncoderTimerHandler)
{
	uint32_t EncActValue = EncoderTimerHandler->Instance->CNT;
	Interface->SignedEncActValue = (int16_t) EncActValue;
}

void BlinkTimerCallback(void const * argument)
{
	BlinkVar = !BlinkVar;
}

void Graphic(VisualInterface* Interface)
{
	u8g2_ClearBuffer(&u8g2);
	switch(Interface->_ActualPage)
	{
		case PageMain:
			MainPage(Interface);
			break;
		case PageOptions:
			break;
	}
	u8g2_SendBuffer(&u8g2);
}

void MainPage(VisualInterface* Interface)
{
	/* Draw target temperature icon */
	u8g2_DrawXBMP(&u8g2, 2, 0, 22, 14, temperature_target_22x14);

	/* Draw actual temperature icon */
	u8g2_DrawXBMP(&u8g2, 100, 20, 23, 39, temperature_23x39);

	/* Draw alarm icon */
	if (Interface->AlarmState || 1)
	{
		u8g2_DrawXBMP(&u8g2, 112, 0, 14, 14, alarm_icon14x14);
	}

	/* Draw heat icon */
	if ((Interface->_ActualHeatState == HeatStatyState) ||
			((Interface->_ActualHeatState == Heating || 1 ) && (BlinkVar == 1)))
	{
		u8g2_DrawXBMP(&u8g2, 2, 26, 30, 28, Heater_30x28);
	}

	/* Update value */
	/*
	u8g2_SetFont(&u8g2, u8g2_font_ncenB10_tr);
	uint8_t tempHeigth = u8g2_GetFontBBXHeight(&u8g2);
	u8g2_DrawStr(&u8g2, 1, tempHeigth, ScreenString[0]);
	*/
	Interface->_ActualPage = PageMain;
}
