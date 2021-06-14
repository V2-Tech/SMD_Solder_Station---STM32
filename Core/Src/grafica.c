/*
 *  Created on: 01/06/2021
 *      Author: V2Tech
 */

#include "grafica.h"
#include "u8g2/u8g2.h"

u8g2_t u8g2;
_Bool BlinkVar = 0;
_Bool AlarmVar = 0;

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
	static uint32_t EncLastValue;

	// Encoder counts reading
	uint32_t EncActValue = EncoderTimerHandler->Instance->CNT;
	Interface->SignedEncActValue = (int16_t) EncActValue;
	if (EncActValue != EncLastValue)
	{
		EncLastValue = EncActValue;

	}

	// Software counts limit
	if (Interface->SignedEncActValue<0)
	{
		  __HAL_TIM_SET_COUNTER(EncoderTimerHandler, 0);
	}
	if (Interface->SignedEncActValue>MAXTEMPERATURE)
	{
		  __HAL_TIM_SET_COUNTER(EncoderTimerHandler, MAXTEMPERATURE);
	}
}

void BlinkTimerCallback(void const * argument)
{
	BlinkVar = !BlinkVar;
}

void Graphic(VisualInterface* Interface, LPFilter *filter, PID *pid)
{
	u8g2_ClearBuffer(&u8g2);
	switch(Interface->_ActualPage)
	{
		case PageMain:
			MainPage(Interface, filter, pid);
			break;
		case PageOptions:
			break;
	}
	u8g2_SendBuffer(&u8g2);
}

void MainPage(VisualInterface* Interface, LPFilter *filter, PID *pid)
{
	char ScreenString[2][8];

	//*********************************
	//************* ICONS *************
	//*********************************

	// Draw target temperature icon
	u8g2_DrawXBMP(&u8g2, 2, 0, 22, 14, temperature_target_22x14);

	// Draw actual temperature icon
	u8g2_DrawXBMP(&u8g2, 1, 20, 23, 39, temperature_23x39);

	// Draw alarm icon
	if (AlarmVar)
	{
		u8g2_DrawXBMP(&u8g2, 110, 0, 14, 14, alarm_14x14);
	}

	// Draw heat icon
	if ((Interface->_ActualHeatState == HeatStatyState) ||
			((Interface->_ActualHeatState == Heating) && (BlinkVar == 1)))
	{
		u8g2_DrawXBMP(&u8g2, 96, 26, 30, 28, Heater_30x28);
	}

	//*********************************
	//************ VALUES *************
	//*********************************

	// Actual temperature value
	u8g2_SetFont(&u8g2, u8g2_font_helvR14_tf);
	if (filter->FilterOK && !AlarmVar)
	{
		sprintf((uint8_t *)ScreenString[0], "%3.0f C",filter->FilteredValue);
		u8g2_DrawStr(&u8g2, 32, 24, ScreenString[0]);
	}
	else if (AlarmVar)
	{
		sprintf((uint8_t *)ScreenString[0], "???.? C");
		u8g2_DrawStr(&u8g2, 32, 24, ScreenString[0]);
	}
	else
	{
		sprintf((uint8_t *)ScreenString[0], "Calc...");
		u8g2_DrawStr(&u8g2, 32, 24, ScreenString[0]);
	}

	// Setpoint value
	u8g2_SetFont(&u8g2, u8g2_font_helvR10_tf);
	if (Interface->SignedEncActValue <= MAXTEMPERATURE && Interface->SignedEncActValue >= 0)
	{
		if (Interface->SignedEncActValue == pid->Setpoint)
		{
			sprintf(ScreenString[0], "%3.0f C", pid->Setpoint);
			u8g2_DrawStr(&u8g2, 26, 0, ScreenString[0]);
		}
		else
		{
			if (BlinkVar == 1)
			{
				sprintf(ScreenString[0], "%d C", Interface->SignedEncActValue);
				u8g2_DrawStr(&u8g2, 26, 0, ScreenString[0]);
			}
		}
	}
	if (Interface->_PulsEncoderPressed)
	{
		Interface->_PulsEncoderPressed = false;
		PIDNewSetpoint(pid, Interface->SignedEncActValue);
		Interface->_TargetChanging = false;
	}

	Interface->_ActualPage = PageMain;
}
