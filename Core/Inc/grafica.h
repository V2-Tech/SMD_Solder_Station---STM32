/*
 *  Created on: 01/06/2021
 *      Author: V2Tech
 */
/*
 * OLED SSD1306 Yellow-Blue details:
 * - Pixel 128x64: 14Y-px yellow, 50Y-px blue.
 * - I2C connection.
 */

#include "main.h"
#include "stm32f1xx_hal.h"
#include "stdbool.h"
#include "V2Tech_128x64_iconset.h"

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GRAFICA_H
#define __GRAFICA_H

// ------------------------- Defines -------------------------
#define LINESPACE 2

// ------------------------- Variable ------------------------
typedef enum {
	PageMain = 0,
	PageOptions= 1,
} OledPage;

typedef struct
{
	//Public
	int16_t SignedEncActValue;
	_Bool AlarmState;

	//Private
	OledPage _FirstPage;
	OledPage _ActualPage;
	int8_t _ActualHeatMode;//0=Manual, 1=Automatic
	uint8_t _ActualHeatState;//0=OFF, 1=Heating, 2=Steady-State
} VisualInterface;

// ------------------------- Functions  ----------------------
void TestFPS();

void EncoderRead(VisualInterface* Interface, TIM_HandleTypeDef* EncoderTimer);

void GraphicInit(VisualInterface* Interface);
void MainPage(VisualInterface* Interface);

#endif
