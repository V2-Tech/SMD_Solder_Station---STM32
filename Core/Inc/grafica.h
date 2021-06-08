/*
 *  Created on: 01/06/2021
 *      Author: V2Tech
 */

#include "main.h"
#include "stm32f1xx_hal.h"
#include "stdbool.h"

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
	OledPage _ActualPage;
	int16_t SignedEncActValue;
} VisualInterface;

// ------------------------- Functions  ----------------------
void TestFPS();

void EncoderRead(VisualInterface* Interface, TIM_HandleTypeDef* EncoderTimer);

void GraphicInit(VisualInterface* Interface);
void MainPageDraw(VisualInterface* Interface);

#endif
