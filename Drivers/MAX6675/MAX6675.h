/*************************************************************************************
 Title	 :  MAXIM Integrated MAX6675 Library for STM32 Using HAL Libraries
 Author  :  Bardia Alikhan Afshar <bardia.a.afshar@gmail.com>
 Software:  STM32CubeIDE
 Hardware:  Any STM32 device
*************************************************************************************/
#ifndef INC_MAX6675_H_
#define INC_MAX6675_H_
#include "main.h"

// ------------------------- Defines -------------------------

// ------------------------- Variable -------------------------
extern float ActTemperature;
extern _Bool TCF;
// ------------------------- Functions  ----------------------
void Max6675_Read_Temp(void);
void Max6675_Read_TempValue(float *tempartureVariable);
void Max6675_Get_TempValue(float* tempartureVariable);
#endif
